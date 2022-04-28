#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 09:00:06 2022

@author: jonathan
"""

import zmq
import time
import differential_drive as dd
import matplotlib.animation as anim
import matplotlib.pyplot as plt
import json
from scipy.integrate import solve_ivp
import shapely.geometry as geom
import descartes as dc
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("map", help="The path to the map file to be loaded")
args = parser.parse_args()

map_name = args.map

context = zmq.Context()

pub_socket = context.socket(zmq.PUB)
# pub_socket.connect("ipc:///tmp/robo_sim/pub.ipc")
pub_socket.connect("tcp://localhost:5557")

sub_socket = context.socket(zmq.SUB)
# sub_socket.connect("ipc:///tmp/robo_sim/sub.ipc")
sub_socket.connect("tcp://localhost:5555")

sub_socket.setsockopt(zmq.SUBSCRIBE, b"wheel_speeds")

robot_width = 0.1
robot_length = 0.2
robot_radius = 0.05
robot1 = dd.DifferentialDrive(robot_width, robot_length, robot_radius)
robot2 = dd.DifferentialDrive(robot_width, robot_length, robot_radius)
timestep = 0.02

lidar_num_rays = 20
lidar_max_dist = 0.5
lidar_min_angle = -np.pi/2
lidar_max_angle = np.pi/2

wheel_noise_a1 = 0.01
wheel_noise_a2 = 0
wheel_noise_a3 = 0
wheel_noise_a4 = 0.01

landmark_range_sigma = 0.05
landmark_bearing_sigma = 0.01


def load_map(filename):
    obstacles = []
    with open(filename) as file:
        obj = json.load(file)
    for obs_pts in obj["obstacles"]:
        poly = geom.Polygon(obs_pts)
        obstacles.append(poly)
    start = obj["start"]
    goal = obj["goal"]
    bounds = obj["bounds"]

    return obstacles, start, goal, bounds


obstacles, start, goal, bounds = load_map(map_name)

landmarks = []
landmarks.append(goal)
for obs in obstacles:
    landmarks += obs.boundary.coords
landmarks = np.array(landmarks)

fig, [ax1, ax2] = plt.subplots(1, 2)

obs_patches1 = []
for obs in obstacles:
    patch = dc.PolygonPatch(obs)
    ax1.add_patch(patch)
    obs_patches1.append(patch)
    patch = dc.PolygonPatch(obs)
    ax2.add_patch(patch)


ax1.plot(start[0], start[1], 'g.')
ax1.plot(goal[0], goal[1], 'r.')

ax2.plot(start[0], start[1], 'g.')
ax2.plot(goal[0], goal[1], 'r.')


def lidar(state, obstacles):
    x = state[0]
    y = state[1]
    theta = state[2]

    num_rays = lidar_num_rays
    max_dist = lidar_max_dist
    min_angle = lidar_min_angle
    max_angle = lidar_max_angle
    angles = np.linspace(min_angle, max_angle, num_rays)

    lines = []
    for angle in angles:
        end_x = max_dist*np.cos(theta+angle)+x
        end_y = max_dist*np.sin(theta+angle)+y
        l = geom.LineString([[x, y], [end_x, end_y]])
        for obs in obstacles:
            l_temp = l.difference(obs)
            if isinstance(l_temp, geom.MultiLineString):
                l = l_temp[0]
            else:
                l = l_temp
        lines.append(l)
    return lines


def add_wheel_noise(omega1, omega2):
    sigma1 = np.sqrt(wheel_noise_a1*omega1**2+wheel_noise_a2*omega2**2)
    sigma2 = np.sqrt(wheel_noise_a3*omega1**2+wheel_noise_a4*omega2**2)
    omega1 += np.random.normal(0, sigma1)
    omega2 += np.random.normal(0, sigma2)
    return omega1, omega2


def visible_landmarks(state, landmarks):
    landmark_diffs = landmarks-np.array([state[0], state[1]])
    landmark_dists = np.linalg.norm(landmark_diffs, axis=1)
    landmark_thetas = np.arctan2(
        landmark_diffs[:, 1], landmark_diffs[:, 0]) - state[2]

    landmark_thetas = np.remainder(landmark_thetas + np.pi, 2*np.pi) - np.pi

    vis = np.where((landmark_dists < 0.5) & (
        landmark_thetas > -np.pi/2) & (landmark_thetas < np.pi/2))
    return vis[0], landmark_dists[vis[0]], landmark_thetas[vis[0]]


def producer():
    global state
    state = [start[0], start[1], 0]
    omega1 = 0
    omega2 = 0
    count = 0
    t = 0
    while True:
        #  Get the newest message
        try:
            topic, message_str = sub_socket.recv_multipart(flags=zmq.NOBLOCK)
            queue_empty = False
            queue_count = 1
            while not queue_empty:
                try:
                    topic, message_str = sub_socket.recv_multipart(
                        flags=zmq.NOBLOCK)
                    queue_count += 1
                except zmq.ZMQError:
                    queue_empty = True
                    if queue_count > 1:
                        print("Dropped {} old messages from queue. Receiving more than one message per timestep".format(
                            queue_count-1))
            message_dict = json.loads(message_str.decode())
            omega1 = message_dict["omega1"]
            omega2 = message_dict["omega2"]
            # print(omega1, omega2)
        except zmq.ZMQError:
            print("nothing received")
            pass

        noisy_omega1, noisy_omega2 = add_wheel_noise(omega1, omega2)

        res = solve_ivp(robot1.deriv, [0, timestep],
                        state, args=[[noisy_omega1, noisy_omega2], 0])
        t += timestep

        if not robot1.collides(state, obstacles):
            state = res.y[:, -1]
            collision_dict = {"timestamp": t, "collision": False}
        else:
            collision_dict = {"timestamp": t, "collision": True}
            pass
        collision_str = json.dumps(collision_dict).encode()
        pub_socket.send_multipart([b"collision", collision_str])

        lines = lidar(state, obstacles)
        dists = [line.length for line in lines]

        marks, mark_dists, mark_thetas = visible_landmarks(state, landmarks)
        message_dict = {"timestamp": t}
        # print([index[0] for index in marks])
        marks_dict = {}
        for index, dist, theta in zip(marks, mark_dists, mark_thetas):
            mark = landmarks[index]
            marks_dict[int(index)] = {}
            marks_dict[int(index)]["dist"] = dist + \
                np.random.normal(0, landmark_range_sigma)
            marks_dict[int(index)]["theta"] = theta + \
                np.random.normal(0, landmark_bearing_sigma)

            # print(marks_dict)
        message_dict["landmarks"] = marks_dict
        marks_str = json.dumps(message_dict).encode()
        pub_socket.send_multipart([b"landmarks", marks_str])

        lidar_dict = {"timestamp": t, "distances": dists}
        lidar_str = json.dumps(lidar_dict).encode()
        pub_socket.send_multipart([b"lidar", lidar_str])

        state_dict = {"timestamp": t,
                      "x": state[0], "y": state[1], "theta": state[2]}
        state_str = json.dumps(state_dict).encode()
        pub_socket.send_multipart([b"state", state_str])

        # print(count)
        count += 1
        yield state, lines


# Draw the initial state with a zoomed-in and zoomed-out view
state = [start[0], start[1], 0]
patches1 = robot1.draw(ax1, state)
for patch in patches1:
    ax1.add_patch(patch)

patches2 = robot2.draw(ax2, state)
for patch in patches2:
    ax2.add_patch(patch)


drawn_lines1 = []
for line in lidar(state, obstacles):
    x, y = line.xy
    l = ax1.plot(x, y, 'g')[0]
    drawn_lines1.append(l)

drawn_lines2 = []
for line in lidar(state, obstacles):
    x, y = line.xy
    l = ax2.plot(x, y, 'g')[0]
    drawn_lines2.append(l)


ax1.axis("equal")
ax1.xaxis.set_ticklabels([])
ax1.yaxis.set_ticklabels([])

ax2.axis("equal")
ax2.grid("on")
ax2.set_xlim(bounds[0]-1, bounds[1]+1)
ax2.set_ylim(bounds[0]-1, bounds[1]+1)

# Start the animation. It pulls new data from the producer function and passes
# it to the animate function to update the visuals


def animate(data):
    state, lines = data
    patches1 = robot1.draw(ax1, state)
    patches2 = robot2.draw(ax2, state)

    for line, drawn_line in zip(lines, drawn_lines1):
        try:
            x, y = line.xy
            drawn_line.set_data(x, y)
        except NotImplementedError:
            print(line)
    for line, drawn_line in zip(lines, drawn_lines2):
        try:
            x, y = line.xy
            drawn_line.set_data(x, y)
        except NotImplementedError:
            print(line)
    window_size = 1
    ax1.set_xlim(state[0]-window_size, state[0]+window_size)
    ax1.set_ylim(state[1]-window_size, state[1]+window_size)
    return *obs_patches1, *patches1, *patches2, *drawn_lines1, *drawn_lines2


time_scale = 1  # Make this number bigger to slow down time
animation = anim.FuncAnimation(
    fig, animate, producer, interval=timestep*1000*time_scale, blit=True)

plt.show(block=True)
