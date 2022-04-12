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
import map_tools
import shapely.geometry as geom
import descartes as dc
import numpy as np

context = zmq.Context()

pub_socket = context.socket(zmq.PUB)
pub_socket.connect("ipc:///tmp/robotics/pub.ipc")

sub_socket = context.socket(zmq.SUB)
sub_socket.connect("ipc:///tmp/robotics/sub.ipc")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"wheel_speeds")

robot_width = 0.1
robot_length = 0.2
robot_radius = 0.05
robot1 = dd.DifferentialDrive(robot_width, robot_length, robot_radius)
robot2 = dd.DifferentialDrive(robot_width, robot_length, robot_radius)
timestep = 0.02


def load_map(filename):
    obstacles = []
    with open(filename) as file:
        obj = json.load(file)
    for obs_pts in obj["obstacles"]:
        poly = geom.Polygon(obs_pts)
        obstacles.append(poly)
    start = obj["start"]
    goal = obj["goal"]

    return obstacles, start, goal


obstacles, start, goal = load_map("map1.json")

landmarks = []
for obs in obstacles:
    landmarks += obs.boundary.coords
landmarks = np.array(landmarks)

fig, [ax1, ax2] = plt.subplots(1, 2)

for obs in obstacles:
    patch = dc.PolygonPatch(obs)
    ax1.add_patch(patch)
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

    num_rays = 20
    max_dist = 0.5
    min_angle = -np.pi/2
    max_angle = np.pi/2
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
    sigma = 0.5
    omega1 += np.random.normal(0, sigma)
    omega2 += np.random.normal(0, sigma)
    return omega1, omega2


def visible_landmarks(state, landmarks):
    landmark_diffs = landmarks-np.array([state[0], state[1]])
    landmark_dists = np.linalg.norm(landmark_diffs, axis=1)
    landmark_thetas = np.arctan2(landmark_diffs[:, 1], landmark_diffs[:, 0])

    vis = np.where((landmark_dists < 0.5) & (
        landmark_thetas > -np.pi/2) & (landmark_thetas < np.pi/2))
    return vis[0], landmark_dists[vis[0]], landmark_thetas[vis[0]]


def producer():
    global state
    state = [start[0], start[1], 0]
    omega1 = 5
    omega2 = 5.6
    # timestemp = 0.02
    count = 0
    while True:
        #  Wait for next request from client
        try:
            topic, message_str = sub_socket.recv_multipart(flags=zmq.NOBLOCK)
            message_json = json.loads(message_str)
            omega1 = message_json["omega1"]
            omega2 = message_json["omega2"]
            # print(omega1, omega2)
        except zmq.ZMQError:
            # print("nothing received")
            pass

        # if new controls info have arrived
        #   set the controls values
        # simulate one time step
        # yield the result

        noisy_omega1, noisy_omega2 = add_wheel_noise(omega1, omega2)

        res = solve_ivp(robot1.deriv, [0, timestep],
                        state, args=[[noisy_omega1, noisy_omega2], 0])

        if not robot1.collides(state, obstacles):
            state = res.y[:, -1]
        else:
            pub_socket.send_multipart([b"collision", b"{}"])
            pass

        lines = lidar(state, obstacles)
        dists = [line.length for line in lines]

        marks, mark_dists, mark_thetas = visible_landmarks(state, landmarks)
        if len(marks) > 0:
            # print([index[0] for index in marks])
            marks_dict = {}
            for index, dist, theta in zip(marks, mark_dists, mark_thetas):
                mark = landmarks[index]
                marks_dict[int(index)] = {}
                marks_dict[int(index)]["dist"] = dist + \
                    np.random.normal(0, 0.05)
                marks_dict[int(index)]["theta"] = theta + \
                    np.random.normal(0, 0.01)

            print(marks_dict)
            marks_str = json.dumps(marks_dict).encode()
            pub_socket.send_multipart([b"landmarks", marks_str])

        state_dict = {"x": state[0], "y": state[1], "theta": state[2]}
        state_str = json.dumps(state_dict).encode()
        pub_socket.send_multipart([b"state", state_str])

        lidar_dict = {"distances": dists}
        lidar_str = json.dumps(lidar_dict).encode()
        pub_socket.send_multipart([b"lidar", lidar_str])

        # print(count)
        count += 1
        yield state, lines


state_0 = [start[0], start[1], 0]
patches1 = robot1.draw(ax1, state_0)
for patch in patches1:
    ax1.add_patch(patch)

patches2 = robot2.draw(ax2, state_0)
for patch in patches2:
    ax2.add_patch(patch)


drawn_lines1 = []
for line in lidar(state_0, obstacles):
    x, y = line.xy
    l = ax1.plot(x, y, 'g')[0]
    drawn_lines1.append(l)

drawn_lines2 = []
for line in lidar(state_0, obstacles):
    x, y = line.xy
    l = ax2.plot(x, y, 'g')[0]
    drawn_lines2.append(l)


ax1.axis("equal")
ax1.grid("on")

ax2.axis("equal")
ax2.grid("on")
ax2.set_xlim(-20, 20)
ax2.set_ylim(-20, 20)


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
    window_size = 3
    ax1.set_xlim(state[0]-window_size, state[0]+window_size)
    ax1.set_ylim(state[1]-window_size, state[1]+window_size)
    return *patches1, *patches2, *drawn_lines1, *drawn_lines2


animation = anim.FuncAnimation(
    fig, animate, producer, interval=timestep*1000)

plt.show(block=True)
