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
robot = dd.DifferentialDrive(robot_width, robot_length, robot_radius)
timestep = 0.02

obstacles = map_tools.generate_random_obstacles(
    20, "polygons", (0, 0), (15, 15))

fig, ax = plt.subplots()

for obs in obstacles:
    patch = dc.PolygonPatch(obs)
    ax.add_patch(patch)


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
            l = l.difference(obs)
        lines.append(l)
    return lines


def plot_line(ax, ob):
    x, y = ob.xy
    ax.plot(x, y, 'g')


def producer():
    state = [0, 0, 0]
    omega1 = 5
    omega2 = 6
    # timestemp = 0.02
    while True:
        #  Wait for next request from client
        try:
            topic, message_str = sub_socket.recv_multipart(flags=zmq.NOBLOCK)
            message_json = json.loads(message_str)
            omega1 = message_json["omega1"]
            omega2 = message_json["omega2"]
        except zmq.ZMQError:
            print("nothing received")

        # if new controls info have arrived
        #   set the controls values
        # simulate one time step
        # yield the result

        res = solve_ivp(robot.deriv, [0, timestep],
                        state, args=[[omega1, omega2], 0])

        if not robot.collides(state, obstacles):
            state = res.y[:, -1]
        else:
            pub_socket.send_multipart([b"collision", b"{}"])
            pass

        lines = lidar(state, obstacles)
        dists = [line.length for line in lines]

        state_dict = {"x": state[0], "y": state[1], "theta": state[2]}
        state_str = json.dumps(state_dict).encode()
        pub_socket.send_multipart([b"state", state_str])

        lidar_dict = {"distances": dists}
        lidar_str = json.dumps(lidar_dict).encode()
        pub_socket.send_multipart([b"lidar", lidar_str])

        yield state, lines


patches = robot.draw(ax, [0, 0, 0])
for patch in patches:
    ax.add_patch(patch)

drawn_lines = []
for line in lidar([0, 0, 0], obstacles):
    x, y = line.xy
    l = ax.plot(x, y, 'g')[0]
    drawn_lines.append(l)


ax.axis("equal")
ax.grid("on")


def animate(data):
    state, lines = data
    patches = robot.draw(ax, state)
    for line, drawn_line in zip(lines, drawn_lines):
        try:
            x, y = line.xy
            drawn_line.set_data(x, y)
        except NotImplementedError:
            print(line)
    ax.set_xlim(state[0]-1, state[0]+1)
    ax.set_ylim(state[1]-1, state[1]+1)
    return *patches, *drawn_lines


animation = anim.FuncAnimation(
    fig, animate, producer, interval=timestep*1000)

plt.show(block=True)
