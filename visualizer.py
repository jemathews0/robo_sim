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

np.random.seed(10000)

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

sub_socket.setsockopt(zmq.SUBSCRIBE, b"state")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"lidar")

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

def producer():
    while True:
        #  Get the newest message
        try:
            topic, message_str = sub_socket.recv_multipart()
            message_dict = json.loads(message_str.decode())
            yield topic, message_dict
        except zmq.ZMQError:
            print("nothing received")
            pass

def get_lines(state, dists):
    x = state[0]
    y = state[1]
    theta = state[2]

    num_rays = lidar_num_rays
    max_dist = lidar_max_dist
    min_angle = lidar_min_angle
    max_angle = lidar_max_angle
    angles = np.linspace(min_angle, max_angle, num_rays)

    if len(angles) != len(dists):
        dists = np.ones_like(angles)*max_dist

    lines = []
    for angle,dist in zip(angles,dists):
        end_x = dist*np.cos(theta+angle)+x
        end_y = dist*np.sin(theta+angle)+y
        l = geom.LineString([[x, y], [end_x, end_y]])
        lines.append(l)
    return lines

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

# Draw the initial state with a zoomed-in and zoomed-out view
stored_state = [start[0], start[1], 0]
stored_lines = get_lines(stored_state, [lidar_max_dist]*lidar_num_rays)
patches1 = robot1.draw(ax1, stored_state)
for patch in patches1:
    ax1.add_patch(patch)

patches2 = robot2.draw(ax2, stored_state)
for patch in patches2:
    ax2.add_patch(patch)

drawn_lines1 = []
for line in stored_lines:
    x, y = line.xy
    l = ax1.plot(x, y, 'g')[0]
    drawn_lines1.append(l)

drawn_lines2 = []
for line in stored_lines:
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
    global stored_state
    global stored_lines
    topic, message_dict = data
    if topic == b"state":
        stored_state = [message_dict["x"], message_dict["y"], message_dict["theta"]]
    elif topic == b"lidar":
        dists = message_dict["distances"]
        lines = get_lines(stored_state, dists)
        stored_lines = lines

    patches1 = robot1.draw(ax1, stored_state)
    patches2 = robot2.draw(ax2, stored_state)

    for line, drawn_line in zip(stored_lines, drawn_lines1):
        try:
            x, y = line.xy
            drawn_line.set_data(x, y)
        except NotImplementedError:
            print(line)
    for line, drawn_line in zip(stored_lines, drawn_lines2):
        try:
            x, y = line.xy
            drawn_line.set_data(x, y)
        except NotImplementedError:
            print(line)
    window_size = 1
    ax1.set_xlim(stored_state[0]-window_size, stored_state[0]+window_size)
    ax1.set_ylim(stored_state[1]-window_size, stored_state[1]+window_size)
    return *obs_patches1, *patches1, *patches2, *drawn_lines1, *drawn_lines2


time_scale = 0.1  # Make this number bigger to slow down time
animation = anim.FuncAnimation(
    fig, animate, producer, interval=timestep*1000*time_scale, blit=True)

plt.show(block=True)
