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

context = zmq.Context()

pub_socket = context.socket(zmq.PUB)
pub_socket.connect("ipc:///tmp/robotics/pub.ipc")

sub_socket = context.socket(zmq.SUB)
sub_socket.connect("ipc:///tmp/robotics/sub.ipc")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"wheel_speeds")

robot_width = 1
robot_length = 2
robot_radius = 0.5
robot = dd.DifferentialDrive(robot_width, robot_length, robot_radius)
timestep = 0.02


fig, ax = plt.subplots()


def producer():
    state = [0, 0, 0]
    omega1 = 5
    omega2 = 6
    timestemp = 0.02
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
        state = res.y[:, -1]
        state_dict = {"x": state[0], "y": state[1], "theta": state[2]}
        state_str = json.dumps(state_dict).encode()
        pub_socket.send_multipart([b"state", state_str])
        yield state


patches = robot.draw(ax, [0, 0, 0])
for patch in patches:
    ax.add_patch(patch)


ax.axis("equal")
ax.set_ylim(-20, 20)
ax.set_xlim(-20, 20)
ax.grid("on")


def animate(state):
    patches = robot.draw(ax, state)
    ax.set_xlim(state[0]-10, state[0]+10)
    ax.set_ylim(state[1]-10, state[1]+10)
    return patches


animation = anim.FuncAnimation(
    fig, animate, producer, interval=timestep*1000)

plt.show(block=True)
