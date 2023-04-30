#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 09:16:20 2022

@author: jonathan
"""

import zmq
import time
import json
import numpy as np

context = zmq.Context()

pub_socket = context.socket(zmq.PUB)
# pub_socket.connect("ipc:///tmp/robo_sim/pub.ipc")
pub_socket.connect("tcp://localhost:5557")

sub_socket = context.socket(zmq.SUB)
# sub_socket.connect("ipc:///tmp/robo_sim/sub.ipc")
sub_socket.connect("tcp://localhost:5555")

sub_socket.setsockopt(zmq.SUBSCRIBE, b"state")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"collision")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"lidar")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"landmarks")


s = 10
k = 0.0
r = 0.5
w = 1
k_dir = 1

count = 0
while True:
    topic, message = sub_socket.recv_multipart()

    if topic == b"lidar":
        max_dist = 0.5
        angle_interval = np.pi/19
        message_dict = json.loads(message.decode())
        lidar_dists = message_dict["distances"]

        min_index = np.argmin(lidar_dists)
        min_angle = -np.pi/2 + min_index*angle_interval
        min_dist = lidar_dists[min_index]

        if min_dist < 0.8*max_dist:
            k = -10*np.copysign(np.exp(-np.abs(min_angle)), min_angle)
            print(k)
            if not np.isfinite(k):
                k = 0
        else:
            k = 0

        vel_r = (s+w*k)/2
        vel_l = (s-w*k)/2

        twist = {"linear": s, "angular": w*k}
        pub_socket.send_multipart(
            [b"twist", json.dumps(twist).encode()])

        wheel_speeds = {
                "vel_r": vel_r,
                "vel_l": vel_l,
                }
        pub_socket.send_multipart(
            [b"wheel_speeds", json.dumps(wheel_speeds).encode()])


        count += 1
