#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 09:16:20 2022

@author: jonathan
"""

import zmq
import time
import json

context = zmq.Context()

pub_socket = context.socket(zmq.PUB)
pub_socket.connect("ipc:///tmp/robotics/pub.ipc")

sub_socket = context.socket(zmq.SUB)
sub_socket.connect("ipc:///tmp/robotics/sub.ipc")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"state")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"collision")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"lidar")


s = 1
k = 0.5
r = 0.5
w = 1
k_dir = 1

while True:
    if k > 1:
        k_dir = -1
    elif k < -1:
        k_dir = 1
    k += k_dir*0.01

    omega1 = (s+w*k)/(2*r)
    omega2 = (s-w*k)/(2*r)

    wheel_speeds = {"omega1": omega1, "omega2": omega2}
    pub_socket.send_multipart(
        [b"wheel_speeds", json.dumps(wheel_speeds).encode()])

    topic, message = sub_socket.recv_multipart()
    print(topic, ":", json.loads(message.decode()))
