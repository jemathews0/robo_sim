#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 09:16:20 2022

@author: jonathan
"""

import zmq
import time

context = zmq.Context()

pub_socket = context.socket(zmq.PUB)
pub_socket.connect("ipc:///tmp/robotics/pub.ipc")

sub_socket = context.socket(zmq.SUB)
sub_socket.connect("ipc:///tmp/robotics/sub.ipc")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"")


while True:
    print("sending hello")
    pub_socket.send("hello".encode())
    #  Wait for next request from client
    try:
        message = sub_socket.recv(flags=zmq.NOBLOCK)
        print(message.decode())
    except zmq.ZMQError:
        print("nothing received")
        pass

    #  Do some 'work'
    time.sleep(1)
