#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 09:45:37 2022

@author: jonathan
"""
import zmq
import time

context = zmq.Context()

pub_socket = context.socket(zmq.PUB)
pub_socket.bind("ipc:///tmp/robotics/sub.ipc")

sub_socket = context.socket(zmq.SUB)
sub_socket.bind("ipc:///tmp/robotics/pub.ipc")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"")


while True:
    message = sub_socket.recv()
    pub_socket.send(message)
