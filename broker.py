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
# pub_socket.bind("ipc:///tmp/robo_sim/sub.ipc")
pub_socket.bind("tcp://*:5555")

sub_socket = context.socket(zmq.SUB)
# sub_socket.bind("ipc:///tmp/robo_sim/pub.ipc")
sub_socket.bind("tcp://*:5557")
sub_socket.setsockopt(zmq.SUBSCRIBE, b"")


while True:
    msg_parts = sub_socket.recv_multipart()
    pub_socket.send_multipart(msg_parts)
