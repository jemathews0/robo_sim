#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 09:00:06 2022

@author: jonathan
"""

import rospy
from geometry_msgs.msg import Pose, Twist, Quaternion, Point
import tf
import time
import differential_drive as dd
import json
from scipy.integrate import solve_ivp
import shapely.geometry as geom
import numpy as np
import argparse

np.random.seed(10000)

parser = argparse.ArgumentParser()
parser.add_argument("map", help="The path to the map file to be loaded")
args, unknown = parser.parse_known_args()

map_name = args.map

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
landmark_bearing_sigma = 0.1

currentTwist = Twist()

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
                l = l_temp.geoms[0]
            else:
                l = l_temp
        lines.append(l)
    return lines


def add_wheel_noise(omega_r, omega_l):
    sigma1 = np.sqrt(wheel_noise_a1*omega_r**2+wheel_noise_a2*omega_l**2)
    sigma2 = np.sqrt(wheel_noise_a3*omega_r**2+wheel_noise_a4*omega_l**2)
    omega_r += np.random.normal(0, sigma1)
    omega_l += np.random.normal(0, sigma2)
    return omega_r, omega_l


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
    global currentTwist
    state = [start[0], start[1], 0]
    omega_r = 0
    omega_l = 0
    count = 0
    t = 0
    while True:
        # Convert desired linear and angular velocity into left and right wheel speeds
        s = currentTwist.linear.x
        omega = currentTwist.angular.z
        vel_r = s + omega*robot_width/2
        vel_l = s - omega*robot_width/2
        omega_r = vel_r/robot_radius
        omega_l = vel_l/robot_radius

        noisy_omega_r, noisy_omega_l = add_wheel_noise(omega_r, omega_l)

        res = solve_ivp(robot1.deriv, [0, timestep],
                        state, args=[[noisy_omega_r, noisy_omega_l], 0])
        t += timestep

        if not robot1.collides(state, obstacles):
            state = res.y[:, -1]
            collision_dict = {"timestamp": t, "collision": False}
        else:
            collision_dict = {"timestamp": t, "collision": True}
            pass
        #collision_str = json.dumps(collision_dict).encode()
        #pub_socket.send_multipart([b"collision", collision_str])

        lines = lidar(state, obstacles)
        dists = [line.length for line in lines]

        marks, mark_dists, mark_thetas = visible_landmarks(state, landmarks)
        #message_dict = {"timestamp": t}
        ## print([index[0] for index in marks])
        #marks_dict = {}
        #for index, dist, theta in zip(marks, mark_dists, mark_thetas):
        #    mark = landmarks[index]
        #    marks_dict[int(index)] = {}
        #    marks_dict[int(index)]["dist"] = dist + \
        #        np.random.normal(0, landmark_range_sigma)
        #    marks_dict[int(index)]["theta"] = theta + \
        #        np.random.normal(0, landmark_bearing_sigma)

        #    # print(marks_dict)
        #message_dict["landmarks"] = marks_dict
        #marks_str = json.dumps(message_dict).encode()
        #pub_socket.send_multipart([b"landmarks", marks_str])

        #lidar_dict = {"timestamp": t, "distances": dists}
        #lidar_str = json.dumps(lidar_dict).encode()
        #pub_socket.send_multipart([b"lidar", lidar_str])

        pose = Pose()
        pose.position = Point(state[0], state[1], 0)
        pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,state[2]))
        pose_pub.publish(pose)

        # print(count)
        count += 1
        yield state, lines


state = [start[0], start[1], 0]

def twistCallback(twist):
    global currentTwist
    currentTwist = twist


rospy.init_node("simulator", anonymous=True)
rospy.Subscriber("twist", Twist, twistCallback)
pose_pub = rospy.Publisher("pose", Pose, queue_size=100)
ros_rate = rospy.Rate(100)

for data in producer():
    if rospy.is_shutdown():
        break
    else:
        ros_rate.sleep()

