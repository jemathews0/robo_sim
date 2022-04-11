#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 09:00:46 2022

@author: jonathan
"""

import matplotlib.patches as patches
import matplotlib as mpl
import numpy as np


class DifferentialDrive:
    def __init__(self, width, length, wheel_radius):
        self.w = width
        self.r = wheel_radius

        self.p1 = patches.Rectangle(
            (-length/2, -width/2), length, width)

    def deriv(self, t, state, u, params):
        x = state[0]
        y = state[1]
        theta = state[2]

        omega1 = u[0]
        omega2 = u[1]

        s = (omega1+omega2) * self.r
        k = (omega1-omega2) * self.r / self.w

        xdot = s*np.cos(theta)
        ydot = s*np.sin(theta)

        thetadot = k*s

        return [xdot, ydot, thetadot]

    def draw(self, ax, state):
        x = state[0]
        y = state[1]
        theta = state[2]

        t1 = mpl.transforms.Affine2D().rotate(
            theta) + mpl.transforms.Affine2D().translate(x, y) + ax.transData
        self.p1.set_transform(t1)
        return [self.p1]
