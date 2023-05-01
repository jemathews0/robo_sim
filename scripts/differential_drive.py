#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 09:00:46 2022

@author: jonathan
"""

import matplotlib.patches as patches
import matplotlib as mpl
import numpy as np
import shapely.geometry as geom
import shapely.affinity as aff


class DifferentialDrive:
    def __init__(self, width, length, wheel_radius):
        self.w = width
        self.r = wheel_radius

        points1 = [[-length/2, -width/2], [length/2, -width/2],
                   [length/2, width/2], [-length/2, width/2]]
        self.s1 = geom.Polygon(points1)
        #self.p1 = dc.PolygonPatch(self.s1)

        self.p1 = patches.Polygon(points1)


    def deriv(self, t, state, u, params):
        x = state[0]
        y = state[1]
        theta = state[2]

        omega_r = u[0]
        omega_l = u[1]

        s = (omega_r+omega_l) * self.r / 2

        xdot = s*np.cos(theta)
        ydot = s*np.sin(theta)

        thetadot = self.r/self.w * (omega_r-omega_l)

        return [xdot, ydot, thetadot]

    def draw(self, ax, state):
        x = state[0]
        y = state[1]
        theta = state[2]

        t1 = mpl.transforms.Affine2D().rotate(
            theta) + mpl.transforms.Affine2D().translate(x, y) + ax.transData
        self.p1.set_transform(t1)
        return [self.p1]

    def collides(self, state, obstacles):
        x = state[0]
        y = state[1]
        theta = state[2]

        rotated_s1 = aff.rotate(
            self.s1, theta, origin=(0, 0), use_radians=True)
        translated_s1 = aff.translate(rotated_s1, x, y)

        for obs in obstacles:
            if translated_s1.intersects(obs):
                return True
        return False
