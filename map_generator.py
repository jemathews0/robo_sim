#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 15:08:29 2022

@author: jonathan
"""

import matplotlib.pyplot as plt
import json
import argparse

parser = argparse.ArgumentParser()
parser.add_argument(
    "output_file", help="The path to the map file to be created")
args = parser.parse_args()

output_file = args.output_file


curr_polygon = []
polygons = []
start_pt = (0, 0)
goal_pt = (0, 0)


def on_press(event):
    global curr_polygon
    global polypatch
    if event.button == plt.MouseButton.LEFT:
        curr_polygon.append([event.xdata, event.ydata])
        try:
            polypatch.set_xy(curr_polygon)
        except:
            polypatch = plt.Polygon(curr_polygon)
            ax.add_patch(polypatch)
        fig.canvas.draw()

    elif event.button == plt.MouseButton.RIGHT:
        curr_polygon.append([event.xdata, event.ydata])
        temp = plt.Polygon(curr_polygon)
        ax.add_patch(temp)
        fig.canvas.draw()
        polygons.append(curr_polygon)
        curr_polygon = []
        config = {"obstacles": polygons, "start": start_pt,
                  "goal": goal_pt, "bounds": [b_min, b_max]}

        with open(output_file, "w") as file:
            json.dump(config, file, indent=2)
    pass


def on_key(event):
    global start_pt
    global goal_pt
    global start_scatter
    global goal_scatter
    if event.key == "a":
        start_pt = (event.xdata, event.ydata)
        try:
            start_scatter.set_data(event.xdata, event.ydata)
        except:
            start_scatter, = ax.plot(event.xdata, event.ydata, "g.")
        fig.canvas.draw()
        config = {"obstacles": polygons, "start": start_pt,
                  "goal": goal_pt, "bounds": [b_min, b_max]}

        with open(output_file, "w") as file:
            json.dump(config, file, indent=2)
    elif event.key == "b":
        goal_pt = (event.xdata, event.ydata)
        try:
            goal_scatter.set_data(event.xdata, event.ydata)
        except:
            goal_scatter, = ax.plot(event.xdata, event.ydata, "r.")
        fig.canvas.draw()
        config = {"obstacles": polygons, "start": start_pt,
                  "goal": goal_pt, "bounds": [b_min, b_max]}

        with open(output_file, "w") as file:
            json.dump(config, file, indent=2)

    pass


fig, ax = plt.subplots()
fig.suptitle("Left click to drop a vertex for a polygon.\n Right click to drop the last vertex for a polygon.\nPress 'a' to place the start point.\nPress 'b' to place the end point")

boundary_pts = [[-1, -1], [-1, 21], [21, 21], [21, -1]]
interior_pts = [[0, 0], [0, 20], [20, 20], [20, 0]][::-1]

b_min = 0
b_max = 5

boundary_polys = []
boundary_polys.append([[b_min, b_min], [b_min, b_max], [
                      b_min-1, b_max], [b_min-1, b_min]])
boundary_polys.append([[b_min-1, b_max], [b_min-1, b_max+1],
                      [b_max+1, b_max+1], [b_max+1, b_max]])
boundary_polys.append([[b_max, b_max], [b_max+1, b_max],
                      [b_max+1, b_min], [b_max, b_min]])
boundary_polys.append([[b_max+1, b_min], [b_max+1, b_min-1],
                      [b_min-1, b_min-1], [b_min-1, b_min]])

for poly in boundary_polys:
    polygons.append(poly)
    temp = plt.Polygon(poly)
    ax.add_patch(temp)

fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("key_press_event", on_key)

ax.set_xlim(b_min-1, b_max+1)
ax.set_ylim(b_min-1, b_max+1)

plt.show(block=True)
