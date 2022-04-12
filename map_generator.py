#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 11 15:08:29 2022

@author: jonathan
"""

import matplotlib.pyplot as plt
import json


curr_polygon = []
polygons = []
start_pt = (0, 0)
goal_pt = (0, 0)


def on_press(event):
    global curr_polygon
    global polypatch
    if event.button == plt.MouseButton.LEFT:
        print('you pressed', event.button, event.xdata, event.ydata)

        curr_polygon.append([event.xdata, event.ydata])
        try:
            polypatch.set_xy(curr_polygon)
        except:
            polypatch = plt.Polygon(curr_polygon)
            ax.add_patch(polypatch)
        fig.canvas.draw()

    elif event.button == plt.MouseButton.RIGHT:
        print('you pressed', event.button, event.xdata, event.ydata)
        curr_polygon.append([event.xdata, event.ydata])
        temp = plt.Polygon(curr_polygon)
        ax.add_patch(temp)
        fig.canvas.draw()
        polygons.append(curr_polygon)
        curr_polygon = []
        config = {"obstacles": polygons, "start": start_pt, "goal": goal_pt}

        with open("output.json", "w") as file:
            json.dump(config, file)
    pass


def on_key(event):
    global start_pt
    global goal_pt
    if event.key == "s":
        start_pt = (event.xdata, event.ydata)
        ax.plot(event.xdata, event.ydata, "g.")
        config = {"obstacles": polygons, "start": start_pt, "goal": goal_pt}

        with open("output.json", "w") as file:
            json.dump(config, file)
    elif event.key == "g":
        goal_pt = (event.xdata, event.ydata)
        ax.plot(event.xdata, event.ydata, "r.")
        config = {"obstacles": polygons, "start": start_pt, "goal": goal_pt}

        with open("output.json", "w") as file:
            json.dump(config, file)

    pass


fig, ax = plt.subplots()

fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("key_press_event", on_key)

ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)

# plt.show(block=True)
