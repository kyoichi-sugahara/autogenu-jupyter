#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import csv
import os

import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


def read_csv(directory, filename):
    filepath = os.path.join(directory, filename)
    with open(filepath, "r") as file:
        reader = csv.reader(file)
        data = list(reader)
    return [[float(x) for x in row if x] for row in data]


def find_latest_directory(log_directory):
    trajectory_directories = [
        d
        for d in os.listdir(log_directory)
        if os.path.isdir(os.path.join(log_directory, d)) and d.startswith("trajectory_")
    ]
    if trajectory_directories:
        return max(trajectory_directories)
    return None


def plot_trajectory(data_directory=None, creation_time=None, trajectories=None):
    if data_directory is None:
        home_directory = os.path.expanduser("~")
        log_directory = os.path.join(home_directory, ".ros", "log")
        latest_directory = find_latest_directory(log_directory)
        if latest_directory:
            data_directory = os.path.join(log_directory, latest_directory)
        else:
            print("No trajectory directory found.")
            return

    available_trajectories = {
        "original_ref": {"x": "original_ref_x.log", "y": "original_ref_y.log", "label": "Original Reference Trajectory", "marker": "o"},
        "resampled_ref": {"x": "resampled_ref_x.log", "y": "resampled_ref_y.log", "label": "Resampled Reference Trajectory", "marker": "s"},
        "predicted": {"x": "predicted_x.log", "y": "predicted_y.log", "label": "Predicted Trajectory", "marker": "d"},
        "predicted_frenet": {"x": "predicted_frenet_x.log", "y": "predicted_frenet_y.log", "label": "Predicted Frenet Trajectory", "marker": "^"},
        "cgmres_predicted_frenet": {"x": "cgmres_predicted_frenet_x.log", "y": "cgmres_predicted_frenet_y.log", "label": "CGMRES Predicted Frenet Trajectory", "marker": "v"},
        "cgmres_predicted": {"x": "cgmres_predicted_x.log", "y": "cgmres_predicted_y.log", "label": "CGMRES Predicted Trajectory", "marker": "<"},
    }

    if trajectories is None:
        trajectories = available_trajectories.keys()

    fig, ax = plt.subplots(figsize=(8, 6))
    plt.subplots_adjust(bottom=0.2)

    plots = {}
    for traj in trajectories:
        if traj in available_trajectories:
            data = available_trajectories[traj]
            plots[traj] = ax.plot([], [], marker=data["marker"], label=data["label"])[0]

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    if creation_time is None:
        directory_name = os.path.basename(data_directory)
        ax.set_title(f"Trajectory Comparison\n{directory_name}")
    else:
        ax.set_title(f"Trajectory Comparison\n{creation_time}")
    ax.legend()
    ax.grid(True)
    plt.gca().set_aspect("equal", adjustable="box")

    time_data = read_csv(data_directory, "time.log")
    slider_ax = plt.axes([0.2, 0.05, 0.6, 0.03])
    time_slider = Slider(slider_ax, "Time", 0, max(1, len(time_data) - 1), valinit=0, valstep=1)

    def update(time_index):
        for traj in trajectories:
            if traj in available_trajectories:
                data = available_trajectories[traj]
                x_data = read_csv(data_directory, data["x"])[time_index]
                y_data = read_csv(data_directory, data["y"])[time_index]
                plots[traj].set_data(x_data, y_data)

        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw_idle()

    time_slider.on_changed(update)
    update(0)
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot trajectory data from CSV files.")
    parser.add_argument("--directory", type=str, help="Directory containing the CSV files")
    parser.add_argument("--directory-creation-time", type=str, help="Creation time of the directory")
    parser.add_argument("--trajectories", nargs='+', help="List of trajectories to plot (options: original_ref, resampled_ref, predicted, predicted_frenet, cgmres_predicted_frenet, cgmres_predicted)")
    args = parser.parse_args()

    plot_trajectory(args.directory, args.directory_creation_time, args.trajectories)
