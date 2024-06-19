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
        data = file.readlines()
    return [[float(x) for x in line.split()] for line in data if line.strip()]


def plot_solution(data_directory, creation_time, fixed_scale=True):

    optimized_u_array = read_csv(data_directory, "uopt.log")
    initial_solution_array = read_csv(data_directory, "initial_solution.log")
    updated_solution_array = read_csv(data_directory, "updated_solution.log")
    time_data = read_csv(data_directory, "t.log")

    all_optimized_y_data = [val for sublist in optimized_u_array for val in sublist]
    min_optimized_y, max_optimized_y = min(all_optimized_y_data), max(
        all_optimized_y_data
    )
    optimized_y_margin = (max_optimized_y - min_optimized_y) * 0.1
    min_optimized_y_with_margin = min_optimized_y - optimized_y_margin
    max_optimized_y_with_margin = max_optimized_y + optimized_y_margin

    all_initial_y_data = [val for sublist in initial_solution_array for val in sublist]
    all_updated_y_data = [val for sublist in updated_solution_array for val in sublist]
    min_right_y = min(min(all_initial_y_data), min(all_updated_y_data))
    max_right_y = max(max(all_initial_y_data), max(all_updated_y_data))
    right_y_margin = (max_right_y - min_right_y) * 0.1
    if fixed_scale:
        min_right_y_with_margin = min_right_y - right_y_margin
        max_right_y_with_margin = max_right_y + right_y_margin
    else:
        min_right_y_with_margin = None
        max_right_y_with_margin = None

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    plt.subplots_adjust(bottom=0.2)

    (optimized_u_plot,) = ax1.plot(
        [], [], marker="o", label="optimized u", color="blue"
    )
    (initial_solution_plot,) = ax2.plot(
        [], [], linestyle="None", marker="o", label="initial solution", color="red"
    )
    (updated_solution_plot,) = ax2.plot(
        [], [], linestyle="None", marker="x", label="updated solution", color="green"
    )

    for ax in (ax1, ax2):
        ax.set_xlabel("step")
        ax.set_ylabel("value")
        ax.set_xlim(0, len(optimized_u_array[0]) - 1)
        ax.legend()
        ax.grid(True)

    ax1.set_ylim(min_optimized_y_with_margin, max_optimized_y_with_margin)
    ax1.set_title(f"Optimized Solution\n{creation_time}")

    ax2.set_ylim(min_right_y_with_margin, max_right_y_with_margin)
    ax2.set_title(f"Initial and Updated Solutions\n{creation_time}")

    slider_ax = plt.axes([0.2, 0.05, 0.6, 0.03])
    time_slider = Slider(slider_ax, "Time", 0, len(time_data) - 1, valinit=0, valstep=1)

    def update(time_index):
        row_opt = optimized_u_array[time_index]
        row_init = initial_solution_array[time_index]
        row_updated = updated_solution_array[time_index]
        x_data = list(range(len(row_opt)))
        y_data_opt = row_opt
        y_data_init = row_init
        y_data_updated = row_updated
        optimized_u_plot.set_data(x_data, y_data_opt)
        initial_solution_plot.set_data(x_data, y_data_init)
        updated_solution_plot.set_data(x_data, y_data_updated)

        if not fixed_scale:
            min_right_y = min(min(row_init), min(row_updated))
            max_right_y = max(max(row_init), max(row_updated))
            right_y_margin = (max_right_y - min_right_y) * 0.1
            ax2.set_ylim(min_right_y - right_y_margin, max_right_y + right_y_margin)

        fig.canvas.draw_idle()

    time_slider.on_changed(update)
    update(0)
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot trajectory data from CSV files.")
    parser.add_argument(
        "--directory", type=str, help="Directory containing the CSV files"
    )
    parser.add_argument(
        "--directory-creation-time", type=str, help="Creation time of the directory"
    )
    parser.add_argument(
        "--fixed-scale",
        action="store_true",
        help="Fix the scale of the right plot to the first time step",
    )
    args = parser.parse_args()

    plot_solution(args.directory, args.directory_creation_time)
