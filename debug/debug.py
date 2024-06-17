import os
import sys
import argparse
import matplotlib.pyplot as plt
import numpy as np
from plotter import Plotter
import importlib
import animator
from animator import TrajectoryFollowing
from directory_finder import DirectoryFinder

script_dir = os.path.dirname(os.path.abspath(__file__))

import visualize_path
from visualize_path import plot_trajectory


def main(
    show_plotter=True, show_animation=True, show_images=True, show_trajectory=True
):
    home_directory = os.path.expanduser("~")
    cgmres_directory_path = os.path.join(home_directory, ".ros/log")

    directory_finder = DirectoryFinder(cgmres_directory_path)

    latest_cgmres_directory = directory_finder.find_latest_directory(
        prefix="cgmres_debug_"
    )
    latest_trajectory_directory = directory_finder.find_latest_directory(
        prefix="trajectory_"
    )

    cgmres_creation_time = directory_finder.get_directory_creation_time(
        latest_cgmres_directory
    )
    trajectory_creation_time = directory_finder.get_directory_creation_time(
        latest_trajectory_directory
    )

    print(f"The latest cgmres debug directory is: {latest_cgmres_directory}")
    print(f"The latest cgmres debug directory was created at: {cgmres_creation_time}")
    print(f"The latest trajectory directory is: {latest_trajectory_directory}")
    print(f"The latest trajectory directory was created at: {trajectory_creation_time}")

    plotter = Plotter(log_dir=latest_cgmres_directory)
    plotter.set_scales(2, 5, 2)
    if show_plotter:
        plotter.show()
        # plotter.save()
        plt.draw()
        plt.pause(1)
    else:
        plotter.save()

    anim = TrajectoryFollowing(
        cgmres_log_dir=latest_cgmres_directory,
        trajectory_log_dir=latest_trajectory_directory,
    )
    # anim.set_skip_frames(10)
    if show_animation:
        anim.generate_animation()

        plots_dir = os.path.join(latest_cgmres_directory, "plots")
        image_files = sorted([f for f in os.listdir(plots_dir) if f.endswith(".png")])
        first_index = 0
        last_index = len(image_files) - 1
        num_images = 10
        indices = np.linspace(first_index, last_index, num_images, dtype=int)
        selected_images = [image_files[i] for i in indices]

        fig, axes = plt.subplots(5, 2, figsize=(10, 20))
        axes = axes.ravel()
        for i, image_file in enumerate(selected_images):
            img = plt.imread(os.path.join(plots_dir, image_file))
            axes[i].imshow(img)
            axes[i].axis("off")
            axes[i].set_title(f"Frame {os.path.splitext(image_file)[0][6:]}")

        plt.tight_layout()
        plt.draw()
        plt.pause(1)

    if show_trajectory:
        plot_trajectory(latest_trajectory_directory, cgmres_creation_time)
        plt.draw()
        plt.pause(1)

    if show_plotter or show_animation or show_images or show_trajectory:
        plt.show(block=False)

        input("Press Enter to continue and close plots...")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Process and plot CGMRES and trajectory data."
    )
    parser.add_argument(
        "--plotter",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show plotter plots.",
    )
    parser.add_argument(
        "--animation",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Generate animation and images.",
    )
    parser.add_argument(
        "--trajectory",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show trajectory plots.",
    )
    args = parser.parse_args()

    main(
        show_plotter=args.plotter,
        show_animation=args.animation,
        show_trajectory=args.trajectory,
    )
