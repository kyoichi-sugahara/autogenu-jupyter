import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from sympy import sin, cos, tan, exp, log, sinh, cosh, tanh, atan, diff, sqrt
import os
import sys


class TrajectoryFollowing(object):
    """Generates the animation of the simulation results of a trajectory following.

    Attributes:
        set_skip_frames(skip_frames): Sets how many frames you want to
            skip in generating the animation. In the default settings,
            skip_frames = 1.
        generate_animation(): Draws an animation of the simulation reult
            and saves it as a .mp4 files.
    """

    def __init__(self, log_dir):
        """Inits CartPole with loading the simulation results."""
        # Loads the simulation data.
        self.__log_dir = log_dir
        self.__t_data = np.genfromtxt(os.path.join(log_dir, "t.log"))
        self.__x_data = np.genfromtxt(os.path.join(log_dir, "x.log"))
        self.__uopt_data = np.genfromtxt(os.path.join(log_dir, "uopt.log"))
        self.__sampling_time = (self.__t_data[1] - self.__t_data[0]) / 1000
        # Replaces NaN with 0.
        self.__x_data[np.isnan(self.__x_data)] = 0
        # Checks the dimension of the state.
        self.__dim_x = self.__x_data.shape[1]
        if self.__dim_x != 3:
            print(
                "Dimension of the state is not 4!\n"
                "This may not be data for simulation of a cartpole\n"
            )
            sys.exit()
        # Sets the drawing range.
        xabsmax = max(
            abs(np.amin(self.__x_data[:, 0])), abs(np.amax(self.__x_data[:, 0]))
        )
        self.__velocity = 1.0
        self.__N = self.__uopt_data.shape[1]
        self.__prediction_dt = 0.1
        self.__interval = self.__prediction_dt / self.__N
        self.__v_in_reference_trajectory = 1.0
        self.__wheel_base = 2.74
        self.__τ = 0.3
        # Sets frames for drawing the animation.
        self.__skip_frames = 1
        self.__total_frames = (int)(self.__x_data.shape[0] / self.__skip_frames)
        self.__x_min, self.__x_max, self.__y_min, self.__y_max = (
            self.__calculate_plot_range(margin=0.1)
        )

    def set_skip_frames(self, skip_frames):
        """Set how many frames you want to skip in generating the animation.

        Args:
            skip_frames: A number of frames to skip.
        """
        self.__skip_frames = skip_frames
        self.__total_frames = (int)(self.__x_data.shape[0] / skip_frames)

    def generate_animation(self):
        """Generates the animation and saves it as a .mp4 file."""
        # Create a new figure with specified size
        self.__fig = plt.figure(figsize=(8, 6))

        # Add axes to the figure with specified x and y limits
        # self.__ax = plt.axes(
        #     xlim=(self.__x_min, self.__x_max), ylim=(self.__y_min, self.__y_max)
        # )
        self.__ax = plt.axes(
            xlim=np.array([self.__x_min, self.__x_max], dtype=float),
            ylim=np.array([self.__y_min, self.__y_max], dtype=float),
        )

        # Create empty plots for ground and cart parts, which will be updated in each frame
        (self.__reference_trajectory,) = self.__ax.plot(
            [], [], color="#0063B1", linewidth=0.5
        )
        (self.__predicted_path,) = self.__ax.plot(
            [], [], color="#0063B1", linewidth=1.5
        )
        # (self.__base_link,) = self.__ac.plot([], [], color="#0063B1", linewidth=1.5)

        # Hide axis labels
        self.__ax.tick_params(
            labelbottom=True, labelleft=True, labelright=True, labeltop=True
        )

        # Set axis tick color to white
        self.__ax.tick_params(color="white")

        # Create a text object for displaying time, which will be updated in each frame
        self.__time_text = self.__ax.text(
            0.85, 0.05, "", transform=self.__ax.transAxes, fontsize=14
        )

        # Create the "plots" folder under self.__log_dir if it doesn't exist
        plots_dir = os.path.join(self.__log_dir, "plots")
        os.makedirs(plots_dir, exist_ok=True)

        # Generate an animation by repeatedly calling __update_animation method
        anime = FuncAnimation(
            self.__fig,
            self.__update_animation,
            interval=self.__sampling_time * 1000 * self.__skip_frames,
            frames=self.__total_frames,
            blit=True,
        )

        # Save each frame of the animation as a separate plot in the "plots" folder under self.__log_dir
        for i in range(self.__total_frames):
            self.__update_animation(i)
            self.__fig.savefig(os.path.join(plots_dir, f"frame_{i:04d}.png"))

        # Save the generated animation as a .mp4 file
        anime.save(
            os.path.join(self.__log_dir, "animation.mp4"),
            writer="ffmpeg",
            fps=int(1 / (self.__sampling_time * self.__skip_frames)),
        )

        # Print the location where the animation is saved
        print(
            "The animation of the simulation results is generated at " + self.__log_dir
        )

    def __calculate_plot_range(self, margin=0.1):
        """Calculate the range of the plot based on the simulation results."""

        for i in range(self.__total_frames):
            # Get the initial state at the current frame
            state = self.__x_data[i * self.__skip_frames, :]
            # insert x value of initial position as 0
            state = np.insert(state, 0, 0.0)
            input_array = self.__uopt_data[i * self.__skip_frames, :]
            x_series, y_series = self.__calculate_trajecotry(state, input_array)
            x_max = np.amax(x_series)
            x_min = np.amin(x_series)
            y_max = np.amax(y_series)
            y_min = np.amin(y_series)
        return x_min - margin, x_max + margin, y_min - margin, y_max + margin

    def __calculate_values(self, x, u):
        """Calculate three values based on the input vectors x and u."""

        f = [
            self.__v_in_reference_trajectory * cos(x[2]),
            self.__v_in_reference_trajectory * sin(x[2]),
            self.__v_in_reference_trajectory * tan(x[3]) / self.__wheel_base,
            -1 / self.__τ * (x[3] - u[0]),
        ]

        return f

    def __update_state(self, x, u):
        """Calculate three values based on the input vectors x and u."""

        f = self.__calculate_values(x, u)
        x = x + np.array(f) * self.__interval

        return f

    def __calculate_trajecotry(self, x, u):
        """Calculate three values based on the input vectors x and u."""
        x_series = []
        y_series = []
        for i in range(self.__N):
            x_series.append(x[0])
            y_series.append(x[1])
            x = x + np.array(self.__calculate_values(x, u)) * self.__prediction_dt
        return x_series, y_series

    def __update_animation(self, i):
        """Updates the animation for the i-th frame."""
        # Calculate the current frame number
        frame = self.__skip_frames * i

        # Get the initial state at the current frame
        state = self.__x_data[frame, :]
        # insert x value of initial position as 0
        state = np.insert(state, 0, 0.0)
        input_array = self.__uopt_data[frame, :]
        x_series, y_series = self.__calculate_trajecotry(state, input_array)
        # print(len(input_array))
        self.__reference_trajectory.set_data(x_series, y_series)

        # Calculate the x and y coordinates of reference trajectory
        # self.__xc = state[0]
        # self.__yc = state[1]

        # Calculate the x and y coordinates of the pole
        # self.__xp = self.__xc + self.__pole_length * np.sin(state[1])
        # self.__yp = 0.5 * self.__cart_height - self.__pole_length*np.cos(state[1])

        # Update the ground plot
        # self.__ground.set_data((self.__x_min, self.__x_max), (0, 0))

        # Update the top of the cart plot
        # self.__cartt.set_data(
        #     (self.__xc-0.5*self.__cart_width, self.__xc+0.5*self.__cart_width),
        #     (self.__cart_height, self.__cart_height)
        # )

        # Update the bottom of the cart plot
        # self.__cartb.set_data(
        #     (self.__xc-0.5*self.__cart_width, self.__xc+0.5*self.__cart_width),
        #     (0, 0)
        # )

        # Update the right side of the cart plot
        # self.__cartr.set_data(
        #     (self.__xc+0.5*self.__cart_width, self.__xc+0.5*self.__cart_width),
        #     (0, self.__cart_height)
        # )

        # Update the left side of the cart plot
        # self.__cartl.set_data(
        #     (self.__xc-0.5*self.__cart_width, self.__xc-0.5*self.__cart_width),
        #     (0, self.__cart_height)
        # )

        # Update the pole plot
        # self.__pole.set_data(
        #     (self.__xc, self.__xp),
        #     (0.5*self.__cart_height, self.__yp)
        # )

        # Update the time text
        self.__time_text.set_text("{0:.1f} [s]".format(self.__sampling_time * frame))

        # Return the updated plots and text
        return (
            self.__reference_trajectory,
            self.__predicted_path,
            self.__time_text,
        )
