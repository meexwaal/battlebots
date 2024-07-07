import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import itertools
import types

from common import *
from lidar import Lidar
from sensor import Sensor

# TODO
# Model additional factors
# - Motor response--max force isn't constant, and current value is made up anyways
# - Friction, wheel slip
# - Noise/error in motor speeds?

# Simulation time step
DT = 1/1800

class Melty:
    def __init__(self):

        # Constants

        # Total mass (kg)
        self.mass = 1.27 # 2.8 lbs

        # Moment of inertia (m^2 kg)
        self.I = 0.0158

        # Outer radius (m)
        self.radius = 0.127 # 5"

        # Distance from center to wheels (m)
        self.wheel_pos = 0.05715 # 2.25"
        # self.wheel_pos = 0.0762 # 3"

        # Size of wheels (m)
        self.wheel_radius = 0.0381 # 3"

        # Maximum rotation rate (rad/s)
        self.max_w = 366.519143 # 3500 rpm

        ##################
        # Commanded values

        # Motor commands, 0-1.
        # Motors are positioned at +x and -x, to produce force in +y and -y.
        self.motor_px_level = 0.0
        self.motor_nx_level = 0.0


        # True state. Algs running on the bot don't know these!
        self.truth = types.SimpleNamespace()

        # Position (m)
        self.truth.x = 0.0
        self.truth.y = 0.0

        # Orientation (radians)
        self.truth.theta = 0.0

        # Velocity (m/s)
        self.truth.vx = 0.0
        self.truth.vy = 0.0

        # Rotation rate (rad/s)
        self.truth.w = 0.0

        # Time (s)
        self.truth.time = 0.0


        ##############
        # Sensed state. Algs running on the bot use these.
        self.sense = types.SimpleNamespace()

        # Position (m)
        self.sense.x = 0.0
        self.sense.y = 0.0

        # Orientation (radians)
        self.sense.theta = 0.0

        # Velocity (m/s)
        self.sense.vx = 0.0
        self.sense.vy = 0.0

        # Rotation rate (rad/s)
        self.sense.w = 0.0


        #########
        # Sensors
        self.accelerometer = Sensor(
            sample_rate=100,
            min_val=-400 * 9.8,
            max_val=400 * 9.8,
            # bias=1 * 9.8,
            noise_std=3 * 9.8,
            nonlinearity=-0.01)

        self.lidar = Lidar()

    def __repr__(self):
        truth = self.truth
        return (
            f"Bot state:\n"
            f"  Position: {truth.x:0.3f}, {truth.y:0.3f}\n"
            f"  Velocity: {truth.vx:0.3f}, {truth.vy:0.3f}\n"
            f"  Angle:    {np.rad2deg(truth.theta):.0f} deg, {truth.w/(2 * np.pi):.3f} rps")


    def get_motor_torque(self, level):
        "Get the torque (N m) produced by a motor with the given params"
        # todo think it would be something like motor power / motor rpm
        return 0.1 * level


    def set_motor_levels(self, px_level, nx_level):
        self.motor_px_level = px_level
        self.motor_nx_level = nx_level


    def set_max_rotation_rate(self):
        "Force rotation rate to maximum"
        self.truth.w = self.max_w


    def sense_accel(self):
        "Use an accelerometer to sense the rotation rate"
        truth = self.truth

        # Mount the accelerometer this far from the center (m)
        accel_pos = 0.025

        true_accel = truth.w**2 * accel_pos
        self.accelerometer.step(true_accel, DT)
        self.sense.w = np.sqrt(max(0, self.accelerometer.sense) / accel_pos)


    def sense_theta_prop(self):
        "Propagate angular rate to get angular position."
        self.sense.theta += self.sense.w * DT


    def step(self):
        "Step the simulation by time DT"

        truth = self.truth
        truth.time += DT

        self.sense_accel()
        self.sense_theta_prop()
        self.lidar.align_alg(truth.x, truth.y, truth.theta, DT, self.sense.w)

        f_px = self.get_motor_torque(self.motor_px_level) / self.wheel_radius
        f_nx = self.get_motor_torque(self.motor_nx_level) / self.wheel_radius

        bot_torque = (f_px + f_nx) * self.wheel_pos

        #
        #       ^
        #       '
        # O--+--O  theta = 0 -> +y accel
        # nx   px
        #
        bot_accel = (f_px - f_nx) / self.mass
        bot_accel_x = bot_accel * -np.sin(truth.theta)
        bot_accel_y = bot_accel * np.cos(truth.theta)

        truth.vx += bot_accel_x * DT
        truth.vy += bot_accel_y * DT

        truth.x += truth.vx * DT
        truth.y += truth.vy * DT

        truth.w += bot_torque / self.I * DT
        truth.w = np.clip(truth.w, -self.max_w, self.max_w)

        truth.theta += truth.w * DT
        truth.theta = np.fmod(truth.theta, 2 * np.pi)


def gen_spin_up(bot, threshold=0.9):
    "Generator function. Spin bot up to threshold * max rotation speed."

    bot.set_motor_levels(1, 1)
    t = 0
    while bot.truth.w < threshold * bot.max_w:
        bot.step()
        t += DT

        yield

    print(f"{t:.3f} s to spin up to speed")

def skip_spin_up(bot):
    "Generator function. Skip to the end of spinning the bot up to speed."

    bot.set_max_rotation_rate()
    bot.lidar.est_w = bot.truth.w * 0.98
    yield

def gen_translate(bot, arena_theta, distance, tolerance=0.01):
    "Generator function. Control the bot to translate a given distance in a given direction."
    xf = bot.truth.x + distance * np.cos(arena_theta)
    yf = bot.truth.y + distance * np.sin(arena_theta)

    print(f"Translating to {xf:.3f}, {yf:.3f}")

    t = 0
    while (np.abs(bot.truth.x - xf) > tolerance) or (np.abs(bot.truth.y - yf) > tolerance):
        # Use accel sense value
        # if angle_diff(bot.sense.theta, arena_theta) < 0:
        # Use lidar estimated theta
        if angle_diff(bot.lidar.est_theta, arena_theta) < 0:
            bot.set_motor_levels(1, 0)
        else:
            bot.set_motor_levels(0, 1)

        bot.step()
        t += DT
        # print(bot)
        yield

    print(f"{t:.3f} s to translate")

def gen_loop(bot):
    "Generator function. Loop the bot forever."
    print(f"Entering infinite loop")
    while True:
        bot.step()

        yield

def gen_main(bot):
    "Generator for main function"

    return itertools.chain(
        gen_spin_up(bot),
        # skip_spin_up(bot),
        # gen_loop(bot),
        gen_translate(bot, 0, 1),
        # gen_translate(bot, 0.2, 1),
    )

def animate(save=False):
    bot = Melty()
    # bot.truth.x = 1.0
    # bot.truth.y = 0.8
    prog = gen_main(bot)

    fig, ax = plt.subplots()

    # Draw circle for bot
    bot_circle = plt.Circle((0, 0), bot.radius, fill=False, color='red')

    # Draw points for sensed orientation
    orientation, = ax.plot([], [], 'go', markersize=2)

    # Draw wheels
    px_wheel, = ax.plot([], [], color='black', linewidth=1)
    nx_wheel, = ax.plot([], [], color='black', linewidth=1)

    # Track position
    xlog, ylog = [], []
    plot_pos, = ax.plot([], [], color='blue', linewidth=0.5)

    # Draw lidar history
    plot_lidar, = ax.plot([], [], 'ro', markersize=2)

    def init():
        ax.set_xlim(-2.5, 2.5)
        ax.set_ylim(-2.5, 2.5)

        # ax.set_xlim(-1, 1)
        # ax.set_ylim(-1, 1)

        # ax.set_xlim(0, 1)
        # ax.set_ylim(-0.005, 0.002)

        ax.add_patch(bot_circle)
        return plot_pos, plot_lidar, bot_circle, orientation, px_wheel, nx_wheel

    def update(_):
        next(prog)

        bx, by, btheta = bot.truth.x, bot.truth.y, bot.truth.theta
        bot_circle.center = (bx, by)

        err_theta = btheta - bot.sense.theta
        # Plot orientation error
        # orientation.set_data(
        #     [bx + 0.9 * bot.radius * np.cos(err_theta)],
        #     [by + 0.9 * bot.radius * np.sin(err_theta)])
        # Plot true and estimated theta, and lidar estimated position
        orientation.set_data(
            [bx + 0.9 * bot.radius * np.cos(btheta),
             bx + 1.1 * bot.radius * np.cos(bot.lidar.est_theta),
             bot.lidar.est_x,
            ],
            [by + 0.9 * bot.radius * np.sin(btheta),
             by + 1.1 * bot.radius * np.sin(bot.lidar.est_theta),
             bot.lidar.est_y,
            ])

        xlog.append(bx)
        ylog.append(by)
        plot_pos.set_data(xlog, ylog)

        lidar_points = np.array(bot.lidar.hist_points)
        if len(lidar_points) > 0:
            plot_lidar.set_data(lidar_points[:,0], lidar_points[:,1])

        px_base_x = bx + bot.wheel_pos * np.cos(btheta)
        px_base_y = by + bot.wheel_pos * np.sin(btheta)
        px_ends_x = np.array((
            px_base_x - bot.wheel_radius * np.sin(btheta),
            px_base_x + bot.wheel_radius * np.sin(btheta),
        ))
        px_ends_y = np.array((
            px_base_y + bot.wheel_radius * np.cos(btheta),
            px_base_y - bot.wheel_radius * np.cos(btheta),
        ))
        px_wheel.set_data(px_ends_x, px_ends_y)
        nx_wheel.set_data(2 * bx - px_ends_x, 2 * by - px_ends_y)

        px_wheel.set_linewidth(max(0.2, bot.motor_px_level * 2))
        nx_wheel.set_linewidth(max(0.2, bot.motor_nx_level * 2))

        # ax.set_xlim(bx - 0.15, bx + 0.15)
        # ax.set_ylim(by - 0.15, by + 0.15)

        return plot_pos, plot_lidar, bot_circle, orientation, px_wheel, nx_wheel


    ani = FuncAnimation(
        fig,
        update, init_func=init,
        # frames=5000,
        blit=True, repeat=False,

        # Set how fast the animation plays
        interval=1
        # interval=40
        # interval=300
    )

    plt.gca().set_aspect('equal')

    if save:
        writer = PillowWriter(fps=20)
        ani.save('melty.gif', writer=writer)
    else:
        plt.show()


if __name__ == '__main__':
    animate(save=False)
