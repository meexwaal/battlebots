import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

import socket
import time
from cffi import FFI

# Parse protocol.h for packet format definitions
ffi = FFI()
with open('protocol.h', 'r') as protocol_file:
    ffi.cdef(protocol_file.read())

ServerSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# ServerSocket.settimeout(1)
host = '0.0.0.0'
port = 2390

print("Start")
try:
    ServerSocket.bind((host, port))
except socket.error as e:
    print(str(e))
print("bound")

prev_theta = 0
prev_time = time.time()

plot_lidar_xy = np.zeros((100, 2))

def animate(save=False):
    fig, ax = plt.subplots()

    # Draw circle for bot
    bot_radius = 0.127
    bot_circle = plt.Circle((0, 0), bot_radius, fill=False, color='red')

    # Draw points for sensed orientation
    orientation, = ax.plot([], [], 'go', markersize=2)

    # Draw lidar history
    plot_lidar, = ax.plot([], [], 'ro', markersize=2)

    # Show some text
    plot_text = ax.text(-6, 6, 'foo')

    def init():
        ax.set_xlim(-7, 7)
        ax.set_ylim(-7, 7)

        ax.add_patch(bot_circle)
        return plot_lidar, bot_circle, orientation, plot_text

    def update(_):
        global prev_theta
        global prev_time

        try:
            telem_packet, address = ServerSocket.recvfrom(4096)
            telem_struct = ffi.from_buffer("telem_packet_t *", telem_packet)

            theta = telem_struct.theta
            rs = np.array(list(telem_struct.lidar_mm)) / 1e3
        except socket.timeout:
            theta = np.random.rand()
            rs = [np.random.rand()] * 16

        # while theta + np.pi < prev_theta:
        #     theta += 2 * np.pi
        # while prev_theta + np.pi < theta:
        #     prev_theta += 2 * np.pi

        # if theta > 2 * np.pi:
        #     theta -= 2 * np.pi
        #     prev_theta -= 2 * np.pi

        # thetas = np.linspace(theta, prev_theta, num=16, endpoint=False)[::-1]
        dtheta = np.diff(np.unwrap([prev_theta, theta]))[0]
        now = time.time()
        dt = now - prev_time
        prev_theta = theta
        prev_time = now

        # Plot measured theta
        orientation.set_data(
            [1.1 * bot_radius * np.cos(theta)],
            [1.1 * bot_radius * np.sin(theta)])

        # Show rotation speed
        rps = dtheta / dt / (2 * np.pi)
        rpm = rps * 60
        plot_text.set_text(f"{rps:.2f} rps, {rpm:.0f} rpm")

        # plot_lidar.set_data(rs * np.cos(thetas), rs * np.sin(thetas))
        plot_lidar_xy[:-1,:] = plot_lidar_xy[1:,:]
        plot_lidar_xy[-1] = (rs[0] * np.cos(theta), rs[0] * np.sin(theta))
        plot_lidar.set_data(plot_lidar_xy[:,0], plot_lidar_xy[:,1])

        return plot_lidar, bot_circle, orientation, plot_text


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
