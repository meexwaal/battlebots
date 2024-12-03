"""This file reads from the specified CSV file and constantly animates its changes."""
import os

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import pandas as pd

channels: list[str] = os.listdir("./umb/live")


fig, plots = plt.subplots(2)

# Has to be global, I think
def animate(_):
    data = pd.read_csv(os.path.join("./umb/live", "theta"))

    timestamps = data["timestamp"].tail(1000).to_list()

    data = data.drop("timestamp", axis=1)

    # for i, column in enumerate(data.columns):
    plots[0].clear()
    plots[0].set_title("theta")
    plots[0].plot(timestamps, data["theta"].tail(1000).to_list())
    plots[0].set_xticks([])

# Store the animation call in a pointer so Python doesn't GC it away immediately
animation_ptr = animation.FuncAnimation(fig, animate, interval=200)

plt.show()