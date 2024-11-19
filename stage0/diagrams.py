"""This file reads from the specified CSV file and constantly animates its changes."""
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import pandas as pd

EVENT_FILE = "./umb/2024-11-18--23-30-43.csv"

data = pd.read_csv(EVENT_FILE, index_col=0)

fig, plots = plt.subplots(len(data.columns) - 1)

# Has to be global, I think
def animate(_):
    data = pd.read_csv(EVENT_FILE, index_col=0)

    timestamps = data["timestamp"].tail(16 * 30).to_list()

    data = data.drop("timestamp", axis=1)

    for i, column in enumerate(data.columns):
        plots[i].clear()
        plots[i].set_title(column)
        plots[i].plot(timestamps, data[column].tail(16 * 30).to_list())

# Store the animation call in a pointer so Python doesn't GC it away immediately
animation_ptr = animation.FuncAnimation(fig, animate, interval=200)

plt.show()