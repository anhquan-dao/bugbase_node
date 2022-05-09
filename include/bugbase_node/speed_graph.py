#!/usr/bin/python
# -*- coding: utf-8 -*-

import datetime as dt
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class SpeedVisualizer:
    def __init__(self):   
        # Create figure for plotting
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)
        self.xs = []
        self.ys = []

        self.updated = True
        self.data = 0

        # Set up plot to call animate() function periodically
        self.ani = animation.FuncAnimation(self.fig, self.animate, interval=1000)
        plt.show()

        # This function is called periodically from FuncAnimation
    def animate(self, i):

        if self.updated:
            # Add x and y to lists
            self.xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
            self.ys.append(self.data)
            # self.updated = False

            # Limit x and y lists to 20 items
            self.xs = self.xs[-20:]
            self.ys = self.ys[-20:]

            # Draw x and y lists
            self.ax.clear()
            self.ax.plot(self.xs, self.ys)

            # Format plot
            plt.xticks(rotation=45, ha='right')
            plt.subplots_adjust(bottom=0.30)
            plt.title('TMP102 Temperature over Time')
            plt.ylabel('Temperature (deg C)')

    def update_data(self, data):
        self.data = data

if __name__ == "__main__":
    
    visualizer = SpeedVisualizer()

    visualizer.data = 100
    # plt.show()

        