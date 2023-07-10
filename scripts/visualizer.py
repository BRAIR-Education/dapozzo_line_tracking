import os

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

import rospkg


class Visualizer:
    def __init__(self):
        self.rospack = rospkg.RosPack()

    def visualize_data(self):
        pkgdir = self.rospack.get_path("dapozzo_line_tracking")
        logdir = os.path.join(pkgdir, "logs")

        while True:
            filenames = []

            entries = os.scandir(logdir)
            for entry in entries:
                if entry.is_dir():
                    continue

                filenames.append(entry.name)

            file_count = len(filenames)

            print(f"{file_count} available log(s):")
            for i, filename in enumerate(filenames):
                print(f"{i}) - {filename}")

            print(f"{file_count}) - Quit")
            filenames.append("")

            choice = -1
            while choice < 0 or choice > file_count:
                choice = int(input("Input the index of the desired log: "))

            if choice == file_count:
                print("Quitting.")
                return

            filepath = os.path.join(logdir, filenames[choice])

            columns = ["Time", "Error"]
            bb_data = pd.read_csv(filepath, usecols=columns)
            plt.plot(np.array(bb_data.Time), np.array(bb_data.Error))
            plt.show()


if __name__ == "__main__":
    viz = Visualizer()

    try:
        viz.visualize_data()
    except KeyboardInterrupt:
        pass
