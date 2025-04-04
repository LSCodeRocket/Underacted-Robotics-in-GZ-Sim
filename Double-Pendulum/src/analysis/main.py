import os
import csv
import numpy as np
import matplotlib.pyplot as plt

for file in os.listdir("logs"):
    if not file.endswith(".csv"):
        continue
    t = []
    p = []
    m = 0
    L = 0


    with open("logs/" + file, 'r', newline='') as logfile:
        reader = csv.reader(logfile)
        for i, row in enumerate(reader):
            if i == 0:
                continue

            t.append(float(row[0]))
            p.append(float(row[1]))
            m = (float(row[3]))
            L = (float(row[4]))

    plt.xlabel("TIME [s]")
    plt.ylabel("ANGLE [rad]")

    plt.plot(t, p)
    plt.plot(t, np.ones((len(t),1)) * np.pi, "r--")
    plt.plot(t, np.ones((len(t),1)) * -np.pi, "r--")
    plt.title("MASS : " + str(m) + "[kg] | LENGTH: " + str(L) + "[m]")
    plt.savefig("figures/fig_" + file + ".png")
    plt.clf();