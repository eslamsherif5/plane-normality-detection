#!/usr/bin/python3

import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

df_0deg  = pd.read_csv("new_data_trial1/new_data_trial1_0deg.csv" , sep=';')
df_5deg  = pd.read_csv("new_data_trial1/new_data_trial1_5deg.csv" , sep=';')
df_10deg = pd.read_csv("new_data_trial1/new_data_trial1_10deg.csv", sep=';')
df_15deg = pd.read_csv("new_data_trial1/new_data_trial1_15deg.csv", sep=';')

est_angles_0deg = df_0deg["Estimated Angle"].to_numpy()
ref_angle_0deg = np.zeros_like(est_angles_0deg)
mean_0deg = df_0deg["Estimated Angle"].mean()
stdev_0deg = df_0deg["Estimated Angle"].std()

est_angles_5deg = df_5deg["Estimated Angle"].to_numpy()
ref_angle_5deg = 5.0 * np.ones_like(est_angles_5deg)
mean_5deg = df_5deg["Estimated Angle"].mean()
stdev_5deg = df_5deg["Estimated Angle"].std()

est_angles_10deg = df_10deg["Estimated Angle"].to_numpy()
ref_angle_10deg = 10.0 * np.ones_like(est_angles_10deg)
mean_10deg = df_10deg["Estimated Angle"].mean()
stdev_10deg = df_10deg["Estimated Angle"].std()

est_angles_15deg = df_15deg["Estimated Angle"].to_numpy()
ref_angle_15deg = 15.0 * np.ones_like(est_angles_15deg)
mean_15deg = df_15deg["Estimated Angle"].mean()
stdev_15deg = df_15deg["Estimated Angle"].std()

fig1, ax1 = plt.subplots()
fig1.set_dpi(300)
fig1.set_size_inches(8, 4)
# fig1.set_tight_layout(True)

# mpl.rcParams['text.usetex'] = True
# mpl.rcParams['font.family'] = "Times"
# mpl.rcParams['text.latex.preamble'] = r'\usepackage{amsmath}'# \usepackage{amssymb}'

# plt.rcParams.({
#     "text.usetex": True,
#     "font.family": "serif",
#     "font.monospace": 'Times'
#     })

plt.xlim((0, 16))
plt.ylim((-0.5, 16))
plt.xticks(np.linspace(0, 15, 16), fontsize=6)
plt.yticks(np.linspace(0, 15, 7), fontsize=6)

plt.scatter(est_angles_0deg, ref_angle_0deg, s=2)
ax1.annotate("Mean = " + str(round(mean_0deg, 3)),
             (mean_0deg+0.05, 1.25), fontsize=4.5)
ax1.annotate("Stdev = " + str(round(stdev_0deg, 3)),
             (mean_0deg+0.05, 0.75), fontsize=4.5)
ax1.annotate("# of points = " + str(len(est_angles_0deg)),
             (mean_0deg+0.05, 0.25), fontsize=4.5)

plt.scatter(est_angles_5deg, ref_angle_5deg, s=2)
ax1.annotate("Mean = " + str(round(mean_5deg, 3)),
             (mean_5deg-0.25, 6.5), fontsize=4.5)
ax1.annotate("Stdev = " + str(round(stdev_5deg, 3)),
             (mean_5deg-0.25, 6.0), fontsize=4.5)
ax1.annotate("# of points = " + str(len(est_angles_5deg)),
             (mean_5deg-0.25, 5.5), fontsize=4.5)

plt.scatter(est_angles_10deg, ref_angle_10deg, s=2)
ax1.annotate("Mean = " + str(round(mean_10deg, 3)),
             (mean_10deg-0.25, 11.5), fontsize=4.5)
ax1.annotate("Stdev = " + str(round(stdev_10deg, 3)),
             (mean_10deg-0.25, 11.0), fontsize=4.5)
ax1.annotate("# of points = " + str(len(est_angles_10deg)),
             (mean_10deg-0.25, 10.5), fontsize=4.5)

plt.scatter(est_angles_15deg, ref_angle_15deg, s=2)
ax1.annotate("Mean = " + str(round(mean_15deg, 3)),
             (mean_15deg - 0.75, 14.5), fontsize=4.5)
ax1.annotate("Stdev = " + str(round(stdev_15deg, 3)),
             (mean_15deg - 0.75, 14.0), fontsize=4.5)
ax1.annotate("# of points = " + str(len(est_angles_15deg)),
             (mean_15deg - 0.75, 13.5), fontsize=4.5)

fig1.legend(['0 deg', '5 deg', '10 deg', '15 deg'], fontsize=5,
            loc='center left', bbox_to_anchor=(0.91, 0.82))

plt.plot(mean_0deg * np.ones_like(est_angles_0deg),
         np.linspace(-1, mean_0deg - 1, len(est_angles_0deg)), linewidth=0.4)
plt.plot(mean_5deg * np.ones_like(est_angles_5deg), np.linspace(-1,
         mean_5deg + 0.1, len(est_angles_5deg)), linewidth=0.4)
plt.plot(mean_10deg * np.ones_like(est_angles_10deg), np.linspace(-1,
         mean_10deg + 0.1, len(est_angles_10deg)), linewidth=0.4)
plt.plot(mean_15deg * np.ones_like(est_angles_15deg), np.linspace(-1,
         mean_15deg + 0.1, len(est_angles_15deg)), linewidth=0.4)

plt.scatter(mean_0deg, 0.0, s=15, marker='*')
plt.scatter(mean_5deg, 5.0, s=15, marker='*')
plt.scatter(mean_10deg, 10.0, s=15, marker='*')
plt.scatter(mean_15deg, 15.0, s=15, marker='*')

plt.grid(linewidth=0.15)

plt.xlabel("Estimated Angles", fontsize=8)
plt.ylabel("Ground Truth Angles", fontsize=8)

fig1.show()

fig2, ax2 = plt.subplots()
fig2.set_dpi(150)

plt.ylabel("Standard Deviation", fontsize=10)
plt.xlabel("Ground Truth Angles", fontsize=10)

plt.ylim((0.0, 0.5))
plt.xlim((-1, 16))

# plt.yticks(np.linspace(0,15,7), fontsize=6)
plt.xticks(np.array([0, 5, 10, 15]), fontsize=8)

plt.scatter(np.array([0.0, 5.0, 10.0, 15.0]), np.array(
    [stdev_0deg, stdev_5deg, stdev_10deg, stdev_15deg]))

plt.grid(linewidth=0.25)
fig2.show()

# plt.show()

fig1.savefig("fig1.pdf")
fig2.savefig("fig2.pdf")
