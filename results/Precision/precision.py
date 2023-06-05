#!/usr/bin/python3

import matplotlib as mpl
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

df_0deg  = pd.read_csv("new_data_0deg_euler_adjust2.csv" , sep=';')
df_5deg  = pd.read_csv("new_data_5deg_euler_adjust2.csv" , sep=';')
df_10deg = pd.read_csv("new_data_10deg_euler_adjust2.csv", sep=';')
df_15deg = pd.read_csv("new_data_15deg_euler_adjust2.csv", sep=';')

# df_0deg["x"] = df_0deg["x"].abs()
# df_5deg["x"] = df_5deg["x"].abs()
# df_10deg["x"] = df_10deg["x"].abs()
# df_15deg["x"] = df_15deg["x"].abs()

# df_0deg["y"] = df_0deg["y"].abs()
# df_5deg["y"] = df_5deg["y"].abs()
# df_10deg["y"] = df_10deg["y"].abs()
# df_15deg["y"] = df_15deg["y"].abs()

# 0 degree
est_angles_0deg_x = df_0deg["x"].to_numpy()
ref_angle_0deg_x = np.zeros_like(est_angles_0deg_x)
mean_0deg_x = df_0deg["x"].mean()
stdev_0deg_x = df_0deg["x"].std()

# 5 degree
est_angles_5deg_x = df_5deg["x"].to_numpy()
ref_angle_5deg_x = 5.0 * np.ones_like(est_angles_5deg_x)
mean_5deg_x = df_5deg["x"].mean()
stdev_5deg_x = df_5deg["x"].std()

# 10 degree
est_angles_10deg_x = df_10deg["x"].to_numpy()
ref_angle_10deg_x = 10.0 * np.ones_like(est_angles_10deg_x)
mean_10deg_x = df_10deg["x"].mean()
stdev_10deg_x = df_10deg["x"].std()

# 15 degree
est_angles_15deg_x = df_15deg["x"].to_numpy()
ref_angle_15deg_x = 15.0 * np.ones_like(est_angles_15deg_x)
mean_15deg_x = df_15deg["x"].mean()
stdev_15deg_x = df_15deg["x"].std()

fig_x = plt.figure()
fig_x.set_dpi(300)
fig_x.set_size_inches(8, 4)
# fig_x.set_tight_layout(True)

plt.rc('text', usetex=True)
# plt.rc('font', family="Times")

plt.xlim((-2, 16))
plt.xticks(np.linspace(-2, 15, 18), fontsize=6)
plt.ylim((-1, 16))
plt.yticks(np.linspace(-1, 15, 17), fontsize=6)

plt.scatter(est_angles_0deg_x, ref_angle_0deg_x, s=1)
plt.text(mean_0deg_x * 1.5, 1.25, "Mean = "          + str(round(mean_0deg_x, 3)), fontsize=4.5)
plt.text(mean_0deg_x * 1.5, 0.75, "Stdev = "         + str(round(stdev_0deg_x, 3)), fontsize=4.5)
plt.text(mean_0deg_x * 1.5, 0.25, "No. of points = " + str(len(est_angles_0deg_x)), fontsize=4.5)

plt.scatter(est_angles_5deg_x, ref_angle_5deg_x, s=1)
plt.text(mean_5deg_x * 1.05, 6.5, "Mean = "          + str(round(mean_5deg_x, 3)) , fontsize=4.5)
plt.text(mean_5deg_x * 1.05, 6.0, "Stdev = "         + str(round(stdev_5deg_x, 3)), fontsize=4.5)
plt.text(mean_5deg_x * 1.05, 5.5, "No. of points = " + str(len(est_angles_5deg_x)), fontsize=4.5)

plt.scatter(est_angles_10deg_x, ref_angle_10deg_x, s=1)
plt.text(mean_10deg_x * 1.025, 11.5, "Mean = "          + str(round(mean_10deg_x, 3)),  fontsize=4.5)
plt.text(mean_10deg_x * 1.025, 11.0, "Stdev = "         + str(round(stdev_10deg_x, 3)), fontsize=4.5)
plt.text(mean_10deg_x * 1.025, 10.5, "No. of points = " + str(len(est_angles_10deg_x)), fontsize=4.5)

plt.scatter(est_angles_15deg_x, ref_angle_15deg_x, s=1)
plt.text(mean_15deg_x * 0.87, 14.5, "Mean = "          + str(round(mean_15deg_x, 3)),  fontsize=4.5)
plt.text(mean_15deg_x * 0.87, 14.0, "Stdev = "         + str(round(stdev_15deg_x, 3)), fontsize=4.5)
plt.text(mean_15deg_x * 0.87, 13.5, "No. of points = " + str(len(est_angles_15deg_x)), fontsize=4.5)

fig_x.legend(['0 deg', '5 deg', '10 deg', '15 deg'], fontsize=5, loc='center left', bbox_to_anchor=(0.91, 0.81))

plt.plot(mean_0deg_x  * np.ones_like(est_angles_0deg_x) , np.linspace(-1, 16 , len(est_angles_0deg_x)) , linewidth=0.5)
plt.plot(mean_5deg_x  * np.ones_like(est_angles_5deg_x) , np.linspace(-1, 16 , len(est_angles_5deg_x)) , linewidth=0.5)
plt.plot(mean_10deg_x * np.ones_like(est_angles_10deg_x), np.linspace(-1, 16, len(est_angles_10deg_x)), linewidth=0.5)
plt.plot(mean_15deg_x * np.ones_like(est_angles_15deg_x), np.linspace(-1, 16, len(est_angles_15deg_x)), linewidth=0.5)

plt.scatter(mean_0deg_x, 0.0, s=8, marker='|', color='k')
plt.scatter(mean_5deg_x, 5.0, s=8, marker='|', color='k')
plt.scatter(mean_10deg_x, 10.0, s=8, marker='|', color='k')
plt.scatter(mean_15deg_x, 15.0, s=8, marker='|', color='k')

plt.grid(linewidth=0.15)

plt.title("Estimated Camera Orientation (angle around $x$)", fontsize=9)
plt.xlabel(r"Estimated ($x$) Angles", fontsize=7)
plt.ylabel(r"Ground Truth Angles", fontsize=7)

fig_x.show()

## Estimated y angles
# 0 degree
est_angles_0deg_y = df_0deg["y"].to_numpy()
ref_angle_0deg_y = np.zeros_like(est_angles_0deg_y)
mean_0deg_y = df_0deg["y"].mean()
stdev_0deg_y = df_0deg["y"].std()

# 5 degree
est_angles_5deg_y = df_5deg["y"].to_numpy()
ref_angle_5deg_y = 5.0 * np.ones_like(est_angles_5deg_y)
mean_5deg_y = df_5deg["y"].mean()
stdev_5deg_y = df_5deg["y"].std()

# 10 degree
est_angles_10deg_y = df_10deg["y"].to_numpy()
ref_angle_10deg_y = 10.0 * np.ones_like(est_angles_10deg_y)
mean_10deg_y = df_10deg["y"].mean()
stdev_10deg_y = df_10deg["y"].std()

# 15 degree
est_angles_15deg_y = df_15deg["y"].to_numpy()
ref_angle_15deg_y = 15.0 * np.ones_like(est_angles_15deg_y)
mean_15deg_y = df_15deg["y"].mean()
stdev_15deg_y = df_15deg["y"].std()

fig_y = plt.figure()
fig_y.set_dpi(300)
fig_y.set_size_inches(8, 4)
# fig_x.set_tight_layout(True)

plt.rc('text', usetex=True)
# plt.rc('font', family="Times")

plt.xlim((-3.5, -2.5))
plt.xticks(np.linspace(-3.5, -2.5, 11), fontsize=6)
plt.ylim((-1, 16))
plt.yticks(np.linspace(-1, 15, 17), fontsize=6)

plt.scatter(est_angles_0deg_y, ref_angle_0deg_y, s=1)
plt.text(mean_0deg_y, 1.25, "Mean = "          + str(round(mean_0deg_y, 3)), fontsize=4.5)
plt.text(mean_0deg_y, 0.75, "Stdev = "         + str(round(stdev_0deg_y, 3)), fontsize=4.5)
plt.text(mean_0deg_y, 0.25, "No. of points = " + str(len(est_angles_0deg_y)), fontsize=4.5)

plt.scatter(est_angles_5deg_y, ref_angle_5deg_y, s=1)
plt.text(mean_5deg_y, 6.5, "Mean = "          + str(round(mean_5deg_y, 3)) , fontsize=4.5)
plt.text(mean_5deg_y, 6.0, "Stdev = "         + str(round(stdev_5deg_y, 3)), fontsize=4.5)
plt.text(mean_5deg_y, 5.5, "No. of points = " + str(len(est_angles_5deg_y)), fontsize=4.5)

plt.scatter(est_angles_10deg_y, ref_angle_10deg_y, s=1)
plt.text(mean_10deg_y, 11.5, "Mean = "          + str(round(mean_10deg_y, 3)),  fontsize=4.5)
plt.text(mean_10deg_y, 11.0, "Stdev = "         + str(round(stdev_10deg_y, 3)), fontsize=4.5)
plt.text(mean_10deg_y, 10.5, "No. of points = " + str(len(est_angles_10deg_y)), fontsize=4.5)

plt.scatter(est_angles_15deg_y, ref_angle_15deg_y, s=1)
plt.text(mean_15deg_y, 14.5, "Mean = "          + str(round(mean_15deg_y, 3)),  fontsize=4.5)
plt.text(mean_15deg_y, 14.0, "Stdev = "         + str(round(stdev_15deg_y, 3)), fontsize=4.5)
plt.text(mean_15deg_y, 13.5, "No. of points = " + str(len(est_angles_15deg_y)), fontsize=4.5)

fig_y.legend(['0 deg', '5 deg', '10 deg', '15 deg'], fontsize=5, loc='center left', bbox_to_anchor=(0.91, 0.81))

plt.plot(mean_0deg_y  * np.ones_like(est_angles_0deg_y) , np.linspace(-1, 16 , len(est_angles_0deg_y)) , linewidth=0.45)
plt.plot(mean_5deg_y  * np.ones_like(est_angles_5deg_y) , np.linspace(-1, 16 , len(est_angles_5deg_y)) , linewidth=0.45)
plt.plot(mean_10deg_y * np.ones_like(est_angles_10deg_y), np.linspace(-1, 16, len(est_angles_10deg_y)), linewidth=0.45)
plt.plot(mean_15deg_y * np.ones_like(est_angles_15deg_y), np.linspace(-1, 16, len(est_angles_15deg_y)), linewidth=0.45)

plt.scatter(mean_0deg_y, 0.0, s=8, marker='|', color='k')
plt.scatter(mean_5deg_y, 5.0, s=8, marker='|', color='k')
plt.scatter(mean_10deg_y, 10.0, s=8, marker='|', color='k')
plt.scatter(mean_15deg_y, 15.0, s=8, marker='|', color='k')

plt.grid(linewidth=0.15)

plt.title("Estimated Camera Orientation (angle around $y$)", fontsize=9)
plt.xlabel(r"Estimated ($y$) Angles", fontsize=7)
plt.ylabel(r"Ground Truth Angles", fontsize=7)

fig_y.show()

## Standard deviation
fig2 = plt.figure()
fig2.set_dpi(300)
fig2.set_size_inches(6,4)

plt.title("Standard deviation of estimated angles", fontsize=9)
plt.ylabel("Standard Deviation", fontsize=7)
plt.xlabel("Ground Truth Angles", fontsize=7)

plt.xlim((-1, 16))
plt.xticks(np.linspace(-1, 15, 17), fontsize=6)
plt.ylim((0.0, 1.25))
plt.yticks(np.linspace(0,1.25,6), fontsize=6)

plt.scatter(np.array([0.0, 5.0, 10.0, 15.0]), 
            np.array([stdev_0deg_x, stdev_5deg_x, stdev_10deg_x, stdev_15deg_x]), s=5, label='$x$ angles')
plt.scatter(np.array([0.0, 5.0, 10.0, 15.0]), 
            np.array([stdev_0deg_y, stdev_5deg_y, stdev_10deg_y, stdev_15deg_y]), s=5, label='$y$ angles')

fig2.legend(fontsize=5, ncol=2)

plt.grid(linewidth=0.25)

fig2.show()

fig_x.savefig("Estimated Camera Orientation - angle around x.pdf")
fig_y.savefig("Estimated Camera Orientation - angle around y.pdf")
fig2.savefig("Standard deviation of estimated angles.pdf")

# plt.show()

