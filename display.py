import math
import matplotlib.pyplot as plt

rmse_values = []
radar_x_values = []
radar_y_values = []
ladar_x_values = []
ladar_y_values = []
filter_y_values = []
filter_x_values = []

# Read sensor input
f = "./data/obj_pose-laser-radar-synthetic-input.txt"
data = open(f, "r")
if data.mode == 'r':
    contents = data.readlines()
    for line in contents:
        values = line.split("\t")
        if(values[0] == 'L'):
            x = float(values[1])
            y = float(values[2])
            ladar_x_values.append(x)
            ladar_y_values.append(y)
        else:
            rho = float(values[1])
            phi = float(values[2])
            rho_dot = float(values[3])
            x = rho * math.cos(phi)
            y = rho * math.sin(phi)
            radar_x_values.append(x)
            radar_y_values.append(y)

# Read filter output
data = open('./data/filter_output.txt', "r")
if data.mode == 'r':
    contents = data.readlines()
    for line in contents:
        values = line.split("\t")
        x = float(values[0])
        y = float(values[1])
        filter_x_values.append(x)
        filter_y_values.append(y)

# # Read filter values
data = open('./data/rmse_data.txt', "r")
if data.mode == 'r':
    contents = data.readlines()
    for line in contents:
        rmse_values.append(float(line.split('\n')[0]))

# display results
fig = plt.figure()
ax=fig.add_axes([0,0,1,1])
fig.suptitle("Extended Kalman Filter", fontsize=16)

ax.scatter(radar_x_values, radar_y_values, color='r', s=4)
ax.scatter(ladar_x_values, ladar_y_values, color='b', s=4)
ax.scatter(filter_x_values, filter_y_values, color='g', s=4)

legend1 = ax.legend(['Radar Reading', 'Ladar Reading', 'EKF Output'], loc="upper left", title="Legend", prop={'size': 12})
ax.add_artist(legend1)
fig.set_size_inches(10,7)

props = dict(boxstyle='round', facecolor='white', alpha=0.2)
ax.text(12, -8, 'RMSE Values:\npx = {}\npy = {}\nvx = {}\nvy = {}'.format(rmse_values[0], rmse_values[1], rmse_values[2], rmse_values[3]), fontsize=12, bbox=props)

plt.savefig('./data/result.png')
plt.show()