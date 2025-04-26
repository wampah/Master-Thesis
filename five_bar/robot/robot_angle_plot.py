from five_bar.five_bar import five_bar
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import numpy as np

# === Initialize robot ===
robot = five_bar(SERIAL_PORT="COM15", max_speed=100)
robot.set_control_mode("position")

robot.set_target(45,-135)
robot.start()

vals=[[63.92, -242.82],
[75, -153],
[21.31, -105.03],
[-55, -117.61]]

# === Parameters ===
max_len = 100  # Rolling window size (number of data points)
start_time = time.time()

# === Data buffers ===
timestamps = deque([0]*max_len, maxlen=max_len)
angle1 = deque([0]*max_len, maxlen=max_len)
angle2 = deque([0]*max_len, maxlen=max_len)

# === Plot setup ===
fig, ax = plt.subplots()
line1, = ax.plot([], [], label='Motor 1')
line2, = ax.plot([], [], label='Motor 2')

ax.set_ylim(-1, 1)  # Adjust based on your robot's angle range
ax.set_xlabel("Time (s)")
ax.set_ylabel("Angle (deg)")
ax.legend()
plt.title("Live Motor Angles")

t_0=time.time()
i=0

# === Update function for animation ===
def update(frame):
    global t_0, i
    if time.time()-t_0>2:
        t_0=time.time()
        robot.set_target(vals[i][0],vals[i][1])
        i+=1
        if i==len(vals):
            i=0
            
            
    current_time = time.time() - start_time
    angles = robot.get_motors_data()["torque"]

    timestamps.append(current_time)
    angle1.append(angles[0])
    angle2.append(angles[1])

    line1.set_data(timestamps, angle1)
    line2.set_data(timestamps, angle2)

    ax.set_xlim(max(0, timestamps[0]), timestamps[-1] + 0.1)
    return line1, line2

# === Animate ===
ani = animation.FuncAnimation(fig, update, interval=1)  # 100ms = 0.1s

plt.tight_layout()
plt.show()
