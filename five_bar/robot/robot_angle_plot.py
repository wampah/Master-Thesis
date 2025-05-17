from five_bar.five_bar import five_bar
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import numpy as np

# === Initialize robot ===
robot = five_bar(SERIAL_PORT="COM3", max_speed=300)
robot.set_control_mode("speed",0,0)

robot.start()

# vals=[[135,-135],
# [117.91, -217.76],
# [226.68, -93.44],
# [157.75, -92.08],
# [117.11, -198.33],
# [130.25, -125.78],
# [199.07, -195.59]]

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

ax.set_ylim(-1500, 1500)  # Adjust based on your robot's angle range
ax.set_xlabel("Time (s)")
ax.set_ylabel("Torque (Nm)")
ax.legend()
plt.title("Live Motor Torque")

t_0=time.time()
i=1

vals=[-1000,0,1000,0]

target=[-1000,-1000]
robot.set_target(*target)
# === Update function for animation ===
def update(frame):
    global t_0, i , target
    
    if time.time()-t_0>4:
        t_0=time.time()
        robot.set_target(vals[i],vals[i])
        i+=1
        if i==len(vals):
            i=0

            
    current_time = time.time() - start_time
    angles = robot.get_motors_data()["speed"]

    timestamps.append(current_time)
    angle1.append(angles[0])
    angle2.append(angles[1])

    line1.set_data(timestamps, angle1)
    line2.set_data(timestamps, angle2)

    ax.set_xlim(max(0, timestamps[0]), timestamps[-1] + 0.1)
    return line1, line2

# === Animate ===
ani = animation.FuncAnimation(fig, update, interval=10)  # 100ms = 0.1s

plt.tight_layout()
plt.show()
