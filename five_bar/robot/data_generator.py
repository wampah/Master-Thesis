from five_bar.five_bar import five_bar
import time
import numpy as np
import pandas as pd
import os

robot = five_bar(SERIAL_PORT="COM16", max_speed=200)
robot.set_control_mode("position", 135, -135)
robot.start()
time.sleep(5)  # Let robot settle

vals = [
    [135, -135],
    [117.91, -217.76],
    [226.68, -93.44],
    [157.75, -92.08],
    [117.11, -198.33],
    [130.25, -125.78],
    [199.07, -195.59]
]

# Data collection setup
data_log = []
current_series_id = 0
series_duration = 3
start_time = time.time()
series_phase = 0

while current_series_id < 20:
    now = time.time()
    t_series = now - start_time

    # Get and log data
    motor_data = robot.get_motors_data()
    data_log.append({
        'time': t_series,
        'series_id': current_series_id,
        'angle1': motor_data["angle"][0],
        'angle2': motor_data["angle"][1],
        'speed1': motor_data["speed"][0],
        'speed2': motor_data["speed"][1],
        'torque1': motor_data["torque"][0],
        'torque2': motor_data["torque"][1],
        'target1': vals[current_series_id % len(vals)][0],
        'target2': vals[current_series_id % len(vals)][1]
    })

    # Timing control logic
    if series_phase == 0 and t_series >= 0:
        robot.set_control_mode("torque", 0, 0)
        series_phase = 1

    elif series_phase == 1 and t_series >= 0.5:
        rand_int = np.random.randint(len(vals))
        target = vals[rand_int]
        robot.set_control_mode("position", target[0], target[1])
        series_phase = 2

    elif series_phase == 2 and t_series >= 2.5:
        robot.set_control_mode("torque", 0, 0)
        series_phase = 3

    elif series_phase == 3 and t_series >= 0.5:
        current_series_id += 1
        start_time = now
        series_phase = 0

    time.sleep(0.01)

# Save data
df = pd.DataFrame(data_log)
df.to_csv(os.path.join('five_bar', 'digital_twin', 'data', 'robot_data_final.csv'), index=False)
