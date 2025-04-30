from five_bar.five_bar import five_bar
import time
import numpy as np
import pandas as pd
import os

robot = five_bar(SERIAL_PORT="COM16", max_speed=200)
robot.set_control_mode("position")

target1=135
target2=-135

robot.set_target(target1,target2)

robot.start()



time.sleep(5)


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
t_series_start = time.time()
t_last_change = t_series_start

while current_series_id<20:
    # Get current time and data
    t_now = time.time()
    motor_data = robot.get_motors_data()
    
    # Record data point
    data_log.append({
        'time': t_now - t_series_start,
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
    
    # Print current angles
    #print(", ".join([f"{angle:.2f}" for angle in motor_data["angle"]]))
    
    # Change target every 2 seconds
    if t_now - t_last_change > 2:
        current_series_id += 1
        new_target = vals[current_series_id % len(vals)]
        robot.set_target(new_target[0], new_target[1])
        t_last_change = t_now
        t_series_start = t_now  # Reset timer for new series
        
    time.sleep(0.05)

# Final save when loop exits
df = pd.DataFrame(data_log)
df.to_csv(os.path.join('five_bar','digital_twin','data','robot_data_final.csv'), index=False)

