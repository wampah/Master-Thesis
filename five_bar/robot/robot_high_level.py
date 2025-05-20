from five_bar_robot.five_bar import five_bar
import time
import numpy as np
import pandas as pd

log=[]
# === Initialize robot ===
robot = five_bar(SERIAL_PORT="COM16", max_speed=50)

#robot._write_motor_zero_command()

target1=135
target2=-135

robot.set_control_mode("position",target1,target2) # available modes are position, torque and speed
time.sleep(2)
robot.start()

vals=[[135,-135],
[117.91, -217.76],
[226.68, -93.44],
[157.75, -92.08],
[117.11, -198.33],
[130.25, -125.78],
[199.07, -195.59]]

t_0=time.time()

i=0
while True:
    
    if time.time()-t_0>2:
        rand_int=np.random.randint(len(vals))
        robot.set_target(vals[rand_int][0],vals[rand_int][1])
        t_0=time.time()
    
    print(", ".join([f"{angle:.2f}" for angle in robot.get_motors_data()["angle"]]))
    time.sleep(0.01)

