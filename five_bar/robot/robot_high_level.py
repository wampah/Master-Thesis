from five_bar.five_bar import five_bar
import time
import numpy as np

robot=five_bar(SERIAL_PORT="COM15",pos_max_speed=int(5e6))

robot.begin() # Default control mode is torque

robot.change_mode("position")

while True:
    robot.set_target(360*np.sin(time.time()),360*np.sin(time.time())) #Set targets
    info = robot.get_motors_data()
    print(info)
    time.sleep(0.1) 