from five_bar.five_bar import five_bar
import time
import numpy as np

# === Initialize robot ===
robot = five_bar(SERIAL_PORT="COM15", max_speed=100)

#robot._write_motor_zero_command()

robot.set_control_mode("position")
robot.set_target(45,-135)
robot.start()

vals=[[63.92, -242.82],
[75, -153],
[21.31, -105.03],
[-55, -117.61]]
i=0
while True:
    robot.set_target(vals[i][0],vals[i][1])
    #robot.set_target(0,0)
    print(robot.get_motors_data()["angle"])
    time.sleep(1)
    i+=1
    if i==len(vals):
        i=0
    # print(robot.target)


    # robot.set_target(-360,360)
    # time.sleep(1)
