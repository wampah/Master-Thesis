from five_bar.five_bar import five_bar
import time
import numpy as np

# === Initialize robot ===
robot = five_bar(SERIAL_PORT="COM15", max_speed=500)
robot.change_mode("torque")
vals=0
while True:
    robot.set_target(1,1)
    print(robot.target)
    time.sleep(0.1)

    # robot.set_target(-360,360)
    # time.sleep(1)
