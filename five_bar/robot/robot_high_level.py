from five_bar.five_bar import five_bar
import time
import numpy as np
robot=five_bar(SERIAL_PORT="COM15",pos_max_speed=int(5e6))

robot.begin()
robot.change_mode("torque")
time.sleep(1)
robot.set_target(0,0)
while True:
    t1,t2=np.sin(time.time()),-np.sin(time.time())
    robot.set_target(t1,t2)
    torques = robot.get_motors_data()["angle"]
    #print(", ".join(f"{t:.2f}" for t in torques))
    print(t1,t2)
    time.sleep(0.01) 