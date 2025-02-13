import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
from numpy.linalg import inv
import matplotlib.pyplot as plt
import pandas as pd
from scipy.interpolate import interp1d

df = pd.read_csv(os.path.join(os.path.dirname(__file__), 'trajectory_data.csv'), delimiter=",",header=None, names=['time', 'q1', 'q2', 'dq1', 'dq2'])

interp_q1 = interp1d(df['time'], df['q1'], kind='linear', fill_value='extrapolate')
interp_q2 = interp1d(df['time'], df['q2'], kind='linear', fill_value='extrapolate')

interp_dq1 = interp1d(df['time'], df['dq1'], kind='linear', fill_value='extrapolate')
interp_dq2 = interp1d(df['time'], df['dq2'], kind='linear', fill_value='extrapolate')

q1_0=df.q1[0]
q2_0=df.q2[0]
xml_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "assets\\5_barras.xml") #xml file (assumes this is in the same folder as this file)
simend = 4 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    q1=data.qpos[0]
    q2=data.qpos[2]
    
    dq1 = data.qvel[0]
    dq2 = data.qvel[2]
    
    ddq1 = data.qacc[0]
    ddq2 = data.qacc[2]
    
    pos_sens_x=data.sensordata[0]
    pos_sens_y=data.sensordata[1]
    
    c = 0.5
    k=10
    f=.05
    
    #Gives time for the robot to get to the initial position of the trajectory
    if(data.time<2):
        data.ctrl[0] = -10*(q1-(q1_0))-0.5*dq1
        data.ctrl[1] = -10*(q2-(q2_0))-0.5*dq2
    else:
        data.ctrl[0] = -0.01*(q1-(interp_q1(data.time-2)))-0.5*(dq1-interp_dq1(data.time-2))
        data.ctrl[1] = -0.01*(q2-(interp_q2(data.time-2)))-0.5*(dq2-interp_dq2(data.time-2))
        



def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)


# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

print([model.geom(i).name for i in range(model.ngeom)])
print(data.qpos)

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
# cam.azimuth = 90
# cam.elevation = -45
# cam.distance = 2
# cam.lookat = np.array([0.0, 0.0, 0])
cam.azimuth = 90.11676025390628 ; cam.elevation = -50.03149414062499 ; cam.distance =  2
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])

#initialize the controll er
init_controller(model,data)

data.qvel[0] = 0
data.qvel[2] = 0

#set the controller
mj.set_mjcb_control(controller)

times=[]
pxs=[]
pys=[]

vxs=[]
vys=[]

axs=[]
ays=[]

eff_xs=[]
eff_ys=[]
while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        mj.mj_step(model, data)
        if data.time>=2:
            times.append(data.time-2)
            pxs.append(data.qpos[0])
            pys.append(data.qpos[2])

            vxs.append(data.qvel[0])
            vys.append(data.qvel[2])

            axs.append(data.qacc[0])
            ays.append(data.qacc[2])
            
            eff_xs.append(data.sensordata[0])
            eff_ys.append(data.sensordata[1])
            

        
    if (data.time>=simend):
        break

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()

# Creating subplots
fig, axis = plt.subplots(4, 1, figsize=(8, 12))

time_obj=np.linspace(0,2,20)

# Plotting positions
axis[0].plot(times, pxs,'r-', label='q1')
axis[0].plot(times, pys, 'b-',label='q2')
axis[0].plot(time_obj, interp_q1(time_obj),'r*', label='q1_obj')
axis[0].plot(time_obj, interp_q2(time_obj),'b*', label='q1_obj')
axis[0].set_title('Positions')
axis[0].set_xlabel('Time')
axis[0].set_ylabel('Position')
axis[0].legend()

# Plotting velocities
axis[1].plot(times, vxs, 'r-',label='dq1')
axis[1].plot(times, vys, 'b-',label='dq2')
axis[1].plot(time_obj, interp_dq1(time_obj),'r*', label='dq1_obj')
axis[1].plot(time_obj, interp_dq2(time_obj),'b*', label='dq1_obj')
axis[1].set_title('Velocities')
axis[1].set_xlabel('Time')
axis[1].set_ylabel('Velocity')
axis[1].legend()

# Plotting accelerations
axis[2].plot(times, axs, 'r-',label='ddq1')
axis[2].plot(times, ays, 'b-',label='ddq2')
axis[2].set_title('Accelerations')
axis[2].set_xlabel('Time')
axis[2].set_ylabel('Acceleration')
axis[2].legend()

# Plotting accelerations
axis[3].plot(times, eff_xs, 'r-',label='eff_x')
axis[3].plot(times, eff_ys, 'b-',label='eff_y')
axis[3].set_title('Effector Position')
axis[3].set_xlabel('Time')
axis[3].set_ylabel('Position')
axis[3].legend()

# Adjusting layout and showing plot
plt.tight_layout()
plt.show()