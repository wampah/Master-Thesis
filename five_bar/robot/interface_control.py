import tkinter as tk
import threading
import time
from five_bar.five_bar import five_bar

# Initialize the robot
robot = five_bar(SERIAL_PORT="COM3", max_speed=100)
robot.set_control_mode("position",135,-135)  # Default mode
robot.start()

# GUI Setup
root = tk.Tk()
root.title("Motor Control")
root.geometry("400x450")

# Default mode
mode = tk.StringVar(value="position")

# Frame for motor data
data_frame = tk.Frame(root)
data_frame.pack()

# Column 1 (Motor 1)
motor1_frame = tk.Frame(data_frame)
motor1_frame.grid(row=0, column=0, padx=10, pady=10)
tk.Label(motor1_frame, text="Motor 1", font=("Arial", 12, "bold")).pack()
motor1_temp_label = tk.Label(motor1_frame, text="Temp: --°C")
motor1_temp_label.pack()
motor1_current_label = tk.Label(motor1_frame, text="Current: -- A")
motor1_current_label.pack()
motor1_voltage_label = tk.Label(motor1_frame, text="Voltage: -- V")
motor1_voltage_label.pack()
motor1_speed_label = tk.Label(motor1_frame, text="Speed: --°/s")
motor1_speed_label.pack()
motor1_angle_label = tk.Label(motor1_frame, text="Angle: --°")
motor1_angle_label.pack()


# Column 2 (Motor 2)
motor2_frame = tk.Frame(data_frame)
motor2_frame.grid(row=0, column=1, padx=10, pady=10)
tk.Label(motor2_frame, text="Motor 2", font=("Arial", 12, "bold")).pack()
motor2_temp_label = tk.Label(motor2_frame, text="Temp: --°C")
motor2_temp_label.pack()
motor2_current_label = tk.Label(motor2_frame, text="Current: -- A")
motor2_current_label.pack()
motor2_voltage_label = tk.Label(motor2_frame, text="Voltage: -- V")
motor2_voltage_label.pack()
motor2_speed_label = tk.Label(motor2_frame, text="Speed: --°/s")
motor2_speed_label.pack()
motor2_angle_label = tk.Label(motor2_frame, text="Angle: --°")
motor2_angle_label.pack()


# Mode settings
MODE_SETTINGS = {
    "speed": {"min": -100, "max": 100, "step": 1},
    "position": {"min": -360, "max": 360, "step": 1},
    "torque": {"min": -1, "max": 1, "step": 0.01},
}

# Function to Update GUI Labels
def update_motor_data(motor_id, data):
    if motor_id == 1:
        motor1_temp_label.config(text=f"Temp: {data['temperature']:.2f}°C")
        motor1_current_label.config(text=f"Current: {data['current']:.2f} A")
        motor1_speed_label.config(text=f"Speed: {data['speed']:.2f}°/s")
        motor1_angle_label.config(text=f"Angle: {data['angle']:.2f}°")
        motor1_voltage_label.config(text=f"Voltage: {data['voltage']:.2f}V")
    elif motor_id == 2:
        motor2_temp_label.config(text=f"Temp: {data['temperature']:.2f}°C")
        motor2_current_label.config(text=f"Current: {data['current']:.2f} A")
        motor2_speed_label.config(text=f"Speed: {data['speed']:.2f}°/s")
        motor2_angle_label.config(text=f"Angle: {data['angle']:.2f}°")
        motor2_voltage_label.config(text=f"Voltage: {data['voltage']:.2f}V")

# Create mode selection (radio buttons)
def change_mode():
    """Update sliders and reset values when mode changes."""
    settings = MODE_SETTINGS[mode.get()]
    slider1.config(from_=settings["min"], to=settings["max"], resolution=settings["step"])
    slider2.config(from_=settings["min"], to=settings["max"], resolution=settings["step"])
    if mode.get()== "position":
        robot.set_control_mode(mode.get(),robot.get_motors_data()["angle"][0],robot.get_motors_data()["angle"][1])
    else:
        robot.set_control_mode(mode.get(),0,0)
    reset_sliders()

mode_frame = tk.Frame(root)
mode_frame.pack(pady=10)
tk.Radiobutton(mode_frame, text="Speed", variable=mode, value="speed", command=change_mode).pack(side="left")
tk.Radiobutton(mode_frame, text="Position", variable=mode, value="position", command=change_mode).pack(side="left")
tk.Radiobutton(mode_frame, text="Torque", variable=mode, value="torque", command=change_mode).pack(side="left")

# Create sliders
slider1 = tk.Scale(root, from_=MODE_SETTINGS[mode.get()]["min"], to=MODE_SETTINGS[mode.get()]["max"], orient="horizontal", resolution=1, length=300, label="Slider 1")
slider1.pack(pady=10)
slider1_value_label = tk.Label(root, text=f"Slider 1: {slider1.get()}")
slider1_value_label.pack()
slider1.set(robot.get_motors_data()["angle"][0])

slider2 = tk.Scale(root, from_=MODE_SETTINGS[mode.get()]["min"], to=MODE_SETTINGS[mode.get()]["max"], orient="horizontal", resolution=1, length=300, label="Slider 2")
slider2.pack(pady=10)
slider2_value_label = tk.Label(root, text=f"Slider 2: {slider2.get()}")
slider2_value_label.pack()
slider2.set(robot.get_motors_data()["angle"][1])

def update_values():
    """Update slider labels and send data to robot."""
    slider1_value_label.config(text=f"Slider 1: {slider1.get()}")
    slider2_value_label.config(text=f"Slider 2: {slider2.get()}")

    setpoint1 = slider1.get()
    setpoint2 = slider2.get()
    
    robot.set_target(setpoint1, setpoint2)

def reset_sliders():
    """Reset sliders to 0."""
    
    if mode.get()== "position":
        slider1.set(robot.get_motors_data()["angle"][0])
        slider2.set(robot.get_motors_data()["angle"][1])
    else:
        slider1.set(0)
        slider2.set(0)
    
    update_values()

# Bind events for real-time updates
slider1.bind("<B1-Motion>", lambda event: update_values())  
slider1.bind("<ButtonRelease-1>", lambda event: update_values())  
slider2.bind("<B1-Motion>", lambda event: update_values())  
slider2.bind("<ButtonRelease-1>", lambda event: update_values())

reset_button = tk.Button(root, text="Reset Sliders", command=reset_sliders)
reset_button.pack(pady=10)

# Data polling thread
def poll_motor_data():
    """Continuously polls motor data and updates the GUI."""
    while True:
        try:
            # Get data for both motors
            data = robot.get_motors_data()
            
            # Prepare motor 1 data
            motor1_data = {
                'voltage': data['voltage'][0],
                'current': data['current'][0],
                'speed': data['speed'][0],
                'angle': data['angle'][0],
                'temperature': data['temperature'][0],
                'power': data['power'][0],
                'torque': data['torque'][0]
            }
            
            # Prepare motor 2 data
            motor2_data = {
                'voltage': data['voltage'][1],
                'current': data['current'][1],
                'speed': data['speed'][1],
                'angle': data['angle'][1],
                'temperature': data['temperature'][1],
                'power': data['power'][1],
                'torque': data['torque'][1]
            }
            
            # Update GUI
            root.after(0, update_motor_data, 1, motor1_data)
            root.after(0, update_motor_data, 2, motor2_data)
            
            time.sleep(0.1)  # Polling interval
        except Exception as e:
            print(f"Error polling motor data: {e}")
            time.sleep(1)

# Start data polling thread
poll_thread = threading.Thread(target=poll_motor_data, daemon=True)
poll_thread.start()

# Start GUI
root.mainloop()
