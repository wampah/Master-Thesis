import serial
import time
import threading
import tkinter as tk

# Configure Serial Port
SERIAL_PORT = "COM15"  # Change accordingly
BAUD_RATE = 115200
START_BYTE = 0xAB
STOP_BYTE = 0xBA

# Open Serial Connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# GUI Setup
root = tk.Tk()
root.title("Motor Control")
root.geometry("400x450")

# Default mode
mode = tk.StringVar(value="speed")

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
motor1_brake_label = tk.Label(motor1_frame, text="Brake Status: --")
motor1_brake_label.pack()

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
motor2_brake_label = tk.Label(motor2_frame, text="Brake Status: --")
motor2_brake_label.pack()

# Mode settings
MODE_SETTINGS = {
    "speed": {"min": -500000, "max": 500000, "step": 1000},
    "position": {"min": -36000, "max": 36000, "step": 1},
    "torque": {"min": -100, "max": 100, "step": 1},
}

# Function to Update GUI Labels
def update_motor_data_9c(motor_id, temp, current, speed, angle):
    if motor_id == 1:
        motor1_temp_label.config(text=f"Temp: {temp:.2f}°C")
        motor1_current_label.config(text=f"Current: {current:.2f} A")
        motor1_speed_label.config(text=f"Speed: {speed:.2f}°/s")
        motor1_angle_label.config(text=f"Angle: {angle:.2f}°")

    elif motor_id == 2:
        motor2_temp_label.config(text=f"Temp: {temp:.2f}°C")
        motor2_current_label.config(text=f"Current: {current:.2f} A")
        motor2_speed_label.config(text=f"Speed: {speed:.2f}°/s")
        motor2_angle_label.config(text=f"Angle: {angle:.2f}°")

def update_motor_data_9a(motor_id, voltage, brake):
    if motor_id == 1:
        motor1_voltage_label.config(text=f"Voltage: {voltage:.2f}V")
        motor1_brake_label.config(text=f"Brake Status: {brake:.0f}")
    elif motor_id == 2:
        motor2_voltage_label.config(text=f"Voltage: {voltage:.2f}V")
        motor2_brake_label.config(text=f"Brake Status: {brake:.0f}")

# Create mode selection (radio buttons)
def change_mode():
    """Update sliders and reset values when mode changes."""
    settings = MODE_SETTINGS[mode.get()]
    slider1.config(from_=settings["min"], to=settings["max"], resolution=settings["step"])
    slider2.config(from_=settings["min"], to=settings["max"], resolution=settings["step"])
    reset_sliders()

mode_frame = tk.Frame(root)
mode_frame.pack(pady=10)
tk.Radiobutton(mode_frame, text="Speed", variable=mode, value="speed", command=change_mode).pack(side="left")
tk.Radiobutton(mode_frame, text="Position", variable=mode, value="position", command=change_mode).pack(side="left")
tk.Radiobutton(mode_frame, text="Torque", variable=mode, value="torque", command=change_mode).pack(side="left")

# Create sliders
slider1 = tk.Scale(root, from_=-500000, to=500000, orient="horizontal", resolution=1000, length=300, label="Slider 1")
slider1.pack(pady=10)
slider1_value_label = tk.Label(root, text=f"Slider 1: {slider1.get()}")
slider1_value_label.pack()

slider2 = tk.Scale(root, from_=-500000, to=500000, orient="horizontal", resolution=1000, length=300, label="Slider 2")
slider2.pack(pady=10)
slider2_value_label = tk.Label(root, text=f"Slider 2: {slider2.get()}")
slider2_value_label.pack()

def update_values():
    """Update slider labels and send data via serial."""
    slider1_value_label.config(text=f"Slider 1: {slider1.get()}")
    slider2_value_label.config(text=f"Slider 2: {slider2.get()}")

    setpoint1 = int(slider1.get())
    setpoint2 = int(slider2.get())

    # Construct message based on mode
    if mode.get() == "speed":
        message1 = [
            0xA2, 0x00, 0x00, 0x00,
            setpoint1 & 0xFF,
            (setpoint1 >> 8) & 0xFF,
            (setpoint1 >> 16) & 0xFF,
            (setpoint1 >> 24) & 0xFF
        ]
        message2 = [
            0xA2, 0x00, 0x00, 0x00,
            setpoint2 & 0xFF,
            (setpoint2 >> 8) & 0xFF,
            (setpoint2 >> 16) & 0xFF,
            (setpoint2 >> 24) & 0xFF
        ]
    elif mode.get() == "position":
        max_speed = 250
        message1 = [
            0xA4, 
            0x00, 
            max_speed & 0xFF,
            (max_speed >> 8) & 0xFF,
            setpoint1 & 0xFF,
            (setpoint1 >> 8) & 0xFF,
            (setpoint1 >> 16) & 0xFF,
            (setpoint1 >> 24) & 0xFF
        ]
        message2 = [
            0xA4, 
            0x00, 
            max_speed & 0xFF,
            (max_speed >> 8) & 0xFF,
            setpoint2 & 0xFF,
            (setpoint2 >> 8) & 0xFF,
            (setpoint2 >> 16) & 0xFF,
            (setpoint2 >> 24) & 0xFF
        ]
    else:  # torque mode
        message1 = [
            0xA1, 0x00, 0x00, 0x00,
            setpoint1 & 0xFF,
            (setpoint1 >> 8) & 0xFF,
            0x00,
            0x00
        ]
        message2 = [
            0xA1, 0x00, 0x00, 0x00,
            setpoint2 & 0xFF,
            (setpoint2 >> 8) & 0xFF,
            0x00,
            0x00
        ]

    # Encode and send message
    packet = bytes([START_BYTE] + message1 + message2 + [STOP_BYTE])
    ser.write(packet)

def reset_sliders():
    """Reset sliders to 0."""
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

# Serial Reading Thread
def read_data():
    """Reads and processes serial messages."""
    
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue

                parts = line.split()  # Split by spaces
                if "Data:" in parts:
                    data_index = parts.index("Data:") + 1
                    id_index=parts.index("ID:") + 1
                    if len(parts) >= data_index + 8:
                        data_bytes = bytes(int(h, 16) for h in parts[data_index:data_index + 8]) 
                        
                        if parts[id_index]=="0x241":
                            motor_id = 1
                        elif parts[id_index]=="0x242":
                            motor_id = 2
                        else:
                            raise Exception("Motor ID error")
                        
                        if data_bytes[0] == 0x9C:
                            
                            process_9c_message(data_bytes, motor_id)

                        elif data_bytes[0] == 0x9a:
                            process_9a_message(data_bytes, motor_id)

                        else:
                            print(line)
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            ser.close()

# Process 0x9C Message
def process_9c_message(data_bytes, motor_id):
    """Processes 0x9C motor response message and updates GUI labels."""
    if len(data_bytes) != 8:
        print("Invalid 0x9C message:", data_bytes.hex())
        return

    # Extract values
    temperature = int.from_bytes(data_bytes[1:2], byteorder="little", signed=True)
    current = int.from_bytes(data_bytes[2:4], byteorder="little", signed=True) * 0.01  # 0.01A/LSB
    speed = int.from_bytes(data_bytes[4:6], byteorder="little", signed=True)  # 1dps/LSB
    angle = int.from_bytes(data_bytes[6:8], byteorder="little", signed=True)  # 1°/LSB

    # Update GUI
    root.after(0, update_motor_data_9c, motor_id, temperature, current, speed, angle)

def process_9a_message(data_bytes, motor_id):
    """Processes 0x9A motor response message and updates GUI labels."""
    if len(data_bytes) != 8:
        print("Invalid 0x9A message:", data_bytes.hex())
        return

    brake = int.from_bytes(data_bytes[3:4], byteorder="little", signed=True)   # 0.01A/LSB
    voltage = int.from_bytes(data_bytes[4:6], byteorder="little", signed=True)*0.1  # 1dps/LSB

    # Update GUI
    root.after(0, update_motor_data_9a, motor_id, voltage,brake)

# Periodic message sending thread
def send_periodic_message():
    """Send a periodic message every few seconds."""
    while True:
        time.sleep(0.1)  # Adjust time interval as needed (e.g., every 2 seconds)

        message1 = [
        0x9c, 
        0x00, 
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
        ]
        message2 = [
            0x9c, 
            0x00, 
            0x00, 
            0x00, 
            0x00, 
            0x00, 
            0x00, 
            0x00
        ]
        
        # Example message (heartbeat or status request)
        packet = bytes([START_BYTE] + message1 + message2 + [STOP_BYTE])
        ser.write(packet)
        message1 = [
        0x9a, 
        0x00, 
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
        ]
        message2 = [
            0x9a, 
            0x00, 
            0x00, 
            0x00, 
            0x00, 
            0x00, 
            0x00, 
            0x00
        ]
        
        # Example message (heartbeat or status request)
        packet = bytes([START_BYTE] + message1 + message2 + [STOP_BYTE])
        ser.write(packet)

# Start serial reading thread
read_thread = threading.Thread(target=read_data, daemon=True)
read_thread.start()

# Start periodic sending thread
send_thread = threading.Thread(target=send_periodic_message, daemon=True)
send_thread.start()

# Start GUI
root.mainloop()
ser.close()
