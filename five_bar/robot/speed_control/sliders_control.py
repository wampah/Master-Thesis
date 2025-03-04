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
root.geometry("400x400")

# Default mode
mode = tk.StringVar(value="speed")

# Mode settings
MODE_SETTINGS = {
    "speed": {"min": -500000, "max": 500000, "step": 1000},
    "position": {"min": -36000, "max": 36000, "step": 1},
    "torque": {"min": -100, "max": 100, "step": 1},
}

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
        max_speed = 5000000
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

def read_data():
    """Continuously read data from serial port."""
    buf = b""
    while True:
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            buf += response
            if b'\n' in buf:
                lines = buf.split(b'\n')
                for line in lines[:-1]:  # Process complete lines
                    print("Python Data RCV:", line.decode(errors='ignore'))
                buf = lines[-1]  # Keep incomplete data

# Start serial reading thread
read_thread = threading.Thread(target=read_data, daemon=True)
read_thread.start()

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


# Start periodic sending thread
send_thread = threading.Thread(target=send_periodic_message, daemon=True)
send_thread.start()

# Start GUI
root.mainloop()
ser.close()
