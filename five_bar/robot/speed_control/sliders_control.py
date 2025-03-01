import serial
import time
import threading
import tkinter as tk
import argparse

# Configure Serial Port
SERIAL_PORT = "COM15"  # Change accordingly
BAUD_RATE = 115200
START_BYTE = 0xAB
STOP_BYTE = 0xBA

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Run serial control with different modes.")
parser.add_argument("mode", choices=["speed", "position", "torque"], help="Control mode: speed, position, or torque")
args = parser.parse_args()

# Open Serial Connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# GUI Setup
root = tk.Tk()
root.title(f"{args.mode.capitalize()} Control")
root.geometry("400x300")

# Set slider ranges based on mode
if args.mode == "speed":
    slider_min, slider_max, resolution = -500000, 500000, 1000
elif args.mode == "position":
    slider_min, slider_max, resolution = -36000, 36000, 1
else:  # Torque mode
    slider_min, slider_max, resolution = -100, 100, 1

# Create sliders
slider1 = tk.Scale(root, from_=slider_min, to=slider_max, orient="horizontal", resolution=resolution, length=300, label="Slider 1")
slider1.set(0)
slider1.pack(pady=10)
slider1_value_label = tk.Label(root, text=f"Slider 1: {slider1.get()}")
slider1_value_label.pack()

slider2 = tk.Scale(root, from_=slider_min, to=slider_max, orient="horizontal", resolution=resolution, length=300, label="Slider 2")
slider2.set(0)
slider2.pack(pady=10)
slider2_value_label = tk.Label(root, text=f"Slider 2: {slider2.get()}")
slider2_value_label.pack()

def update_values():
    """Update slider labels and send data via serial."""
    slider1_value_label.config(text=f"Slider 1: {slider1.get()}")
    slider2_value_label.config(text=f"Slider 2: {slider2.get()}")

    controlSetpoint1 = int(slider1.get())
    controlSetpoint2 = int(slider2.get())

    # Construct message based on mode
    if args.mode == "speed":
        message1 = [
            0xA2, 0x00, 0x00, 0x00,
            controlSetpoint1 & 0xFF,
            (controlSetpoint1 >> 8) & 0xFF,
            (controlSetpoint1 >> 16) & 0xFF,
            (controlSetpoint1 >> 24) & 0xFF
        ]
        message2 = [
            0xA2, 0x00, 0x00, 0x00,
            controlSetpoint2 & 0xFF,
            (controlSetpoint2 >> 8) & 0xFF,
            (controlSetpoint2 >> 16) & 0xFF,
            (controlSetpoint2 >> 24) & 0xFF
        ]
    
    elif args.mode == "position":
        max_speed=20000
        message1 = [
            0xA4, 
            0x00, 
            max_speed & 0xFF,
            (max_speed >> 8) & 0xFF,
            controlSetpoint1 & 0xFF,
            (controlSetpoint1 >> 8) & 0xFF,
            (controlSetpoint1 >> 16) & 0xFF,
            (controlSetpoint1 >> 24) & 0xFF
        ]
        message2 = [
            0xA4, 
            0x00, 
            max_speed & 0xFF,
            (max_speed >> 8) & 0xFF,
            controlSetpoint2 & 0xFF,
            (controlSetpoint2 >> 8) & 0xFF,
            (controlSetpoint2 >> 16) & 0xFF,
            (controlSetpoint2 >> 24) & 0xFF
        ]
    
    else:  # Torque mode
        message1 = [
            0xA1, 0x00, 0x00, 0x00,
            controlSetpoint1 & 0xFF,
            (controlSetpoint1 >> 8) & 0xFF,
            0x00,
            0x00
        ]
        message2 = [
            0xA1, 0x00, 0x00, 0x00,
            controlSetpoint2 & 0xFF,
            (controlSetpoint2 >> 8) & 0xFF,
            0x00,
            0x00
        ]

    # Encode message
    packet = bytes([START_BYTE] + message1 + message2 + [STOP_BYTE])
    
    # Send data
    ser.write(packet)

def reset_sliders():
    """Reset sliders to 0."""
    slider1.set(0)
    slider2.set(0)
    update_values()

# Bind events for real-time updates
slider1.bind("<B1-Motion>", lambda event: update_values())  # While dragging
slider1.bind("<ButtonRelease-1>", lambda event: update_values())  # On release
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

# Start GUI main loop
root.mainloop()

# Close serial port when GUI exits
ser.close()
