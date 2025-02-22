import serial
import time
import threading
import tkinter as tk
import math

# Configure Serial Port
SERIAL_PORT = "COM14"  # Change accordingly
BAUD_RATE = 115200
BYTE_SIZE = 8
START_BYTE_1 = 0xAB
STOP_BYTE_1 = 0xBA

START_BYTE_2 = 0xAC
STOP_BYTE_2 = 0xCA

# Open Serial Connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

root = tk.Tk()
root.title("Sliders for Pi Values")

# Set window size
root.geometry("400x300")

# Create the first slider (from -pi to pi)
slider1 = tk.Scale(root, from_=-500000, to=500000, orient="horizontal", resolution=0.001, length=300, label="Slider 1")
slider1.set(0)  # Set initial value to 0
slider1.pack(pady=20)

# Label to display the value of slider 1
slider1_value_label = tk.Label(root, text=f"Slider 1: {slider1.get():.4f}")
slider1_value_label.pack()

# Create the second slider (from -pi to pi)
slider2 = tk.Scale(root, from_=-500000, to=500000, orient="horizontal", resolution=0.001, length=300, label="Slider 2")
slider2.set(0)  # Set initial value to 0
slider2.pack(pady=20)

# Label to display the value of slider 2
slider2_value_label = tk.Label(root, text=f"Slider 2: {slider2.get():.4f}")
slider2_value_label.pack()



def update_values():
    
    slider1_value_label.config(text=f"Slider 1: {slider1.get():.4f}")
    
    slider2_value_label.config(text=f"Slider 2: {slider2.get():.4f}")
    
    speedSetpoint1 = int(slider1.get())
    message1 = [
0xA2, 0x00, 0x00, 0x00,
speedSetpoint1 & 0xFF,
(speedSetpoint1 >> 8) & 0xFF,
(speedSetpoint1 >> 16) & 0xFF,
(speedSetpoint1 >> 24) & 0xFF
]
    speedSetpoint2 = int(slider2.get())
    message2 = [
0xA2, 0x00, 0x00, 0x00,
speedSetpoint2 & 0xFF,
(speedSetpoint2 >> 8) & 0xFF,
(speedSetpoint2 >> 16) & 0xFF,
(speedSetpoint2 >> 24) & 0xFF
]
    msg1=[START_BYTE_1] + message1 + [STOP_BYTE_1]
    msg2=[START_BYTE_2] + message2 + [STOP_BYTE_2]
    packet1 = bytes(msg1)
    packet2 = bytes(msg2)
    #print(packet)
    # Send the list as bytes
    ser.write(bytes(packet1))
    ser.write(bytes(packet2))
    
    print(f"Python Data SNT: {' '.join(f'{x:02X}' for x in packet1)}")
    print(f"Python Data SNT: {' '.join(f'{x:02X}' for x in packet2)}")

def reset_sliders():
    slider1.set(0)  # Reset slider1 to 0
    slider2.set(0)  # Reset slider2 to 0
    update_values()


slider1.bind("<Motion>", lambda event: update_values())
slider2.bind("<Motion>", lambda event: update_values())

reset_button = tk.Button(root, text="Reset Sliders", command=reset_sliders)
reset_button.pack(pady=20)



def read_data():
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().strip()
            print("Python Data RCV:", response)

read_thread = threading.Thread(target=read_data, daemon=True)

read_thread.start()

try:
    while True:
        root.mainloop()  # Main thread doing nothing, just waiting for other threads
except KeyboardInterrupt:
    print("Program exited.")
    
ser.close()
