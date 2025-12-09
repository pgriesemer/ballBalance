import tkinter as tk
from tkinter import messagebox
import serial
import time

SERIAL_PORT = 'COM3'   # Change this
BAUD_RATE = 19200
DEBOUNCE_TIME = 50

try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
except serial.SerialException:
    arduino = None
    print(f"Could not open serial port {SERIAL_PORT}")

# Debounce handler
debounce_id = None

def debounce_send(value):
    global debounce_id
    if debounce_id:
        root.after_cancel(debounce_id)
    debounce_id = root.after(DEBOUNCE_TIME, lambda: send_angle(value))  # 150 ms debounce

def send_angle(value):
    angle = int(value)
    if arduino:
        arduino.write(f"{angle}\n".encode())
        status_label.config(text=f"Sent angle: {angle}")
    else:
        messagebox.showerror("Serial Error", "Serial connection not available.")

# GUI setup
root = tk.Tk()
root.title("Servo Controller (Debounced)")

tk.Label(root, text="Servo Angle (0–180)").pack(pady=10)

angle_slider = tk.Scale(root, from_=0, to=180, orient=tk.HORIZONTAL, length=300,
                        command=debounce_send)
angle_slider.pack(pady=10)

status_label = tk.Label(root, text="Move the slider to send angle")
status_label.pack(pady=10)

root.mainloop()

if arduino:
    arduino.close()


