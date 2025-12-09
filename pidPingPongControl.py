import tkinter as tk
from tkinter import ttk
import threading
import time
import serial
import cv2
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from collections import deque

# ==== Servo & Serial Setup ====
SERIAL_PORT = 'COM4'   # Change this as needed
BAUD_RATE = 9600

try:
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
except serial.SerialException:
    arduino = None
    print(f"Could not open serial port {SERIAL_PORT}")

# ==== PID Controller ====
class PIDController:
    def __init__(self, kp=0.5, ki=0.0, kd=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def update_constants(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

# ==== Angle Mapping ====
def map_pid_to_servo_angle(pid_output):
    max_tilt = 35
    tilt_angle = max(-max_tilt, min(max_tilt, pid_output))
    return int(np.interp(tilt_angle, [-35, 35], [0, 180]))

def send_angle_to_servo(angle):
    if arduino:
        arduino.write(f"{angle}\n".encode())

# ==== Main Application ====
class BallBalancerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Ball Balancer with PID Control")

        # PID Controls
        self.kp_var = tk.DoubleVar(value=0.5)
        self.ki_var = tk.DoubleVar(value=0.0)
        self.kd_var = tk.DoubleVar(value=0.1)

        self.create_widgets()

        # PID and camera setup
        self.pid = PIDController(self.kp_var.get(), self.ki_var.get(), self.kd_var.get())
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise Exception("Webcam not accessible")
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.target_x = self.frame_width // 2
        self.prev_time = time.time()
        self.prev_pos = None

        # Data buffers
        self.x_data = deque(maxlen=100)
        self.v_data = deque(maxlen=100)
        self.t_data = deque(maxlen=100)
        self.start_time = time.time()

        # Start processing
        self.running = True
        self.update_pid()
        self.run()

    def create_widgets(self):
        control_frame = tk.Frame(self.root)
        control_frame.pack(side=tk.TOP, fill=tk.X)

        tk.Label(control_frame, text="Kp").pack(side=tk.LEFT)
        tk.Entry(control_frame, textvariable=self.kp_var, width=5).pack(side=tk.LEFT)
        tk.Label(control_frame, text="Ki").pack(side=tk.LEFT)
        tk.Entry(control_frame, textvariable=self.ki_var, width=5).pack(side=tk.LEFT)
        tk.Label(control_frame, text="Kd").pack(side=tk.LEFT)
        tk.Entry(control_frame, textvariable=self.kd_var, width=5).pack(side=tk.LEFT)
        ttk.Button(control_frame, text="Update PID", command=self.update_pid).pack(side=tk.LEFT)

        # Canvas for video
        self.video_label = tk.Label(self.root)
        self.video_label.pack()

        # Matplotlib figure for plotting
        fig, self.ax = plt.subplots(2, 1, figsize=(5, 4), dpi=100)
        self.line_pos, = self.ax[0].plot([], [], label="Position (px)")
        self.line_vel, = self.ax[1].plot([], [], label="Velocity (px/s)")
        self.ax[0].set_ylim(0, self.frame_width)
        self.ax[1].set_ylim(-100, 100)
        self.ax[0].set_ylabel("X Position")
        self.ax[1].set_ylabel("X Velocity")
        self.ax[1].set_xlabel("Time (s)")
        self.ax[0].legend()
        self.ax[1].legend()

        self.canvas = FigureCanvasTkAgg(fig, master=self.root)
        self.canvas.get_tk_widget().pack()

    def update_pid(self):
        self.pid.update_constants(self.kp_var.get(), self.ki_var.get(), self.kd_var.get())

    def run(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        _, thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ball_x = None
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 50:
                x, y, w, h = cv2.boundingRect(largest)
                ball_x = x + w // 2
                cv2.circle(frame, (ball_x, y + h // 2), w // 2, (0, 0, 255), 2)
                cv2.line(frame, (ball_x, 0), (ball_x, frame.shape[0]), (0, 255, 0), 1)

        # Draw target center
        cv2.line(frame, (self.target_x, 0), (self.target_x, frame.shape[0]), (255, 0, 0), 2)

        # PID logic
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        if ball_x is not None:
            error = self.target_x - ball_x
            pid_output = self.pid.compute(error, dt)
            servo_angle = map_pid_to_servo_angle(pid_output)
            send_angle_to_servo(servo_angle)

            # Velocity estimation
            velocity = 0
            if self.prev_pos is not None:
                velocity = (ball_x - self.prev_pos) / dt
            self.prev_pos = ball_x

            # Append data for graph
            t = now - self.start_time
            self.x_data.append(ball_x)
            self.v_data.append(velocity)
            self.t_data.append(t)

            # Update plots
            self.line_pos.set_data(self.t_data, self.x_data)
            self.line_vel.set_data(self.t_data, self.v_data)
            for ax in self.ax:
                ax.relim()
                ax.autoscale_view()

            self.canvas.draw()

        # Convert frame to ImageTk and update video feed
        cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = cv2.resize(cv2image, (640, 480))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)
        img = np.array(img)
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        photo = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        photo = cv2.resize(photo, (480, 360))
        photo = cv2.cvtColor(photo, cv2.COLOR_RGB2BGR)
        photo = cv2.cvtColor(photo, cv2.COLOR_BGR2RGBA)
        self.video_label.imgtk = tk.PhotoImage(master=self.video_label, data=photo.tobytes())
        self.video_label.configure(image=self.video_label.imgtk)

        if self.running:
            self.root.after(10, self.run)

    def close(self):
        self.running = False
        self.cap.release()
        if arduino:
            arduino.close()
        self.root.destroy()

# ==== Start GUI ====
root = tk.Tk()
app = BallBalancerApp(root)
root.protocol("WM_DELETE_WINDOW", app.close)
root.mainloop()

