import tkinter as tk
from tkinter import ttk
import time
import serial
import cv2
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from collections import deque
import platform
import subprocess

# ==============================================================================
# --- Constants & Global Setup ---
# ==============================================================================
SERIAL_PORT = 'COM4'   # <--- IMPORTANT: Change this to your Arduino's port
BAUD_RATE = 19200
ARDUINO = None
RESIZE_WIDTH = 320     # --- OPTIMIZATION: Process smaller frames for performance
TARGET_FREQ_HZ = 10   # --- Set a more realistic control loop frequency
TARGET_LOOP_TIME_S = 1.0 / TARGET_FREQ_HZ

# --- NEW: Conversion factor for real-world units ---
PIXELS_PER_FOOT = 300.0
METERS_PER_FOOT = 0.3048
PIXELS_PER_METER = PIXELS_PER_FOOT / METERS_PER_FOOT

# Try to connect to the Arduino
try:
    ARDUINO = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1, write_timeout=0.1)
    time.sleep(2)  # Wait for the connection to establish
    print(f"Successfully connected to Arduino on {SERIAL_PORT}")
except serial.SerialException:
    print(f"--- WARNING: Could not open serial port {SERIAL_PORT}. ---")
    print("--- The application will run in simulation mode without servo output. ---")

# ==============================================================================
# --- PID Controller Class ---
# ==============================================================================
class PIDController:
    """A simple PID controller with a low-pass filter on the derivative term."""
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, d_filter_alpha=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0
        self.prev_error = 0
        self.last_compute_time = time.time()
        self.d_filter_alpha = d_filter_alpha
        self.last_derivative = 0.0
        self.raw_derivative = 0.0

    def compute(self, error):
        """Computes the PID output."""
        now = time.time()
        dt = now - self.last_compute_time
        if dt <= 0:
            return 0
            
        p_term = self.kp * error
        self.integral += error * dt
        i_term = self.ki * self.integral

        self.raw_derivative = (error - self.prev_error) / dt
        
        filtered_derivative = (self.d_filter_alpha * self.last_derivative) + \
                              (1 - self.d_filter_alpha) * self.raw_derivative
        self.last_derivative = filtered_derivative
        d_term = self.kd * filtered_derivative
        
        self.prev_error = error
        self.last_compute_time = now
        
        return p_term + i_term + d_term

    def update_constants(self, kp, ki, kd, d_filter_alpha):
        """Updates PID gains and resets the controller state."""
        self.kp, self.ki, self.kd = kp, ki, kd
        self.d_filter_alpha = d_filter_alpha
        self.reset_integral()
        self.prev_error = 0
        print(f"PID constants updated: Kp={kp}, Ki={ki}, Kd={kd}, D-Alpha={d_filter_alpha}")

    def reset_integral(self):
        """Resets the integral (accumulated error) term."""
        self.integral = 0
        print("PID Integral term has been reset to zero.")


# ==============================================================================
# --- Helper Functions ---
# ==============================================================================
def map_pid_to_servo_angle(pid_output, max_tilt=35):
    """Maps the PID output to a servo angle (0-180), clamping the tilt."""
    tilt_angle = max(-max_tilt, min(max_tilt, pid_output))
    return int(np.interp(tilt_angle, [-max_tilt, max_tilt], [145, 35]))

def send_angle_to_servo(angle):
    """Sends the target angle to the Arduino over serial."""
    if ARDUINO and ARDUINO.is_open:
        try:
            command = f"{int(angle)}\n".encode()
            ARDUINO.write(command)
        except serial.SerialTimeoutException:
            print("--- WARNING: Serial write timeout. Arduino buffer may be full. ---")
        except serial.SerialException as e:
            print(f"Error writing to serial port: {e}")

def select_camera_index(parent_root):
    """Creates a dialog to let the user select a webcam."""
    print(f"DEBUG [{time.perf_counter():.2f}s]: Starting camera search...")
    available = []
    for i in range(5):
        probe_start_time = time.perf_counter()
        print(f"DEBUG [{probe_start_time:.2f}s]: Probing camera index {i}...")
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
        if cap is not None and cap.isOpened():
            available.append(i)
            cap.release()
        probe_end_time = time.perf_counter()
        print(f"DEBUG [{probe_end_time:.2f}s]: Finished probing index {i}. Took {probe_end_time - probe_start_time:.2f}s.")


    if not available:
        print("ERROR: No webcams found!")
        return None
    
    print(f"DEBUG [{time.perf_counter():.2f}s]: Camera search finished. Found: {available}.")
    print(f"DEBUG [{time.perf_counter():.2f}s]: Displaying camera selection window.")
        
    cam_win = tk.Toplevel(parent_root)
    cam_win.title("Select Webcam")
    selected_index = tk.IntVar(value=available[0])
    
    tk.Label(cam_win, text="Select webcam index:").pack(pady=5, padx=10)
    dropdown = ttk.Combobox(cam_win, values=available, textvariable=selected_index, state='readonly')
    dropdown.pack(pady=5, padx=10)
    
    def confirm():
        cam_win.destroy()
        
    tk.Button(cam_win, text="Confirm", command=confirm).pack(pady=10)
    parent_root.wait_window(cam_win)
    
    print(f"DEBUG [{time.perf_counter():.2f}s]: Camera selected.")
    return selected_index.get()


# ==============================================================================
# --- Main Application Class ---
# ==============================================================================
class BallBalancerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Ball Balancer Controller")
        self.root.withdraw()

        self.running = False
        self.pid_enabled = False
        self.start_time = time.time()
        self.count = 0 
        self.ball_found = False
        
        DATA_POINTS = 1000
        self.t_data = deque(maxlen=DATA_POINTS)
        self.x_data = deque(maxlen=DATA_POINTS)
        self.v_data = deque(maxlen=DATA_POINTS)
        self.v_filt_data = deque(maxlen=DATA_POINTS) 
        self.i_data = deque(maxlen=DATA_POINTS)

        self.kp_var = tk.DoubleVar(value=0.0)
        self.ki_var = tk.DoubleVar(value=0.0)
        self.kd_var = tk.DoubleVar(value=0.0)
        self.d_filter_alpha_var = tk.DoubleVar(value=0.6)
        self.target_offset_m_var = tk.DoubleVar(value=0.0)
        # --- NEW: Internal state variable for the active target offset ---
        self.active_target_offset_m = 0.0
        
        self.show_video_var = tk.BooleanVar(value=False)
        self.show_mask_var = tk.BooleanVar(value=False)
        
        self.h_min = tk.IntVar(value=0)
        self.s_min = tk.IntVar(value=0)
        self.v_min = tk.IntVar(value=200)
        self.h_max = tk.IntVar(value=180)
        self.s_max = tk.IntVar(value=30)
        self.v_max = tk.IntVar(value=255)
        
        print(f"DEBUG [{time.perf_counter():.2f}s]: Initializing application components...")
        self.pid = PIDController()
        
        print(f"DEBUG [{time.perf_counter():.2f}s]: Creating control window...")
        self.create_control_window()
        print(f"DEBUG [{time.perf_counter():.2f}s]: Control window created.")
        
        cam_index = select_camera_index(self.root)
        if cam_index is None:
            self.root.destroy()
            return
            
        self.cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            print(f"ERROR: Cannot open camera index {cam_index}")
            self.root.destroy()
            return
            
        print(f"DEBUG [{time.perf_counter():.2f}s]: Camera capture opened successfully.")

        self.center_x_m = (RESIZE_WIDTH / 2.0) / PIXELS_PER_METER

        print(f"DEBUG [{time.perf_counter():.2f}s]: Creating display and plot windows...")
        self.create_display_windows()
        self.create_plot_window()
        print(f"DEBUG [{time.perf_counter():.2f}s]: All windows created.")

        self.update_pid_constants()
        send_angle_to_servo(90)
        self.running = True
        
        print(f"DEBUG [{time.perf_counter():.2f}s]: Starting main loop.")
        self.run_main_loop()

    def create_control_window(self):
        """Creates the main window for all user controls."""
        self.control_window = tk.Toplevel(self.root)
        self.control_window.title("Controls")
        self.control_window.protocol("WM_DELETE_WINDOW", self.close)

        pid_frame = ttk.LabelFrame(self.control_window, text="PID Gains & Filter")
        pid_frame.pack(padx=10, pady=5, fill=tk.X)
        
        gains_frame = tk.Frame(pid_frame)
        gains_frame.pack(fill=tk.X, pady=2)
        for var, label in zip([self.kp_var, self.ki_var, self.kd_var], ["Kp", "Ki", "Kd"]):
            ttk.Label(gains_frame, text=label, width=4).pack(side=tk.LEFT)
            ttk.Entry(gains_frame, textvariable=var, width=8).pack(side=tk.LEFT, padx=2)
        
        filter_frame = tk.Frame(pid_frame)
        filter_frame.pack(fill=tk.X, pady=2)
        ttk.Label(filter_frame, text="D-Filter Alpha").pack(side=tk.LEFT)
        ttk.Entry(filter_frame, textvariable=self.d_filter_alpha_var, width=8).pack(side=tk.LEFT, padx=2)

        target_frame = tk.Frame(pid_frame)
        target_frame.pack(fill=tk.X, pady=2)
        ttk.Label(target_frame, text="Target Offset (m)").pack(side=tk.LEFT)
        ttk.Entry(target_frame, textvariable=self.target_offset_m_var, width=8).pack(side=tk.LEFT, padx=2)

        button_frame = tk.Frame(pid_frame)
        button_frame.pack(pady=5)
        ttk.Button(button_frame, text="Update PID", command=self.update_pid_constants).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Reset Integral", command=self.pid.reset_integral).pack(side=tk.LEFT, padx=5)

        display_frame = ttk.LabelFrame(self.control_window, text="Display Options")
        display_frame.pack(padx=10, pady=5, fill=tk.X)
        ttk.Checkbutton(display_frame, text="Show Camera Feed", variable=self.show_video_var, command=self._toggle_window_visibility).pack(side=tk.LEFT, padx=5)
        ttk.Checkbutton(display_frame, text="Show Color Mask", variable=self.show_mask_var, command=self._toggle_window_visibility).pack(side=tk.LEFT, padx=5)

        hsv_frame = ttk.LabelFrame(self.control_window, text="HSV Color Threshold")
        hsv_frame.pack(padx=10, pady=5, fill=tk.X)
        sliders1 = tk.Frame(hsv_frame)
        sliders1.pack(fill=tk.X)
        sliders2 = tk.Frame(hsv_frame)
        sliders2.pack(fill=tk.X)
        
        for var, label, parent in zip(
            [self.h_min, self.s_min, self.v_min, self.h_max, self.s_max, self.v_max],
            ["H min", "S min", "V min", "H max", "S max", "V max"],
            [sliders1, sliders1, sliders1, sliders2, sliders2, sliders2]):
            frame = tk.Frame(parent)
            frame.pack(fill=tk.X, expand=True, side=tk.LEFT)
            ttk.Label(frame, text=label).pack()
            tk.Scale(frame, from_=0, to=255, orient=tk.HORIZONTAL, variable=var).pack(fill=tk.X, expand=True)

    def _toggle_window_visibility(self):
        """Shows or hides windows based on the checkbox state."""
        if self.show_video_var.get():
            self.video_window.deiconify()
        else:
            self.video_window.withdraw()
        
        if self.show_mask_var.get():
            self.mask_window.deiconify()
        else:
            self.mask_window.withdraw()

    def create_display_windows(self):
        self.video_window = tk.Toplevel(self.root)
        self.video_window.title("Live Camera Feed")
        self.video_label = tk.Label(self.video_window)
        self.video_label.pack()

        self.mask_window = tk.Toplevel(self.root)
        self.mask_window.title("Color Mask View")
        self.mask_label = tk.Label(self.mask_window)
        self.mask_label.pack()

        self._toggle_window_visibility()


    def create_plot_window(self):
        self.plot_window = tk.Toplevel(self.root)
        self.plot_window.title("Real-time Performance")
        fig, (self.ax_pos, self.ax_vel, self.ax_vel_filt, self.ax_int) = plt.subplots(4, 1, figsize=(6, 9), dpi=100, sharex=True)
        
        self.line_pos, = self.ax_pos.plot([], [], 'r-', label="Position (m)")
        self.ax_pos.set_title("Ball Position vs. Time")
        self.ax_pos.set_ylabel("X Position (m)")
        self.ax_pos.set_ylim(0, RESIZE_WIDTH / PIXELS_PER_METER)
        self.target_line = self.ax_pos.axhline(self.center_x_m, color='b', linestyle='--', label="Target")
        self.ax_pos.legend()
        self.ax_pos.grid(True)

        self.line_vel, = self.ax_vel.plot([], [], 'g-', label="Raw Velocity (m/s)")
        self.ax_vel.set_title("Ball Velocity vs. Time")
        self.ax_vel.set_ylabel("Raw Velocity (m/s)")
        self.ax_vel.legend()
        self.ax_vel.grid(True)
        
        self.line_vel_filt, = self.ax_vel_filt.plot([], [], 'c-', label="Filtered Velocity (m/s)")
        self.ax_vel_filt.set_title("Filtered Ball Velocity vs. Time")
        self.ax_vel_filt.set_ylabel("Filtered Vel (m/s)")
        self.ax_vel_filt.legend()
        self.ax_vel_filt.grid(True)
        
        self.line_int, = self.ax_int.plot([], [], 'm-', label="Integral Term (m*s)")
        self.ax_int.set_title("Accumulated Error (Integral) vs. Time")
        self.ax_int.set_ylabel("Integral Value (m*s)")
        self.ax_int.set_xlabel("Time (s)")
        self.ax_int.legend()
        self.ax_int.grid(True)
        
        fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(fig, master=self.plot_window)
        self.canvas.get_tk_widget().pack()

    def update_pid_constants(self):
        """Reads values from the GUI and updates the PID controller and target setpoint."""
        kp = self.kp_var.get()
        ki = self.ki_var.get()
        kd = self.kd_var.get()
        d_alpha = self.d_filter_alpha_var.get()
        
        # --- NEW: Update the active target offset only when this button is pressed ---
        self.active_target_offset_m = self.target_offset_m_var.get()
        print(f"Target offset updated to: {self.active_target_offset_m} m")

        self.pid_enabled = any(g != 0 for g in [kp, ki, kd])
        self.pid.update_constants(kp, ki, kd, d_alpha) 
        
        if not self.pid_enabled:
            print("PID is DISABLED (all gains are zero).")
            send_angle_to_servo(90)
        else:
            print("PID is ENABLED.")


    def run_main_loop(self):
        loop_start_time = time.time()

        ret, frame = self.cap.read()
        if not ret:
            self.root.after(100, self.run_main_loop)
            return

        h, w, _ = frame.shape
        r = RESIZE_WIDTH / float(w)
        dim = (RESIZE_WIDTH, int(h * r))
        frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
        frame = frame[0:frame.shape[0] // 2, :]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_bound = np.array([self.h_min.get(), self.s_min.get(), self.v_min.get()])
        upper_bound = np.array([self.h_max.get(), self.s_max.get(), self.v_max.get()])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # --- MODIFIED: Use the internal state variable for the target ---
        current_target_m = self.center_x_m + self.active_target_offset_m
        target_pixel = int(current_target_m * PIXELS_PER_METER)

        if self.show_video_var.get():
            cv2.line(frame, (target_pixel, 0), (target_pixel, frame.shape[0]), (255, 0, 0), 2)
        
        ball_x = None
        if contours:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius > 5:
                ball_x = int(x)
                if self.show_video_var.get():
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
        
        if ball_x is not None:
            self.ball_found = True
            
            ball_x_m = ball_x / PIXELS_PER_METER
            error_m = current_target_m - ball_x_m
            
            pid_output = self.pid.compute(error_m) 
            
            if self.pid_enabled:
                servo_angle = map_pid_to_servo_angle(pid_output)
                send_angle_to_servo(servo_angle)
            
            now = time.time()
            self.t_data.append(now - self.start_time)
            self.x_data.append(ball_x_m)
            self.v_data.append(-self.pid.raw_derivative)
            self.v_filt_data.append(-self.pid.last_derivative)
            self.i_data.append(self.pid.integral)
            
            if self.show_video_var.get():
                text = f"X: {ball_x_m:.3f} m"
                cv2.putText(frame, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        else:
            if self.ball_found:
                self.ball_found = False
                send_angle_to_servo(90)
            
            if self.show_video_var.get():
                text = "X: Not Found"
                cv2.putText(frame, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        try:
            self.line_pos.set_data(self.t_data, self.x_data)
            self.line_vel.set_data(self.t_data, self.v_data)
            self.line_vel_filt.set_data(self.t_data, self.v_filt_data) 
            self.line_int.set_data(self.t_data, self.i_data)
            self.target_line.set_ydata([current_target_m, current_target_m])
            
            if len(self.t_data) > 1:
                self.ax_pos.set_xlim(self.t_data[0], self.t_data[-1])
                for ax in [self.ax_pos, self.ax_vel, self.ax_vel_filt, self.ax_int]:
                    ax.relim()
                    ax.autoscale_view(scalex=False, scaley=True)
            
            if self.plot_window.winfo_exists():
                self.canvas.draw()

            if self.show_video_var.get():
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img_data = cv2.imencode('.ppm', frame_rgb)[1].tobytes()
                img_video = tk.PhotoImage(master=self.video_label, data=img_data)
                self.video_label.imgtk = img_video
                self.video_label.configure(image=img_video)

            if self.show_mask_var.get():
                mask_display = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                img_data_mask = cv2.imencode('.ppm', mask_display)[1].tobytes()
                img_mask = tk.PhotoImage(master=self.mask_label, data=img_data_mask)
                self.mask_label.imgtk = img_mask
                self.mask_label.configure(image=img_mask)

        except (RuntimeError, tk.TclError) as e:
            print(f"Caught a Tkinter error, likely during shutdown: {e}")
            self.running = False

        if self.running:
            processing_time_s = time.time() - loop_start_time
            delay_s = TARGET_LOOP_TIME_S - processing_time_s
            delay_ms = max(1, int(delay_s * 1000))
            self.count += 1
            if self.pid_enabled:
                 print(f"DEBUG: Total Loop Time: {processing_time_s*1000:.2f}ms, Delaying for: {delay_ms}ms, Count: {self.count}")

            self.root.after(delay_ms, self.run_main_loop)

    def close(self):
        print("Closing application...")
        self.running = False
        time.sleep(0.1)
        self.cap.release()
        cv2.destroyAllWindows()
        if ARDUINO and ARDUINO.is_open:
            send_angle_to_servo(90)
            ARDUINO.close()
            print("Arduino connection closed.")
        self.root.destroy()
        print("Application closed.")

# ==============================================================================
# --- Entry Point ---
# ==============================================================================
if __name__ == "__main__":
    print(f"DEBUG [{time.perf_counter():.2f}s]: Application starting...")
    root = tk.Tk()
    try:
        app = BallBalancerApp(root)
        print(f"DEBUG [{time.perf_counter():.2f}s]: Entering main GUI loop.")
        root.mainloop()
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        if root:
            root.destroy()

