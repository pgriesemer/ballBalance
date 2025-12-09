import tkinter as tk
import math
import time

class PIDController:
    """A simple PID controller."""
    def __init__(self, Kp, Ki, Kd, setpoint):
        """
        Initializes the PID controller.

        Args:
            Kp (float): Proportional gain.
            Ki (float): Integral gain.
            Kd (float): Derivative gain.
            setpoint (float): The target value for the process variable.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self.integral_error = 0
        self.previous_error = 0
        self.last_time = time.monotonic()
        
        # Integral windup limits to prevent the integral term from becoming excessively large
        self.integral_min = -10
        self.integral_max = 10

    def update(self, process_variable):
        """
        Calculates the PID output.

        Args:
            process_variable (float): The current measured value.

        Returns:
            float: The calculated control output.
        """
        current_time = time.monotonic()
        delta_time = current_time - self.last_time

        if delta_time == 0:
            return 0  # Avoid division by zero

        error = self.setpoint - process_variable

        # Proportional term
        p_term = self.Kp * error

        # Integral term with anti-windup
        self.integral_error += error * delta_time
        self.integral_error = max(self.integral_min, min(self.integral_error, self.integral_max))
        i_term = self.Ki * self.integral_error

        # Derivative term
        derivative_error = (error - self.previous_error) / delta_time
        d_term = self.Kd * derivative_error

        # Update state for next iteration
        self.previous_error = error
        self.last_time = current_time

        # Total output
        output = p_term + i_term + d_term
        return output

    def reset(self):
        """Resets the integral and derivative error terms."""
        self.integral_error = 0
        self.previous_error = 0
        self.last_time = time.monotonic()

class PingPongSimulation:
    def __init__(self, master):
        self.master = master
        self.master.title("Ping-Pong Incline Simulation")
        self.master.geometry("800x600")

        # --- Simulation Parameters ---
        self.g = 9.81
        self.ball_radius_meters = 0.02
        self.scaling_factor = 1000
        self.ball_radius_pixels = self.ball_radius_meters * self.scaling_factor
        self.last_update_time = time.monotonic()
        
        # UPDATED: Set a fixed length for the inclined plane (1 foot = 0.3048 meters)
        self.plane_length_meters = 0.3048

        # --- Ball State Variables ---
        self.position = 0.0
        self.velocity = 0.0
        self.angle_rad = 0.0
        
        # --- GUI Variables ---
        self.incline_angle_deg = tk.DoubleVar(value=0.0)
        self.pid_enabled = tk.BooleanVar(value=False)
        self.kp_var = tk.DoubleVar(value=80.0) # Proportional Gain
        self.ki_var = tk.DoubleVar(value=25.0) # Integral Gain
        self.kd_var = tk.DoubleVar(value=40.0) # Derivative Gain

        # --- PID Controller ---
        # The setpoint is the middle of the fixed-length plane
        pid_setpoint = self.plane_length_meters / 2.0
        self.pid_controller = PIDController(self.kp_var.get(), self.ki_var.get(), self.kd_var.get(), setpoint=pid_setpoint)

        # --- GUI Setup ---
        self.canvas = tk.Canvas(self.master, bg="lightblue")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.create_control_window()
        self.draw()
        self.update_simulation()

    def create_control_window(self):
        control_window = tk.Toplevel(self.master)
        control_window.title("Controls")
        control_window.geometry("350x300")
        control_window.attributes("-topmost", True)

        # --- PID Control Section ---
        pid_frame = tk.LabelFrame(control_window, text="PID Controller", padx=10, pady=10)
        pid_frame.pack(pady=10, padx=10, fill="x")

        self.pid_checkbutton = tk.Checkbutton(pid_frame, text="Enable PID Control", variable=self.pid_enabled, command=self.toggle_pid_control)
        self.pid_checkbutton.grid(row=0, columnspan=2, pady=5)

        tk.Label(pid_frame, text="Kp (Proportional):").grid(row=1, column=0, sticky="w")
        tk.Entry(pid_frame, textvariable=self.kp_var, width=10).grid(row=1, column=1)
        
        tk.Label(pid_frame, text="Ki (Integral):").grid(row=2, column=0, sticky="w")
        tk.Entry(pid_frame, textvariable=self.ki_var, width=10).grid(row=2, column=1)

        tk.Label(pid_frame, text="Kd (Derivative):").grid(row=3, column=0, sticky="w")
        tk.Entry(pid_frame, textvariable=self.kd_var, width=10).grid(row=3, column=1)

        # --- Manual Control Section ---
        manual_frame = tk.LabelFrame(control_window, text="Manual Control", padx=10, pady=10)
        manual_frame.pack(pady=10, padx=10, fill="x")
        
        self.angle_slider = tk.Scale(
            manual_frame, from_=-35, to=35, orient=tk.HORIZONTAL,
            variable=self.incline_angle_deg, resolution=0.5, length=300
        )
        self.angle_slider.pack(pady=5)

        reset_button = tk.Button(control_window, text="Reset Ball", command=self.reset_simulation)
        reset_button.pack(pady=10)
        
        self.toggle_pid_control() # Set initial state of the slider

    def toggle_pid_control(self):
        """Enables/disables the manual slider based on PID checkbox state."""
        if self.pid_enabled.get():
            self.angle_slider.config(state=tk.DISABLED)
            self.pid_controller.reset() # Reset PID on enabling
        else:
            self.angle_slider.config(state=tk.NORMAL)

    def calculate_acceleration(self):
        theta = math.radians(self.incline_angle_deg.get())
        return (3.0 / 5.0) * self.g * math.sin(theta)

    def update_simulation(self):
        current_time = time.monotonic()
        delta_time = current_time - self.last_update_time
        self.last_update_time = current_time
        
        if self.pid_enabled.get():
            self.pid_controller.Kp = self.kp_var.get()
            self.pid_controller.Ki = self.ki_var.get()
            self.pid_controller.Kd = self.kd_var.get()
            
            output_angle = self.pid_controller.update(self.position)
            clamped_angle = max(-35.0, min(output_angle, 35.0))
            self.incline_angle_deg.set(clamped_angle)

        # --- Physics Update ---
        acceleration = self.calculate_acceleration()
        self.velocity += acceleration * delta_time
        self.position += self.velocity * delta_time

        if self.ball_radius_meters > 0:
            angular_velocity = self.velocity / self.ball_radius_meters
            self.angle_rad += angular_velocity * delta_time

        self.draw()
        self.master.after(16, self.update_simulation)

    def reset_simulation(self, event=None):
        self.position = 0.0
        self.velocity = 0.0
        self.angle_rad = 0.0
        self.pid_controller.reset()
        self.last_update_time = time.monotonic()

    def draw(self):
        self.canvas.delete("all")
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()

        if width < 2 or height < 2: return

        theta = math.radians(self.incline_angle_deg.get())
        pivot_x = width / 2
        pivot_y = height / 2

        self.canvas.create_line(pivot_x, 0, pivot_x, height, fill="blue", dash=(4, 4))
        
        # UPDATED: Calculate endpoints based on the fixed plane length
        half_length_pixels = (self.plane_length_meters * self.scaling_factor) / 2.0
        
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        plane_start_x = pivot_x - half_length_pixels * cos_theta
        plane_start_y = pivot_y - half_length_pixels * sin_theta
        plane_end_x = pivot_x + half_length_pixels * cos_theta
        plane_end_y = pivot_y + half_length_pixels * sin_theta

        self.canvas.create_line(plane_start_x, plane_start_y, plane_end_x, plane_end_y, width=5, fill="saddlebrown")
        self.canvas.create_polygon(plane_start_x, plane_start_y, plane_end_x, plane_end_y, plane_end_x, plane_end_y+50, plane_start_x, plane_start_y+50, fill="darkkhaki", outline="black")

        # Boundary Checks
        if self.position >= self.plane_length_meters:
            self.position = self.plane_length_meters
            self.velocity = 0
        elif self.position < 0:
            self.position = 0
            self.velocity = 0

        pos_pixels = self.position * self.scaling_factor
        ball_center_x = plane_start_x + pos_pixels * math.cos(theta)
        ball_center_y = plane_start_y + pos_pixels * math.sin(theta)

        r = self.ball_radius_pixels
        offset_x = r * math.sin(theta)
        offset_y = -r * math.cos(theta)

        x0, y0 = ball_center_x + offset_x - r, ball_center_y + offset_y - r
        x1, y1 = ball_center_x + offset_x + r, ball_center_y + offset_y + r
        self.canvas.create_oval(x0, y0, x1, y1, fill="white", outline="black", width=2)

        line_end_x = (ball_center_x + offset_x) + r * math.cos(self.angle_rad)
        line_end_y = (ball_center_y + offset_y) + r * math.sin(self.angle_rad)
        self.canvas.create_line(ball_center_x + offset_x, ball_center_y + offset_y, line_end_x, line_end_y, fill="red", width=2)

        info_text = (
            f"Angle: {self.incline_angle_deg.get():.1f}°\n"
            f"Velocity: {self.velocity:.2f} m/s\n"
            f"Position: {self.position:.2f} m"
        )
        self.canvas.create_text(10, 10, anchor="nw", text=info_text, font=("Arial", 12), fill="black")

def main():
    root = tk.Tk()
    app = PingPongSimulation(root)
    root.mainloop()

if __name__ == "__main__":
    main()
