import tkinter as tk
import math
import time

class PingPongSimulation:
    """
    A class to simulate the motion of a Ping-Pong ball on an inclined plane.
    This class handles the GUI, physics calculations, and real-time animation.
    """
    def __init__(self, master):
        """
        Initializes the simulation environment.

        Args:
            master (tk.Tk): The root tkinter window.
        """
        self.master = master
        self.master.title("Ping-Pong Incline Simulation")
        self.master.geometry("800x600")

        # --- Simulation Parameters ---
        self.g = 9.81  # Acceleration due to gravity (m/s^2)
        self.ball_radius_meters = 0.02  # Radius of a ping-pong ball (20 mm)
        self.scaling_factor = 1000 # Pixels per meter for visualization
        self.ball_radius_pixels = self.ball_radius_meters * self.scaling_factor
        
        # --- Real-Time Simulation Variables ---
        self.last_update_time = time.monotonic()

        # --- Ball State Variables ---
        self.position = 0.0  # Position along the incline in meters
        self.velocity = 0.0  # Velocity along the incline in m/s
        self.angle_rad = 0.0  # Angle of the ball's rotation in radians

        # --- GUI Setup ---
        self.canvas = tk.Canvas(self.master, bg="lightblue")
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # The angle of the incline, controlled by the slider
        self.incline_angle_deg = tk.DoubleVar(value=0.0)

        # Create the control window for the slider
        self.create_control_window()

        # Draw initial state
        self.draw()

        # Start the simulation loop
        self.update_simulation()

    def create_control_window(self):
        """
        Creates a separate window with a slider to control the incline angle.
        """
        control_window = tk.Toplevel(self.master)
        control_window.title("Controls")
        control_window.geometry("300x150")
        control_window.attributes("-topmost", True) # Keep control window on top

        # Label for the slider
        tk.Label(control_window, text="Incline Angle (degrees)").pack(pady=10)

        # Slider to control the incline angle
        angle_slider = tk.Scale(
            control_window,
            from_=-35,
            to=35,
            orient=tk.HORIZONTAL,
            variable=self.incline_angle_deg,
            resolution=0.5,
            length=250
        )
        angle_slider.pack(pady=5)

        # Reset Button
        reset_button = tk.Button(control_window, text="Reset Ball", command=self.reset_simulation)
        reset_button.pack(pady=10)


    def calculate_acceleration(self):
        """
        Calculates the linear acceleration of the ball down the incline.
        For a hollow sphere rolling without slipping, a = (3/5) * g * sin(theta).

        Returns:
            float: The linear acceleration in m/s^2.
        """
        theta = math.radians(self.incline_angle_deg.get())
        acceleration = (3.0 / 5.0) * self.g * math.sin(theta)
        return acceleration

    def update_simulation(self):
        """
        The main simulation loop. Updates physics and redraws the canvas.
        Uses a dynamic delta_time to ensure real-time simulation speed.
        """
        # --- Real-Time Update ---
        current_time = time.monotonic()
        delta_time = current_time - self.last_update_time
        self.last_update_time = current_time

        # --- Physics Update ---
        acceleration = self.calculate_acceleration()
        self.velocity += acceleration * delta_time
        self.position += self.velocity * delta_time

        # Update angular rotation
        if self.ball_radius_meters > 0:
            angular_velocity = self.velocity / self.ball_radius_meters
            self.angle_rad += angular_velocity * delta_time

        # --- Redraw ---
        self.draw()

        # --- Loop ---
        self.master.after(16, self.update_simulation)

    def reset_simulation(self, event=None):
        """Resets the ball to the start of the incline."""
        self.position = 0.0
        self.velocity = 0.0
        self.angle_rad = 0.0
        # Reset the timer to avoid a large time jump on the next frame
        self.last_update_time = time.monotonic()

    def draw(self):
        """
        Clears and redraws the canvas with the current state of the simulation.
        """
        self.canvas.delete("all")
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()

        if width < 2 or height < 2:
            return

        theta = math.radians(self.incline_angle_deg.get())

        # --- Draw Inclined Plane ---
        pivot_x = width / 2
        pivot_y = height / 2

        # UPDATED: Draw a vertical blue line at the pivot point
        self.canvas.create_line(pivot_x, 0, pivot_x, height, fill="blue", dash=(4, 4))
        
        plane_horizontal_length = width - 100
        half_horizontal_length = plane_horizontal_length / 2
        half_vertical_offset = half_horizontal_length * math.tan(theta)

        plane_start_x = pivot_x - half_horizontal_length
        plane_start_y = pivot_y - half_vertical_offset
        plane_end_x = pivot_x + half_horizontal_length
        plane_end_y = pivot_y + half_vertical_offset

        plane_length_pixels = math.sqrt((plane_end_x - plane_start_x)**2 + (plane_end_y - plane_start_y)**2)
        plane_length_meters = plane_length_pixels / self.scaling_factor

        self.canvas.create_line(plane_start_x, plane_start_y, plane_end_x, plane_end_y, width=5, fill="saddlebrown")
        self.canvas.create_polygon(plane_start_x, plane_start_y, plane_end_x, plane_end_y, plane_end_x, plane_end_y+50, plane_start_x, plane_start_y+50, fill="darkkhaki", outline="black")

        # --- Boundary Checks ---
        if self.position >= plane_length_meters:
            self.position = plane_length_meters
            self.velocity = 0 # Stop the ball
        elif self.position < 0:
            self.position = 0
            self.velocity = 0 # Stop the ball

        # --- Calculate Ball Position ---
        pos_pixels = self.position * self.scaling_factor

        ball_center_x = plane_start_x + pos_pixels * math.cos(theta)
        ball_center_y = plane_start_y + pos_pixels * math.sin(theta)

        # --- Draw Ball ---
        r = self.ball_radius_pixels
        offset_x = r * math.sin(theta)
        offset_y = -r * math.cos(theta)

        x0 = ball_center_x + offset_x - r
        y0 = ball_center_y + offset_y - r
        x1 = ball_center_x + offset_x + r
        y1 = ball_center_y + offset_y + r

        self.canvas.create_oval(x0, y0, x1, y1, fill="white", outline="black", width=2)

        # --- Draw a line inside the ball to show rotation ---
        line_end_x = (ball_center_x + offset_x) + r * math.cos(self.angle_rad)
        line_end_y = (ball_center_y + offset_y) + r * math.sin(self.angle_rad)
        self.canvas.create_line(ball_center_x + offset_x, ball_center_y + offset_y,
                                line_end_x, line_end_y, fill="red", width=2)

        # --- Display Info ---
        info_text = (
            f"Angle: {self.incline_angle_deg.get():.1f}°\n"
            f"Velocity: {self.velocity:.2f} m/s\n"
            f"Position: {self.position:.2f} m"
        )
        self.canvas.create_text(10, 10, anchor="nw", text=info_text, font=("Arial", 12), fill="black")


def main():
    """The main function to create the root window and start the application."""
    root = tk.Tk()
    app = PingPongSimulation(root)
    root.mainloop()

if __name__ == "__main__":
    main()
