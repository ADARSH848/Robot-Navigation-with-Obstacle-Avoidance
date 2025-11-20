import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
import matplotlib.pyplot as plt
import math
import sys

class ResultPlotter(Node):
    def __init__(self):
        super().__init__('result_plotter')
        
        # Subscribe to the Reference Path (Green Line)
        self.path_sub = self.create_subscription(Path, '/global_plan', self.path_cb, 10)
        
        # Subscribe to the Robot's Actual Position
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # Data Storage
        self.ref_x = []
        self.ref_y = []
        self.actual_x = []
        self.actual_y = []
        self.cross_track_errors = []
        self.time_stamps = []
        
        self.start_time = None
        self.path_received = False
        
        print("\nSTATUS: Plotter Ready. Waiting for Data...")
        print("--- INSTRUCTIONS ---")
        print("1. Let the robot finish the loop.")
        print("2. Press 'Ctrl+C' in THIS terminal to generate the graphs.\n")

    def path_cb(self, msg):
        # Save the Reference Path only once
        if not self.path_received and len(msg.poses) > 0:
            self.ref_x = [p.pose.position.x for p in msg.poses]
            self.ref_y = [p.pose.position.y for p in msg.poses]
            self.path_received = True
            print(f"Reference Path Received: {len(self.ref_x)} points.")

    def odom_cb(self, msg):
        if not self.path_received: return

        # Time
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds
        
        current_time = (self.get_clock().now().nanoseconds - self.start_time) / 1e9
        
        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.actual_x.append(x)
        self.actual_y.append(y)
        self.time_stamps.append(current_time)
        
        # Calculate Simple Error (Distance to nearest path point)
        min_dist = float('inf')
        # Optimization: Search only a subset if arrays are large, 
        # but for this assignment, full search is fine for plotting accuracy.
        for rx, ry in zip(self.ref_x, self.ref_y):
            dist = math.sqrt((x - rx)**2 + (y - ry)**2)
            if dist < min_dist:
                min_dist = dist
        
        self.cross_track_errors.append(min_dist)

    def save_and_show(self):
        if not self.actual_x:
            print("No data recorded! Did you run the simulation?")
            return

        print("Generating Plots... Please Wait.")
        
        # --- PLOT 1: Trajectory Tracking (Top View) ---
        plt.figure(figsize=(10, 6))
        plt.plot(self.ref_x, self.ref_y, 'g--', linewidth=2, label='Reference Path (Cubic Spline)')
        plt.plot(self.actual_x, self.actual_y, 'b-', linewidth=2, label='Actual Robot Trajectory')
        
        # Add labels
        plt.title('Trajectory Tracking Performance: Inspection Circuit')
        plt.xlabel('X Position [m]')
        plt.ylabel('Y Position [m]')
        plt.legend()
        plt.grid(True)
        plt.axis('equal') # Important so the shape looks right
        
        # Save comparison image
        plt.savefig('trajectory_comparison.png')
        print("Saved: trajectory_comparison.png")

        # --- PLOT 2: Error Analysis ---
        plt.figure(figsize=(10, 4))
        plt.plot(self.time_stamps, self.cross_track_errors, 'r-', linewidth=1)
        plt.title('Tracking Error over Time')
        plt.xlabel('Time [s]')
        plt.ylabel('Cross Track Error [m]')
        plt.grid(True)
        
        # Add mean error text
        avg_err = sum(self.cross_track_errors) / len(self.cross_track_errors)
        plt.axhline(y=avg_err, color='k', linestyle='--', label=f'Mean Error: {avg_err:.3f}m')
        plt.legend()
        
        plt.savefig('error_analysis.png')
        print("Saved: error_analysis.png")
        
        # Show interactive
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    plotter = ResultPlotter()
    
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        plotter.save_and_show()
        plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()