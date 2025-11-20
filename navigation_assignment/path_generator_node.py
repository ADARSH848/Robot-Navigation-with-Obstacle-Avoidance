import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np
from navigation_assignment.cubic_spline import CubicSpline2D

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.vis_pub = self.create_publisher(MarkerArray, 'waypoints_marker', 10)
        
        # --- INSPECTION LOOP WAYPOINTS ---
        # A complex Figure-8 style circuit that returns to start.
        self.wx = [
            0.0,  # Start
            2.5,  # Enter Corridor
            5.0,  # Pass first barrel (Pass on Left)
            7.0,  # Approach Turn
            9.5,  # Far Turn (Go around Dumpster top)
            11.0, # Apex of turn
            9.5,  # Exit turn (Dumpster bottom)
            7.0,  # Return Leg
            5.0,  # Pass return wall
            2.0,  # Align for home
            0.0   # Finish (Loop Closed)
        ]
        self.wy = [
            0.0,  # Start
            0.0,  # Corridor
            0.8,  # Left of Center Barrel
            -0.5, # Weave Right
            1.8,  # Top of Dumpster
            0.0,  # Behind Dumpster
            -1.8, # Bottom of Dumpster
            -1.5, # Middle Return
            -3.0, # Wide Return (Avoid Wall)
            -1.5, # Angling in
            0.0   # Finish
        ]
        
        self.generate_path()

    def generate_path(self):
        # Interpolating Cubic Spline (Loop Logic)
        # Note: We duplicate the first point at the end to ensure the spline closes smoothly
        sp = CubicSpline2D(self.wx, self.wy)
        s = np.arange(0, sp.s[-1], 0.1) 

        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            iyaw = sp.calc_yaw(i_s)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(ix)
            pose.pose.position.y = float(iy)
            q = self.euler_to_quaternion(0, 0, iyaw)
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            path_msg.poses.append(pose)

        marker_array = MarkerArray()
        for i in range(len(self.wx)):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = self.wx[i]
            marker.pose.position.y = self.wy[i]
            marker.pose.position.z = 0.1
            marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3
            marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.path_pub.publish(path_msg)
        self.vis_pub.publish(marker_array)
        self.timer = self.create_timer(1.0, lambda: (self.path_pub.publish(path_msg), self.vis_pub.publish(marker_array)))

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()