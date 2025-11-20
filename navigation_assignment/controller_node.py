import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class PrecisionController(Node):
    def __init__(self):
        super().__init__('precision_controller')
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.path_sub = self.create_subscription(Path, 'global_plan', self.path_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)
        
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.path = []
        self.got_path = False
        self.current_path_index = 0
        
        # State Flags
        self.mission_complete = False
        
        # Obstacle State
        self.closest_obs_dist = 10.0
        self.closest_obs_angle = 0.0
        
        # Loop
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Final Precision Controller (Hard Stop Fixed) Started")

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def path_cb(self, msg):
        # Reset mission if a new path comes in
        if not self.got_path:
            self.path = msg.poses
            self.current_path_index = 0
            self.got_path = True
            self.mission_complete = False
            self.get_logger().info("New Plan Received. Starting Mission.")

    def scan_cb(self, msg):
        ranges = np.array(msg.ranges)
        ranges[ranges == float('inf')] = 10.0
        n = len(ranges)
        if n == 0: return
        
        # Front 60 deg
        window = 30 
        indices = list(range(0, window)) + list(range(n-window, n))
        min_dist = 10.0
        min_idx = -1
        
        for i in indices:
            if ranges[i] < min_dist and ranges[i] > 0.05:
                min_dist = ranges[i]
                min_idx = i
        
        self.closest_obs_dist = min_dist
        if min_idx != -1:
            if min_idx < n/2:
                self.closest_obs_angle = min_idx * msg.angle_increment
            else:
                self.closest_obs_angle = (min_idx - n) * msg.angle_increment

    def control_loop(self):
        # 1. PARKING MODE (If finished, stay stopped)
        if self.mission_complete:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_pub.publish(stop_msg)
            return

        if not self.got_path or len(self.path) == 0: return

        # 2. CHECK GOAL REACHED (Before calculating anything)
        final_x = self.path[-1].pose.position.x
        final_y = self.path[-1].pose.position.y
        dist_to_finish = math.hypot(self.x - final_x, self.y - final_y)
        
        # If within 20cm of end, STOP.
        if self.current_path_index >= len(self.path) - 10 and dist_to_finish < 0.20:
             self.mission_complete = True
             self.get_logger().info("MISSION COMPLETE. Parking Robot.")
             return

        # --- 3. Dynamic Lookahead ---
        if self.closest_obs_dist < 0.7:
             lookahead_search = 1.5
        else:
             lookahead_search = 0.0 

        # Find Closest Point (Base Index)
        closest_dist = float('inf')
        best_idx = self.current_path_index
        search_limit = min(self.current_path_index + 50, len(self.path))
        
        for i in range(self.current_path_index, search_limit):
            px = self.path[i].pose.position.x
            py = self.path[i].pose.position.y
            dist = math.hypot(self.x - px, self.y - py)
            if dist < closest_dist:
                closest_dist = dist
                best_idx = i
        self.current_path_index = best_idx
        
        # Find Target
        target_idx = best_idx
        if lookahead_search > 0:
            for i in range(best_idx, len(self.path)):
                px = self.path[i].pose.position.x
                py = self.path[i].pose.position.y
                dist = math.hypot(self.x - px, self.y - py)
                if dist > lookahead_search:
                    target_idx = i
                    break

        # --- 4. Stanley Heading ---
        px = self.path[target_idx].pose.position.x
        py = self.path[target_idx].pose.position.y
        
        if lookahead_search > 0:
             path_yaw = math.atan2(py - self.y, px - self.x)
        else:
             next_idx = min(target_idx + 1, len(self.path) - 1)
             npx = self.path[next_idx].pose.position.x
             npy = self.path[next_idx].pose.position.y
             path_yaw = math.atan2(npy - py, npx - px)

        heading_err = path_yaw - self.yaw
        heading_err = (heading_err + math.pi) % (2 * math.pi) - math.pi

        # Cross Track Error
        cpx = self.path[best_idx].pose.position.x
        cpy = self.path[best_idx].pose.position.y
        dx = self.x - cpx
        dy = self.y - cpy
        
        next_idx = min(best_idx + 1, len(self.path) - 1)
        path_dx = self.path[next_idx].pose.position.x - cpx
        path_dy = self.path[next_idx].pose.position.y - cpy
        cross_product = path_dx * dy - path_dy * dx
        
        cte = closest_dist
        if cross_product > 0: cte = -cte 
        
        # --- 5. Control Law ---
        v_base = 0.22
        k_cte = 2.5 
        k_heading = 1.0
        
        steer_correction = math.atan2(k_cte * cte, v_base + 0.1)
        final_steering = k_heading * heading_err + steer_correction

        # --- 6. Dynamic Braking ---
        if abs(heading_err) > 0.35:
            v_cmd = 0.05 
            final_steering *= 1.5 
        elif abs(heading_err) > 0.15:
            v_cmd = 0.12 
        else:
            v_cmd = v_base 

        # --- 7. Obstacle Avoidance Override ---
        if self.closest_obs_dist < 0.6:
             direction = -1.0 if self.closest_obs_angle > 0 else 1.0
             tangent_angle = self.closest_obs_angle + (direction * math.pi / 2.0)
             
             if self.closest_obs_dist < 0.4:
                 weight = 1.0
             else:
                 weight = 1.0 - ((self.closest_obs_dist - 0.4) / (0.6 - 0.4))
                 weight = max(0.0, min(weight, 1.0))
             
             final_steering = (1.0 - weight) * final_steering + weight * tangent_angle
             
             if self.closest_obs_dist < 0.5: v_cmd = 0.15

        # Emergency Unstick
        if self.closest_obs_dist < 0.20:
            v_cmd = 0.0
            direction = -1.0 if self.closest_obs_angle > 0 else 1.0
            final_steering = direction * 0.6

        # Publish
        msg = Twist()
        msg.linear.x = float(v_cmd)
        msg.angular.z = float(max(-2.0, min(final_steering, 2.0)))
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()