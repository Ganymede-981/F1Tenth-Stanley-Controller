#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class StanleyFinal(Node):
    def __init__(self):
        super().__init__('stanley_final')

        # --- Params ---
        self.k = 1.0                # Stanley gain
        self.softening_factor = 0.1 # Avoid division by 0
        self.steer_limit = np.deg2rad(45)  # Max steering (30 deg)

        self.v_min = 3.25   # Slow speed (tight curve)
        self.v_max = 8.3   # Fast speed (straight)
        self.lookahead_idx = 5

        # --- Load raceline/centerline CSV directly (no pkg) ---
        csv_file = "/sim_ws/src/f1tenth_gym_ros/maps/Budapest_raceline.csv"
        # Use only x,y columns of raceline/centerline
        self.centerline = np.loadtxt(csv_file, delimiter=';', comments='#')[:, 1:3]


        if self.centerline.shape[1] < 2:
            raise ValueError("CSV must have at least 2 cols (x,y).")

        self.get_logger().info(f"Loaded {self.centerline.shape[0]} waypoints from {csv_file}")

        # --- ROS2 sub/pub ---
        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def odom_callback(self, msg):
        # Current state
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = self.get_yaw(q)

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v = (vx**2 + vy**2)**0.5

        diff = self.centerline - np.array([x, y])   # (N,2) - (2,)
        dist_sq = np.sum(diff**2, axis=1)
        i = np.argmin(dist_sq)
        i2 = (i + 1) % len(self.centerline)

        # Path tangent from x,y only
        path_tangent = self.centerline[i2] - self.centerline[i]
        psi = np.arctan2(path_tangent[1], path_tangent[0])

        # Cross-track error
        vec_nearest_current = np.array([x, y]) - self.centerline[i]
        cross = np.cross(vec_nearest_current, path_tangent)
        e = np.sign(cross) * np.linalg.norm(vec_nearest_current)

        # Stanley steering
        heading_error = self.wrap_angle(psi - yaw)
        crosstrack_term = np.arctan2(self.k * e, v + self.softening_factor)
        steer_angle = heading_error + crosstrack_term
        steer_angle = np.clip(steer_angle, -self.steer_limit, self.steer_limit)

        # --- Adaptive speed control ---
        idx_next = (i + self.lookahead_idx) % len(self.centerline)
        tangent_next = self.centerline[(idx_next + 1) % len(self.centerline)] - self.centerline[idx_next]
        psi_next = np.arctan2(tangent_next[1], tangent_next[0])

        delta_yaw = abs(self.wrap_angle(psi_next - psi))
        ds = np.linalg.norm(self.centerline[idx_next] - self.centerline[i])
        curvature = delta_yaw / max(ds, 1e-3)

        speed = self.v_max - curvature * (self.v_max - self.v_min) * 8.0
        speed = np.clip(speed, self.v_min, self.v_max)

        # --- Publish drive command ---
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(steer_angle)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)

        self.get_logger().info(
            f"Pos=({x:.2f},{y:.2f}) Yaw={np.rad2deg(yaw):.1f}° "
            f"Steer={np.rad2deg(steer_angle):.1f}° Speed={speed:.2f} m/s"
        )

    @staticmethod
    def get_yaw(q):
        """Convert quaternion to yaw angle (radians)."""
        x, y, z, w = q.x, q.y, q.z, q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return np.arctan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_angle(angle):
        """Wrap angle between [-pi, pi]."""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = StanleyFinal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
