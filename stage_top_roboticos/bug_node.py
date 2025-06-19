#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf_transformations import euler_from_quaternion
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class BugNode(Node):
    def __init__(self):
        super().__init__("bug_node")

        self.angle = np.radians(90)
        self.threshold = 0.5

        self.odom = None
        self.laser = None

        self.state = "MOVE"
        self.follow_state = "AWAY"

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.odom_sub = self.create_subscription(Odometry,"/ground_truth", self.odom_cb, qos_profile=sub_qos)
        self.laser_sub = self.create_subscription(LaserScan,"/base_scan", self.laser_cb, qos_profile=sub_qos)
        
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )
        self.pub = self.create_publisher(Twist,"/cmd_vel", qos_profile=pub_qos)
        self.timer = self.create_timer(1/60, self.run)

    def odom_cb(self, msg: Odometry):
        self.odom = msg
    
    def laser_cb(self, msg: LaserScan):
        self.laser = msg

    def getPointToLineDistance(self,P1,P2,P3) -> float:
        return np.linalg.norm(np.cross(P2-P1, P1-P3))/np.linalg.norm(P2-P1)
    
    def getAngleIdx(self, angle : float, min : float, max, size : int) -> int:
        return int((angle - min) * size / (max - min))
    
    def run(self):
        if self.odom is None or self.laser is None:
            return
        
        pos = np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y])

        target = np.array([7,7])

        _, __, theta = euler_from_quaternion([self.odom.pose.pose.orientation.x,
                                       self.odom.pose.pose.orientation.y,
                                       self.odom.pose.pose.orientation.z,
                                       self.odom.pose.pose.orientation.w])
        # theta += np.pi/4
        
        angle_idx = int(self.angle / (2*self.laser.angle_increment))

        dist = target - pos
        angle_to_target = np.arctan2(dist[0],dist[1])
        if angle_to_target < 0:
            angle_to_target + (2*np.pi)
        
        angle_deviation = angle_to_target - theta 
        if angle_deviation > np.pi:
            angle_deviation -= 2*np.pi
        
        scans = self.laser.ranges
        min_laser = self.laser.angle_min
        max_laser = self.laser.angle_max
        size = len(scans)
        middle = size//2
        
        readings = scans[middle-angle_idx:middle+angle_idx]
        min_distance = min(readings)

        msg = Twist()
        if self.state == "MOVE":
            if min_distance < self.threshold:
                self.obstacle_orig = pos
                self.state = "FOLLOW"
                self.get_logger().info(f"State changed to: {self.state}")
            else:
                msg.linear.x = 0.5 * 2 
                msg.angular.z = 1.0 * angle_deviation * 2

        elif self.state == "FOLLOW":
            distance_to_line = self.getPointToLineDistance(self.obstacle_orig, target, pos)
            if self.follow_state == "CLOSING" and 0.1 > distance_to_line:
                self.state = "MOVE"
                self.follow_state = "AWAY"
                self.get_logger().info(f"State changed to: {self.state}")
                self.get_logger().info(f"Follow state changed to: {self.follow_state}")
            else:
                corner_idx = self.getAngleIdx(np.radians(45),min_laser, max_laser, size)
                side_readings = scans[corner_idx-angle_idx:corner_idx+angle_idx]
                corner_distance = min(side_readings)
                if self.follow_state == "AWAY" and distance_to_line > 0.2:
                    self.follow_state = "CLOSING"
                    self.get_logger().info(f"Follow state changed to: {self.follow_state}")
                if corner_distance > self.threshold and min_distance > self.threshold:
                    msg.linear.x = 0.1*2
                    msg.angular.z = -0.3*2
                else:
                    msg.linear.x = 0.1*2
                    msg.angular.z = 0.3*2
        else:
            raise ValueError(f"Unknown state: {self.state}")           

        
        self.pub.publish(msg)
        return
        
        



def main(args=None):
    rclpy.init(args=args)
    node  = BugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.try_shutdown()
    
