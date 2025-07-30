#!/usr/bin/env python

import math
import enum
import rclpy
import numpy as np
from rclpy.node import Node

from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class BotState(enum.Enum):
    LOOK_TOWARDS = 0  # rotate bots towards the goal
    GOAL_SEEK = 1  # follow line
    WALL_FOLLOW = 2  # Go around the wall / avoid obstacles

class BugNode(Node):

    def __init__(self):
        super().__init__("bug_node")
        self.get_logger().info("Starting Node")



        self.yaw = 0
        self.yaw_threshold = math.pi / 90
        self.goal_distance_threshold = 0.25
        self.currentBotState = BotState.LOOK_TOWARDS

        # base scan laser range values
        self.maxRange = 3
        self.minRange = 0

        self.bot_pose = None
        self.init_bot_pose = []
        self.beacon_pose = None
        self.bot_motion = None  # cmd_vel publisher
        self.homing_signal = None  # subscriber
        self.init_config_complete = False
        self.wall_hit_point = None
        self.beacon_found = False
        self.twist = Twist()
        self.distance_moved = 0

        self.front_obs_distance = None
        self.left_obs_distance = None

        self.wall_folllowing = False
        self.load_waypoints()
        self.init()
    
    def load_waypoints(self):

        self.waypoint_index = 0
        self.waypoints : list[Pose] = []
        self.waypoints.append(Pose())
        self.waypoints[-1].position.x = 7.0
        self.waypoints[-1].position.y = 7.0
        self.waypoints.append(Pose())
        self.waypoints[-1].position.x = 7.0
        self.waypoints[-1].position.y = -3.0

    def normalize(self, angle):
        if math.fabs(angle) > math.pi:
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle


    def look_towards(self, des_pos):
        # global yaw, yaw_threshold, bot_motion, currentBotState, twist
        quaternion = (
            des_pos.orientation.x,
            des_pos.orientation.y,
            des_pos.orientation.z,
            des_pos.orientation.w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]  # bot's yaw
        beacon_yaw = math.atan2(self.beacon_pose.position.y - des_pos.position.y, self.beacon_pose.position.x - des_pos.position.x)
        yaw_diff = self.normalize(beacon_yaw - yaw)

        if math.fabs(yaw_diff) > self.yaw_threshold:
            self.twist.angular.z = -0.5  # clockwise rotation if yaw_diff > 0 else 0.5  # counter-clockwise rotation

        if math.fabs(yaw_diff) <= self.yaw_threshold:
            self.twist.angular.z = 0.0
            self.currentBotState = BotState.GOAL_SEEK
            self.get_logger().info("Goal Seek")
        self.bot_motion_pub.publish(self.twist)


    def goal_seek(self):
        # global zone_F, currentBotState, bot_pose, wall_hit_point, front_obs_distance, left_obs_distance

        obstacle_in_front = np.any((self.zone_F < 0.85))
        distance_bot_to_target = math.sqrt(pow(self.bot_pose.position.y - self.beacon_pose.position.y, 2) + pow(self.bot_pose.position.x - self.beacon_pose.position.x, 2))
        if obstacle_in_front and np.any((self.zone_F < distance_bot_to_target)):
            self.twist.linear.x = 0.0
            self.wall_hit_point = self.bot_pose.position
            self.currentBotState = BotState.WALL_FOLLOW
            self.get_logger().info("Wall Follow")
        else:
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.5
        self.bot_motion_pub.publish(self.twist)


    def wall_follow(self):
        obstacle_in_front = np.any((self.zone_F < self.front_obs_distance))
        distance_moved = math.sqrt(pow(self.bot_pose.position.y - self.wall_hit_point.y, 2) + pow(self.bot_pose.position.x - self.wall_hit_point.x, 2))
        distance_hit_to_target = math.sqrt(pow(self.beacon_pose.position.y - self.wall_hit_point.y, 2) + pow(self.beacon_pose.position.x - self.wall_hit_point.x, 2))
        distance_bot_to_target = math.sqrt(pow(self.bot_pose.position.y - self.beacon_pose.position.y, 2) + pow(self.bot_pose.position.x - self.beacon_pose.position.x, 2))
        # self.get_logger().info("1")

        if self.line_distance() < 0.2 and distance_moved > 0.5 and distance_hit_to_target > distance_bot_to_target:
            # self.get_logger().info("2")
            print("line_hit")
            print(distance_moved)
            # found line point. rotate and move forward
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.0
            self.currentBotState = BotState.LOOK_TOWARDS
            self.get_logger().info("look towards")
            return
        elif obstacle_in_front:  # turn right
            # self.get_logger().info("3")
            self.twist.angular.z = -0.5
            self.twist.linear.x = 0.0
        elif np.all((self.zone_FL >= self.left_obs_distance)):  # or np.any((zone_FR <= 1)):  # turn left
            # self.get_logger().info("4")
            self.twist.angular.z = 0.5
            self.twist.linear.x = 0.1
        else:
            # self.get_logger().info("5")
            self.twist.angular.z = 0.0  # move forward
            self.twist.linear.x = 0.5

        self.bot_motion_pub.publish(self.twist)


    def line_distance(self):
        # global init_bot_pose, beacon_pose, bot_pose
        point_1 = self.init_bot_pose  # in form of array
        point_2 = self.beacon_pose.position
        point_k = self.bot_pose.position
        numerator = math.fabs((point_2.y - point_1[1]) * point_k.x - (point_2.x - point_1[0]) * point_k.y + (point_2.x * point_1[1]) - (point_2.y * point_1[0]))
        denominator = math.sqrt(pow(point_2.y - point_1[1], 2) + pow(point_2.x - point_1[0], 2))
        return numerator / denominator


    # def callback(self, msg : PoseStamped):
    #     self.beacon_pose = msg.pose
    #     self.check_init_config()

    def get_base_truth(self, bot_data : Odometry):
        self.bot_pose = bot_data.pose.pose
        if not self.init_config_complete:
            self.check_init_config()

        if self.beacon_pose is not None:
            goal_distance = math.sqrt(pow(self.bot_pose.position.y - self.beacon_pose.position.y, 2) + pow(self.bot_pose.position.x - self.beacon_pose.position.x, 2))
            if goal_distance <= self.goal_distance_threshold:
                self.get_logger().info(f"Waypoint {self.waypoint_index} found")
                self.waypoint_index += 1
                if  self.waypoint_index < len(self.waypoints):
                    self.currentBotState = BotState.LOOK_TOWARDS
                    self.beacon_pose = self.waypoints[self.waypoint_index]
                    self.get_logger().info(f"Going to x: {self.beacon_pose.position.x}, y: {self.beacon_pose.position.y}")
                else:
                    self.beacon_found = True
                    self.get_logger().info("Done!!")

    def idle(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.bot_motion_pub.publish(self.twist)
        self.get_logger().info("Waypoint found!!")


    def process_sensor_info(self, data : LaserScan):
        
        self.maxRange = data.range_max
        self.minRange = data.range_min
        zone = np.array(data.ranges)
        self.zone_R = zone[0:5]  
        self.zone_FR = zone[6:94]
        self.zone_F = zone[95:175]
        self.zone_FL = zone[176:265]
        self.zone_L = zone[266:270]
        if self.front_obs_distance is None and self.left_obs_distance is None:
            self.front_obs_distance = 1
            self.left_obs_distance = 1


    def check_init_config(self):
        if self.bot_pose is not None and self.beacon_pose is not None:
            self.init_config_complete = True
            self.init_bot_pose = [self.bot_pose.position.x, self.bot_pose.position.y]
            self.runtime = self.create_timer(1/10, self.bot_bug2)
            self.get_logger().info("Inited")


    def bot_bug2(self):

        if not self.init_config_complete:
            return
        if self.beacon_found:
            self.runtime.cancel()
            self.idle()
        elif self.currentBotState is BotState.LOOK_TOWARDS:
            self.look_towards(self.bot_pose)
        elif self.currentBotState is BotState.GOAL_SEEK:
            self.goal_seek()
        elif self.currentBotState is BotState.WALL_FOLLOW:
            self.wall_follow()            



    def init(self):
        
        self.bot_motion_pub = self.create_publisher(Twist,"/cmd_vel", qos_profile=10)
        # self.homing_signal_sub = self.create_subscription(PoseStamped,'/homing_signal', self.callback, qos_profile=10)
        self.base_scan_sub = self.create_subscription(LaserScan, '/base_scan', self.process_sensor_info, qos_profile=10)
        self.ground_truth_sub = self.create_subscription(Odometry, '/ground_truth', self.get_base_truth, qos_profile=10)
        self.beacon_pose = self.waypoints[self.waypoint_index]
        self.get_logger().info("Subscription made")
        

    


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()

if __name__ == "__main__":
    main()