import rclpy
import threading
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from sensor_msgs.msg import LaserScan
from interfaces.msg import Threshold
from interfaces.msg import Radar
from interfaces.srv import ThresholdSrv

# pylint: disable=missing-class-docstring,missing-function-docstring

class RadarNode(Node):

    def __init__(self):
        super().__init__('RadarNode')
        self.back_ranges_ = []
        self.right_ranges_ = []
        self.front_ranges_ = []
        self.left_ranges_ = []
        self.threshold_ = 1.0
        self.radarOk_ = 0

        self.radar_publisher_ = self.create_publisher(Radar,"radar",10)
        self.radar_timer_ = self.create_timer(0.1,self.timer_callback_radar)

        self.scan_subscriber_ = self.create_subscription(LaserScan, "/scan", self.callback_scan, 10)

        self.threshold_subscriber_ = self.create_subscription(Threshold, "threshold", self.callback_threshold, 10)

    def callback_threshold(self, msg: Threshold):
        self.threshold_ = msg.threshold

    def callback_scan(self, msg: LaserScan):

        #define back direction
        back_angle = math.radians(45)
        back_increment = int(back_angle/msg.angle_increment)
        back_right_ranges = list(msg.ranges[0:back_increment])

        last_index = int((msg.angle_max - msg.angle_max) / msg.angle_increment)
        start_index_back_left = int(last_index - back_increment)
        back_left_ranges = list(msg.ranges[start_index_back_left:last_index])

        self.back_ranges_ = back_left_ranges + back_right_ranges
        self.get_logger().info('back ranges calculated: ' + str(self.back_ranges_))

        #define right direction
        right_angle = math.radians(90)
        right_increment = int(right_angle/msg.angle_increment) + back_increment
        self.right_ranges_ = list(msg.ranges[(back_increment + 1):right_increment])
        self.get_logger().info('right ranges calculated')

        #define front direction
        front_angle = math.radians(90)
        front_increment = int(front_angle/msg.angle_increment) + right_increment
        self.front_ranges_ = list(msg.ranges[(right_increment + 1):front_increment])
        self.get_logger().info('front ranges calculated')

        #define left direction
        left_angle = math.radians(90)
        left_increment = int(left_angle/msg.angle_increment) + front_increment
        self.left_ranges_ = list(msg.ranges[(front_increment + 1):left_increment])
        self.get_logger().info('left ranges calculated')

        self.radarOk_ = 1

    def timer_callback_radar(self):
        if (self.radarOk_ == 1):
            msg = Radar()
            #define closest distance
            min_back = min(self.back_ranges_)
            min_right = min(self.right_ranges_)
            min_front = min(self.front_ranges_)
            min_left = min(self.left_ranges_)
            min_range = [min_back, min_right, min_front, min_left]
            min_distance = min(min_range)
            msg.distance = min_distance

            #define direction
            if(min_distance == min_back):
                msg.direction = "Back"
            elif(min_distance == min_right):
                msg.direction = "Right"
            elif(min_distance == min_front):
                msg.direction = "Front"
            elif(min_distance == min_left):
                msg.direction = "Left"

            #define threshold
            msg.threshold.threshold = self.threshold_

            self.radar_publisher_.publish(msg)
            self.radarOk_ = 0

def main(args=None):
    rclpy.init(args=args)
    Radar_Node = RadarNode()
    rclpy.spin(Radar_Node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
