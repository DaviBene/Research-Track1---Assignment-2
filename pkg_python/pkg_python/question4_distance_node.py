import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from math import sqrt
import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance

# pylint: disable=missing-class-docstring,missing-function-docstring

class DistancePublisher(Node):
    def __init__(self):
        super().__init__('DistancePublisher')
        self.x_pos1_ = 0.0
        self.y_pos1_ = 0.0
        self.z_orientation1_ = 0.0
        self.w_orientation1_ = 0.0

        self.x_pos2_ = 0.0
        self.y_pos2_ = 0.0
        self.z_orientation2_ = 0.0
        self.w_orientation2_ = 0.0

        self.pose1_subscriber_ = self.create_subscription(Odometry, "/odom", self.callback_odom1, 10)
        self.pose2_subscriber_ = self.create_subscription(Odometry, "/odom", self.callback_odom2, 10)
        self.distance_publisher_ = self.create_publisher(Float32, "distance", 10)
        self.distance_timer_ = self.create_timer(0.1, self.callback_distance)

    def callback_odom1(self, msg: Odometry):
        self.x_pos1_ = msg.pose.pose.position.x
        self.y_pos1_ = msg.pose.pose.position.y
        self.z_orientation1_ = msg.pose.pose.orientation.z
        self.w_orientation1_ = msg.pose.pose.orientation.w

    def callback_odom2(self, msg: Odometry):
        self.x_pos2_ = msg.pose.pose.position.x
        self.y_pos2_ = msg.pose.pose.position.y
        self.z_orientation2_ = msg.pose.pose.orientation.z
        self.w_orientation2_ = msg.pose.pose.orientation.w

    def callback_distance(self):
        distance = sqrt( (self.x_pos1_ - self.x_pos2_)**2 + (self.y_pos1_ - self.y_pos2_)**2 )
        msg = Float32()
        msg.data = distance
        self.distance_publisher_.publish(msg)
        self.get_logger().info('Distance published: ' +str(distance))

def main(args=None):
    rclpy.init(args=args)
    Distance_Publisher = DistancePublisher()
    rclpy.spin(Distance_Publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
