import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from math import sqrt

# pylint: disable=missing-class-docstring,missing-function-docstring

class DistancePublisher(Node):
    def __init__(self):
        super().__init__('DistancePublisher')
        self.x_pos1_ = 0.0
        self.y_pos1_ = 0.0
        self.x_pos2_ = 0.0
        self.y_pos2_ = 0.0

        self.pose1_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose1, 10)
        self.pose2_subscriber_ = self.create_subscription(Pose, "/turtle2/pose", self.callback_pose2, 10)
        self.distance_publisher_ = self.create_publisher(Float32, "distance", 10)
        self.distance_timer_ = self.create_timer(0.1, self.callback_distance)

    def callback_pose1(self, msg: Pose):
        self.x_pos1_ = msg.x
        self.y_pos1_ = msg.y
        self.get_logger().info('Pose 1 updated: ' +str(self.x_pos1_) +str(self.y_pos1_))

    def callback_pose2(self, msg: Pose):
        self.x_pos2_ = msg.x
        self.y_pos2_ = msg.y
        self.get_logger().info('Pose 2 updated')

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
        
        


