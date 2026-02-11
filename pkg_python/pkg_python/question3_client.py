import rclpy
import threading
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from math import sqrt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from sensor_msgs.msg import LaserScan
from interfaces.msg import Threshold
from interfaces.srv import ThresholdSrv

# pylint: disable=missing-class-docstring,missing-function-docstring

class DistancePublisher(Node):
    def __init__(self):
        super().__init__('DistancePublisher')
        self.x_pos1_ = 0.0
        self.y_pos1_ = 0.0
        self.x_pos2_ = 0.0
        self.y_pos2_ = 0.0
        self.threshold_request_ = 0.0
        self.sign_request_ = ""

        self.pose1_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose1, 10)
        self.pose2_subscriber_ = self.create_subscription(Pose, "/turtle2/pose", self.callback_pose2, 10)
        self.distance_publisher_ = self.create_publisher(Float32, "distance", 10)
        self.distance_timer_ = self.create_timer(0.1, self.callback_distance)

        self.threshold_client_ = self.create_client(ThresholdSrv, "reset_threshold")

    def call_reset_threshold(self, value, string):
        while not self.threshold_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service")
        request = ThresholdSrv.Request()
        request.threshold = value
        request.sign = string
        future = self.threshold_client_.call_async(request)
        future.add_done_callback(self.callback_reset_threshold_response)

    def callback_reset_threshold_response(self, future):
        response = future.result()
        self.get_logger().info("Success flag:" +str(response.success))

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
    spin_thread = threading.Thread(target=rclpy.spin,args=(Distance_Publisher,),daemon=True)
    spin_thread.start()
    while rclpy.ok():
        Distance_Publisher.threshold_request_ = float(input("Do you want to change the treshold distance? Insert a value: "))
        Distance_Publisher.sign_request_ = str(input("Define increase or decrease: "))
        Distance_Publisher.call_reset_threshold(Distance_Publisher.threshold_request_, Distance_Publisher.sign_request_)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        