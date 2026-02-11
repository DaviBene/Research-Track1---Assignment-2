import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from sensor_msgs.msg import LaserScan
from interfaces.msg import Threshold
from interfaces.srv import ThresholdSrv


# pylint: disable=missing-class-docstring,missing-function-docstring

class MoovementNode(Node):

    def __init__(self):
        super().__init__('MoovementNode')
        self.counter_ = 0
        self.v_linear_x_ = 0.0
        self.v_linear_y_ = 0.0
        self.v_angular_z_ = 0.0
        self.actual_pose_x_ = 0.0
        self.actual_pose_y_ = 0.0
        self.actual_orientation_z_ = 0.0        #quaternion
        self.actual_orientation_w_ = 0.0        #quaternion
        self.safe_pose_x_ = 0.0
        self.safe_pose_y_ = 0.0
        self.safe_orientation_z_ = 0.0        #quaternion
        self.safe_orientation_w_ = 0.0        #quaternion 
        self.ready_ = 0
        self.threshold_ = 1.0    
        self.tooClose_ = 0

        self.cmdvel_publisher_ = self.create_publisher(Twist,"/cmd_vel",10)
        self.cmdvel_timer_ = self.create_timer(0.1,self.timer_callback_cmdvel)

        self.odom_subscriber_ = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)

        self.safePosition_publisher_ = self.create_publisher(PoseWithCovariance,"safePosition",10)
        self.safePosition_timer_ = self.create_timer(0.1,self.timer_callback_safePosition)
        self.safePosition_subscriber_ = self.create_subscription(PoseWithCovariance, "safePosition", self.callback_safePosition, 10)

        self.scan_subscriber_ = self.create_subscription(LaserScan, "/scan", self.callback_scan, 10)
        
        self.threshold_servive = self.create_service(ThresholdSrv, "reset_threshold", self.callback_reset_threshold)
        self.threshold_publisher_ = self.create_publisher(Threshold,"threshold",10)
        self.threshold_timer_ = self.create_timer(0.1,self.timer_callback_threshold)

    def callback_reset_threshold(self, request: ThresholdSrv.Request, response: ThresholdSrv.Response):
        self.threshold_ = request.value.threshold
        response.success = True
        response.message = "Success"
        return response
    
    def timer_callback_threshold(self):
        msg = Threshold()
        msg.threshold = self.threshold_
        self.threshold_publisher_.publish(msg)
    
    def callback_odom(self, msg: Odometry):
        self.actual_pose_x_ = msg.pose.pose.position.x
        self.actual_pose_y_ = msg.pose.pose.position.y
        self.actual_orientation_z_ = msg.pose.pose.orientation.z
        self.actual_orientation_w_ = msg.pose.pose.orientation.w

    def timer_callback_safePosition(self):
        if(self.ready_ == 0):
            msg = PoseWithCovariance()
            msg.pose.position.x = self.actual_pose_x_
            msg.pose.position.y = self.actual_pose_y_
            msg.pose.orientation.z = self.actual_orientation_z_
            msg.pose.orientation.w = self.actual_orientation_w_
            self.safePosition_publisher_.publish(msg)

    def callback_safePosition(self, msg: PoseWithCovariance):
        self.safe_pose_x_ = msg.pose.position.x
        self.safe_pose_y_ =  msg.pose.position.y
        self.safe_orientation_z_ = msg.pose.orientation.z
        self.safe_orientation_w_ = msg.pose.orientation.w

    def callback_scan(self, msg: LaserScan):
        for i in msg.ranges:
            if(i < self.threshold_):
                self.tooClose_ = 1

    def timer_callback_cmdvel(self):
        if(self.ready_ == 1):
            msg = Twist()
            if(self.counter_ != 51):
                if(self.tooClose_ == 0):
                    msg.linear.x = self.v_linear_x_
                    msg.linear.y = self.v_linear_y_
                    msg.angular.z = self.v_angular_z_
                    self.counter_ = self.counter_ + 1
                    self.cmdvel_publisher_.publish(msg)
                    self.get_logger().info('Publishing')
                else:
                    while( (abs(self.actual_pose_x_ - self.safe_pose_x_) > 0.1) and (abs(self.actual_pose_y_ - self.safe_pose_y_) > 0.1) and (abs(self.actual_orientation_z_ - self.safe_orientation_z_) > 0.1) and (abs(self.actual_orientation_w_ - self.safe_orientation_w_) > 0.1)):
                        msg.linear.x = -self.v_linear_x_
                        self.get_logger().info('sono qui')
                        msg.linear.y = -self.v_linear_y_
                        msg.angular.z = -self.v_angular_z_
                        self.cmdvel_publisher_.publish(msg)
                        self.get_logger().info('Publishing to safe position')
                    self.counter_ = 0
                    self.ready_ = 0
                    self.tooClose_ = 0
                    self.get_logger().info('Robot was moved to previous position because too close to obstacle')

            if(self.counter_ == 51):
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = 0.0
                self.cmdvel_publisher_.publish(msg)
                self.counter_ = 0
                self.ready_ = 0
                self.get_logger().info('End publication')

def main(args=None):
    rclpy.init(args=args)
    Moovement_Node = MoovementNode()
    spin_thread = threading.Thread(target=rclpy.spin,args=(Moovement_Node,),daemon=True)
    spin_thread.start()
    while rclpy.ok():
        Moovement_Node.v_linear_x_ = float(input("Choose the linear velocity around x axis: "))
        Moovement_Node.v_linear_y_ = float(input("Choose the linear velocity around y axis: "))
        Moovement_Node.v_angular_z_ = float(input("Choose the angular velocity around z axis: "))
        Moovement_Node.ready_ = 1
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    