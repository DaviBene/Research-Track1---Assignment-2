import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# pylint: disable=missing-class-docstring,missing-function-docstring

class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('VelocityPublisher')
        self.counter_ = 0
        self.turtle_ = 0
        self.v_linear_x_ = 0.0
        self.v_linear_y_ = 0.0
        self.v_angular_z_ = 0.0
        self.ready_ = 0
        self.distance_ = 0.0
        self.treshold_ = 0.2     #set by programmer

        self.publisher1_ = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.publisher2_ = self.create_publisher(Twist, "/turtle2/cmd_vel",10)
        self.timer_ = self.create_timer(0.1,self.timer_callback)
        self.distance_subscriber_ = self.create_subscription(Float32, "distance", self.callback_distance, 10)

    def callback_distance(self, msg: Float32):
        self.distance_ = msg.data
        #self.get_logger().info('Distance updated')

    def timer_callback(self):
        if(self.ready_ == 1):
            if(self.counter_ != 11):
                if(self.turtle_ == 1):
                    if(self.distance_ > self.treshold_):
                        msg = Twist()
                        msg.linear.x = self.v_linear_x_
                        msg.linear.y = self.v_linear_y_
                        msg.angular.z = self.v_angular_z_
                        self.counter_ = self.counter_ + 1
                        self.publisher1_.publish(msg)
                        self.get_logger().info('Publishing')
                    else:
                        msg = Twist()
                        msg.linear.x = 0.0
                        msg.linear.y = 0.0
                        msg.angular.z = 0.0
                        self.counter_ = 0
                        self.ready_ = 0
                        self.publisher1_.publish(msg)
                        self.get_logger().info('Turtle 1 stopped because too close to Turtle 2')

                if(self.turtle_ == 2):
                    if(self.distance_ > self.treshold_):
                        msg = Twist()
                        msg.linear.x = self.v_linear_x_
                        msg.linear.y = self.v_linear_y_
                        msg.angular.z = self.v_angular_z_
                        self.publisher2_.publish(msg)
                        self.counter_ = self.counter_ + 1
                        self.get_logger().info('Publishing')
                    else:
                        msg = Twist()
                        msg.linear.x = 0.0
                        msg.linear.y = 0.0
                        msg.angular.z = 0.0
                        self.counter_ = 0
                        self.ready_ = 0
                        self.publisher2_.publish(msg)
                        self.get_logger().info('Turtle 2 stopped because too close to Turtle 1')

            if(self.counter_ == 11):
                self.counter_ = 0
                self.turtle_ = 0
                self.ready_ = 0
                self.get_logger().info('End publication')
    
def main(args=None):
    rclpy.init(args=args)
    Velocity_Publisher = VelocityPublisher()
    spin_thread = threading.Thread(target=rclpy.spin,args=(Velocity_Publisher,),daemon=True)
    spin_thread.start()
    while rclpy.ok():
        Velocity_Publisher.turtle_ = int(input("Choose the turtle to control: "))
        Velocity_Publisher.v_linear_x_ = float(input("Choose the linear velocity around x axis: "))
        Velocity_Publisher.v_linear_y_ = float(input("Choose the linear velocity around y axis: "))
        Velocity_Publisher.v_angular_z_ = float(input("Choose the angular velocity around z axis: "))
        Velocity_Publisher.ready_ = 1
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
