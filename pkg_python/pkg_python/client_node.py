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

class ClientNode(Node):

    def __init__(self):
        super().__init__('ClientNode')
        self.threshold_request_ = 0.0
        self.threshold_client_ = self.create_client(ThresholdSrv, "reset_threshold")

    def call_reset_threshold(self,value):
        while not self.threshold_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service")
        request = ThresholdSrv.Request()
        request.value.threshold = value
        future = self.threshold_client_.call_async(request)
        future.add_done_callback(self.callback_reset_threshold_response)

    def callback_reset_threshold_response(self, future):
        response = future.result()
        self.get_logger().info("Success flag:" +str(response.success))
        self.get_logger().info("Messagge:" +str(response.message))

def main(args=None):
    rclpy.init(args=args)
    Client_Node = ClientNode()
    spin_thread = threading.Thread(target=rclpy.spin,args=(Client_Node,),daemon=True)
    spin_thread.start()
    while rclpy.ok():
        Client_Node.threshold_request_ = float(input("Do you want to change the treshold distance? Insert a value: "))
        Client_Node.call_reset_threshold(Client_Node.threshold_request_)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()