import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import transforms3d

class PoseInit(Node):
    
    def __init__(self):
        super().__init__("Pose_Initializer")

        self.declare_parameter('x', 3.1)
        self.declare_parameter('y', -3.2)
        self.declare_parameter('theta', 0.0)
        self.declare_parameter('cov', 0.5**2)

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10)
        
        while(self.publisher_.get_subscription_count() == 0):
            self.get_logger().debug("Waiting for AMCL to start")
            time.sleep(1)

        self.pose_publisher()
        
    def pose_publisher(self):

        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        theta = self.get_parameter('theta').value
        cov = self.get_parameter('cov').value

        quat = transforms3d.euler.euler2quat(0,0,theta)
        
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y

        msg.pose.pose.orientation.w = quat[0]
        msg.pose.pose.orientation.x = quat[1]
        msg.pose.pose.orientation.y = quat[2]
        msg.pose.pose.orientation.z = quat[3]

        msg.pose.covariance = [
            cov , 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0 , cov, 0.0, 0.0, 0.0, 0.0,
            0.0 , 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0 , 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0 , 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0 , 0.0, 0.0, 0.0, 0.0, cov
        ]

        self.publisher_.publish(msg)

def main(args=None):

    rclpy.init(args=args)
    Node1 = PoseInit()
    rclpy.spin(Node1)
    rclpy.shutdown()
    pass

if __name__=="__main__":
    main()