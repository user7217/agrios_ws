import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomCovRelay(Node):
    def __init__(self):
        super().__init__('odom_cov_relay')
        self.pub = self.create_publisher(Odometry, '/odom_cov', 10)
        self.create_subscription(Odometry, '/odom', self.cb, 10)

    def cb(self, msg):
        c = [0.0] * 36
        c[0]  = 0.05    # vx
        c[7]  = 0.05    # vy
        c[35] = 0.02    # vyaw
        msg.twist.covariance = c
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = OdomCovRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()