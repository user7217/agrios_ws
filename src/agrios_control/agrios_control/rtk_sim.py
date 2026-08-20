import math
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


def yaw_from_quat(q):
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(s, c)


def quat_from_yaw(y):
    return (0.0, 0.0, math.sin(y / 2.0), math.cos(y / 2.0)) 


class RtkSim(Node):
    """Degrades Gazebo's perfect NavSat + ground truth into ArduSimple-like RTK."""

    def __init__(self):
        super().__init__('rtk_sim')

        # ---- noise model (ZED-F9P, RTK fixed) ----
        self.declare_parameter('pos_sigma_h', 0.015)      # m, horizontal
        self.declare_parameter('pos_sigma_v', 0.030)      # m, vertical
        self.declare_parameter('heading_sigma_deg', 0.20) # deg, 1m baseline
        self.declare_parameter('rate_hz', 8.0)

        self.sh = self.get_parameter('pos_sigma_h').value
        self.sv = self.get_parameter('pos_sigma_v').value
        self.syaw = math.radians(self.get_parameter('heading_sigma_deg').value)
        rate = self.get_parameter('rate_hz').value

        self.last_fix = None
        self.last_gt = None

        self.create_subscription(NavSatFix, '/navsat', self._fix_cb, 10)
        self.create_subscription(Odometry, '/ground_truth', self._gt_cb, 10)

        self.fix_pub = self.create_publisher(NavSatFix, '/rtk/fix', 10)
        self.hdg_pub = self.create_publisher(Imu, '/rtk/heading', 10)

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'rtk_sim: {rate} Hz, '
                               f'{self.sh*100:.1f} cm pos, '
                               f'{math.degrees(self.syaw):.2f} deg heading')

    def _fix_cb(self, msg):
        self.last_fix = msg

    def _gt_cb(self, msg):
        self.last_gt = msg

    def _tick(self):
        if self.last_fix is None or self.last_gt is None:
            return

        now = self.get_clock().now().to_msg()

        # ---- position: perfect lat/lon + Gaussian noise ----
        fix = NavSatFix()
        fix.header.stamp = now
        fix.header.frame_id = 'gps_link'
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX   # RTK fixed
        fix.status.service = NavSatStatus.SERVICE_GPS

        # metres -> degrees, small-angle
        lat0 = self.last_fix.latitude
        dlat = random.gauss(0.0, self.sh) / 111320.0
        dlon = random.gauss(0.0, self.sh) / (111320.0 * math.cos(math.radians(lat0)))

        fix.latitude = lat0 + dlat
        fix.longitude = self.last_fix.longitude + dlon
        fix.altitude = self.last_fix.altitude + random.gauss(0.0, self.sv)

        fix.position_covariance = [
            self.sh**2, 0.0,        0.0,
            0.0,        self.sh**2, 0.0,
            0.0,        0.0,        self.sv**2,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix)

        # ---- heading: moving-baseline, absolute, no drift ----
        yaw = yaw_from_quat(self.last_gt.pose.pose.orientation)
        yaw += random.gauss(0.0, self.syaw)

        hdg = Imu()
        hdg.header.stamp = now
        hdg.header.frame_id = 'gps_link'
        x, y, z, w = quat_from_yaw(yaw)
        hdg.orientation.x = x
        hdg.orientation.y = y
        hdg.orientation.z = z
        hdg.orientation.w = w
        hdg.orientation_covariance = [
            1e6, 0.0,        0.0,     # roll: not measured
            0.0, 1e6,        0.0,     # pitch: not measured
            0.0, 0.0, self.syaw**2,   # yaw
        ]
        hdg.angular_velocity_covariance[0] = -1.0     # no gyro
        hdg.linear_acceleration_covariance[0] = -1.0  # no accel
        self.hdg_pub.publish(hdg)


def main():
    rclpy.init()
    node = RtkSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()