#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from kinematics_red import fk


class FKNode(Node):

    def __init__(self):
        super().__init__('fk_node_red')
        self.pub = self.create_publisher(Point, '/fk_position', 10)
        self.create_subscription(Float64MultiArray, '/joint_angles', self.cb, 10)
        self.last = None

    def cb(self, msg):
        X = fk(np.array(msg.data, float))
        self.pub.publish(Point(x=float(X[0]), y=float(X[1]), z=float(X[2])))
        key = tuple(np.round(msg.data, 4))
        if key != self.last:
            self.last = key
            self.get_logger().info(f'q = {np.round(np.degrees(msg.data), 2)} deg -> '
                                   f'FK X = {np.round(X, 4)} m')


def main():
    rclpy.init()
    node = FKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
