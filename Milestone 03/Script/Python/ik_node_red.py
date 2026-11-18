#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from kinematics_red import inverse_jacobian_matrix, inverse_kinematics_func, fk
from robot_io_red import RobotIO


class IKNode(Node):

    def __init__(self):
        super().__init__('ik_node_red')
        p = self.declare_parameter
        self.target = np.array(p('target', [0.4762, 0.3076, 0.6116]).value, float)
        guess = np.radians(np.array(p('q_guess_deg', [10.0, 10.0, 0.0, 0.0, 10.0, 0.0]).value, float))
        self.move_time = float(p('move_time', 3.0).value)
        self.io = RobotIO(self)
        self.q = inverse_kinematics_func(guess, self.target)
        err = np.linalg.norm(fk(self.q) - self.target)
        log = self.get_logger().info
        log(f'Target X                = {np.round(self.target, 4)} m')
        log(f'J_inv at the solution   =\n{np.round(inverse_jacobian_matrix(self.q), 4)}')
        log(f'IK (Newton-Raphson) q   = {np.round(np.degrees(self.q), 2)} deg')
        log(f'FK(q) (Python function) = {np.round(fk(self.q), 4)} m, error {1000 * err:.3f} mm')
        self.t = 0
        self.create_timer(1.0, self.tick)

    def tick(self):
        self.t += 1
        self.io.send(self.q)
        if self.t >= int(self.move_time) + 3:
            if self.io.X_meas is None:
                self.get_logger().warn('No end-effector data yet, check ee_source / ee_topic')
                return
            Xg = self.io.X_meas
            self.get_logger().info(f'Gazebo end-effector = {np.round(Xg, 4)} m, '
                                   f'error vs target {1000 * np.linalg.norm(Xg - self.target):.2f} mm')


def main():
    rclpy.init()
    node = IKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
