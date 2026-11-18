#!/usr/bin/env python3
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from kinematics_red import jacobian_numeric, inverse_velocity_kinematics
from robot_io_red import RobotIO

DT = 0.02


class VelNode(Node):

    def __init__(self):
        super().__init__('velocity_node_red')
        p = self.declare_parameter
        self.mode = p('mode', 'forward').value
        self.q = np.radians(np.array(p('q_start_deg', [30.0, 20.0, -15.0, 45.0, 30.0, 60.0]).value, float))
        self.qdot_cmd = np.radians(np.array(p('qdot_deg', [10.0, -5.0, 8.0, 5.0, -6.0, 4.0]).value, float))
        self.v_des = np.array(p('v_desired', [0.02, 0.01, -0.02]).value, float)
        self.T = float(p('duration', 5.0).value)
        self.settle = 4.0
        self.io = RobotIO(self)
        self.pj = self.create_publisher(Vector3, '/v_jacobian', 10)
        self.pg = self.create_publisher(Vector3, '/v_gazebo', 10)
        self.t = 0.0
        self.t_prev = None
        self.next_log = 1.0
        self.errs = []
        self.vj_hist = deque(maxlen=200)
        self.done = False
        self.get_logger().info(f'{self.mode.upper()} velocity test, q_start = '
                               f'{np.round(np.degrees(self.q), 1)} deg')
        self.create_timer(DT, self.step)

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now == 0.0:
            return
        dt = DT if self.t_prev is None else now - self.t_prev
        self.t_prev = now
        if dt <= 0.0:
            return
        self.t += dt
        if self.t < self.settle:
            self.io.send(self.q)
            return
        tm = self.t - self.settle
        if tm > self.T:
            if not self.done:
                self.done = True
                if self.errs:
                    self.get_logger().info(f'DONE. mean |V_J - V_gazebo| = {1000 * np.mean(self.errs):.2f} mm/s, '
                                           f'max = {1000 * np.max(self.errs):.2f} mm/s')
            return
        if self.mode == 'forward':
            qdot = self.qdot_cmd
        else:
            qdot = inverse_velocity_kinematics(self.q, self.v_des)
        q_now = self.io.q_meas if self.io.q_meas is not None else self.q
        vj = jacobian_numeric(q_now) @ qdot
        self.q = self.q + qdot * dt
        self.io.send(self.q)
        vg = self.io.V_meas if self.io.V_meas is not None else np.zeros(3)
        self.vj_hist.append((now, vj))
        t_mid = now - 0.5 * self.io.window
        vj_mid = min(self.vj_hist, key=lambda e: abs(e[0] - t_mid))[1]
        if tm > 0.5 and self.io.V_meas is not None:
            self.errs.append(np.linalg.norm(vj_mid - vg))
        self.pj.publish(Vector3(x=float(vj[0]), y=float(vj[1]), z=float(vj[2])))
        self.pg.publish(Vector3(x=float(vg[0]), y=float(vg[1]), z=float(vg[2])))
        if tm >= self.next_log:
            self.next_log += 1.0
            self.get_logger().info(f't={tm:.1f}s q_dot={np.round(np.degrees(qdot), 2)} deg/s | '
                                   f'V_J={np.round(1000 * vj, 2)} mm/s | V_gazebo={np.round(1000 * vg, 2)} mm/s')


def main():
    rclpy.init()
    node = VelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
