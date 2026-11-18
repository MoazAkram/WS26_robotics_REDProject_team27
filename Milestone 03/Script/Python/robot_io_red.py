from collections import deque

import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from kinematics_red import fk


class RobotIO:

    def __init__(self, node):
        self.n = node
        p = node.declare_parameter
        self.joint_names = list(p('joint_names', ['joint_1', 'joint_2', 'joint_3',
                                                  'joint_4', 'joint_5', 'joint_6']).value)
        self.offsets = np.radians(np.array(p('offsets_deg', [0.0] * 6).value, float))
        self.signs = np.array(p('signs', [1.0, 1.0, -1.0, -1.0, -1.0, -1.0]).value, float)
        gz_topic = p('gazebo_topic', '/position_controller/commands').value
        fk_topic = p('fk_topic', '/joint_angles').value
        self.ee_source = p('ee_source', 'pose').value
        ee_topic = p('ee_topic', '/end_effector_pose').value
        self.ee_link = p('ee_link', 'link_6').value
        self.base = np.array(p('base_offset', [0.0, 0.0, 0.0]).value, float)

        self.gz_pub = node.create_publisher(Float64MultiArray, gz_topic, 10)
        self.fk_pub = node.create_publisher(Float64MultiArray, fk_topic, 10)

        self.q_meas = None
        self.X_meas = None
        self.V_meas = None
        self._hist = deque()
        self.window = float(p('velocity_window', 0.25).value)
        node.create_subscription(JointState, '/joint_states', self._js, 10)
        if self.ee_source == 'pose':
            node.create_subscription(TFMessage, ee_topic, self._pose, 10)
        node.get_logger().info(f'Gazebo cmd: {gz_topic} | FK node: {fk_topic} | '
                               f'end-effector from: {self.ee_source}')

    def to_gazebo(self, q_dh):
        return self.offsets + self.signs * np.asarray(q_dh, float)

    def to_dh(self, q_gz):
        return (np.asarray(q_gz, float) - self.offsets) / self.signs

    def send(self, q_dh):
        self.fk_pub.publish(Float64MultiArray(data=[float(v) for v in q_dh]))
        self.gz_pub.publish(Float64MultiArray(data=[float(v) for v in self.to_gazebo(q_dh)]))

    def _set_pos(self, xyz, t=None):
        X = np.asarray(xyz, float) - self.base
        if t is None:
            t = self.n.get_clock().now().nanoseconds * 1e-9
        self._hist.append((t, X))
        while len(self._hist) > 2 and t - self._hist[1][0] >= self.window:
            self._hist.popleft()
        t0, X0 = self._hist[0]
        if t - t0 >= 0.5 * self.window:
            self.V_meas = (X - X0) / (t - t0)
        self.X_meas = X

    def _js(self, msg):
        try:
            idx = [list(msg.name).index(j) for j in self.joint_names]
        except ValueError:
            return
        self.q_meas = self.to_dh([msg.position[i] for i in idx])
        if self.ee_source == 'joint_states':
            self._set_pos(fk(self.q_meas) + self.base)

    def _pose(self, msg):
        for t in msg.transforms:
            if t.child_frame_id.replace('::', '/').split('/')[-1] == self.ee_link:
                p = t.transform.translation
                stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                self._set_pos([p.x, p.y, p.z], stamp)
