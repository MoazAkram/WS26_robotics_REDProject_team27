MILESTONE 3 - PYTHON / GAZEBO PART - FANUC LR Mate 200iD (Team 27)
ROS 2 Jazzy + Gazebo Harmonic (Ubuntu 24.04 / Docker container ros2_jazzy)
Gazebo runs with the Bullet physics engine (set in the launch file). With the default DART engine joint 1 stops following commands in some poses.

lrmate200id_description/   robot package: URDF + meshes + ros2_control + launch
python/kinematics_red.py   forward_kinematics_func, jacobian_matrix, inverse_jacobian_matrix,
                           inverse_kinematics_func, forward_velocity_kinematics, inverse_velocity_kinematics
python/fk_node_red.py      FK node: subscribes /joint_angles, publishes /fk_position
python/ik_node_red.py      IK node: target X -> Newton-Raphson q -> /joint_angles (FK node) + Gazebo
python/velocity_node_red.py  forward / inverse velocity: V_J (Jacobian) vs V_gazebo
python/robot_io_red.py     sends angles to Gazebo + FK node, reads the flange pose from Gazebo
python/params_red.yaml     topic names and joint signs

Topics
  /position_controller/commands   Float64MultiArray, 6 joint angles (rad)
  /joint_states
  /end_effector_pose              tf2_msgs/TFMessage, link poses from Gazebo (link_6 = flange)

1) Install (once, inside the container)
   apt update
   apt install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro python3-sympy python3-numpy

2) Build
   source /opt/ros/jazzy/setup.bash
   cd ~/ros2_ws && colcon build --packages-select lrmate200id_description && source install/setup.bash

3) Launch Gazebo (terminal 1)
   ros2 launch lrmate200id_description gazebo.launch.py

4) Maths check (no ROS)
   cd ~/RED_M3/python && python3 kinematics_red.py

5) IK test (terminal 2: FK node, terminal 3: IK node, one case at a time)
   python3 fk_node_red.py --ros-args -p use_sim_time:=true
   python3 ik_node_red.py --ros-args --params-file params_red.yaml -p target:="[0.4762,0.3076,0.6116]" -p q_guess_deg:="[10.0,10.0,0.0,0.0,10.0,0.0]"
   python3 ik_node_red.py --ros-args --params-file params_red.yaml -p target:="[0.4060,-0.4060,0.4273]" -p q_guess_deg:="[-30.0,20.0,0.0,0.0,-20.0,0.0]"
   python3 ik_node_red.py --ros-args --params-file params_red.yaml -p target:="[0.2124,0.3114,0.5732]" -p q_guess_deg:="[45.0,0.0,10.0,0.0,30.0,0.0]"

6) Velocity test
   python3 velocity_node_red.py --ros-args --params-file params_red.yaml -p mode:=forward
   python3 velocity_node_red.py --ros-args --params-file params_red.yaml -p mode:=inverse

Results from a test run (Gazebo Harmonic, ROS 2 Jazzy)
   IK cases 1-3: Gazebo end-effector error vs target 0.00 mm
   Velocity forward: mean |V_J - V_gazebo| about 0.7-0.9 mm/s
   Velocity inverse: mean |V_J - V_gazebo| about 0.5 mm/s
