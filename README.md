# WS26 Robotics RED Project, Team 27

EDPT1009 Robotics, GUC, Winter 2026

Robot: FANUC LR Mate 200iD
Application: Bottle packaging

| Folder | Contents |
|---|---|
| `Milestone 01/` | DH convention and forward position kinematics |
| `Milestone 01/Script/MATLAB` | `transformation_func.m`, `forward_kinematics_func.m`, `fk_xyz.m`, `run_kinematics.m`, `build_simulink_model.m` |
| `Milestone 01/Script/Python` | `forward_kinematics.py`, `fk_gazebo_node.py` (ROS 2 Jazzy) |
| `Milestone 01/Script/Simulink` | Simulink and Simscape `.slx` models |
| `Milestone 01/Script/XML` | Simscape Multibody export from SolidWorks |
| `Milestone 01/Script/URDF` | URDF export and meshes |
| `Milestone 01/Script/STL` | STL files (only if the design is modified) |
| `Milestone 01/Media` | `Video1.mp4` (Simscape + MATLAB), `Video2.mp4` (Gazebo + Python) |
| `Milestone 03/` | Inverse position kinematics (Newton-Raphson), forward and inverse velocity kinematics |
| `Milestone 03/Script/Python` | `kinematics_red.py`, `fk_node_red.py`, `ik_node_red.py`, `velocity_node_red.py`, `robot_io_red.py`, `params_red.yaml` (ROS 2 Jazzy) |
| `Milestone 03/Script/URDF` | `lrmate200id_description` package: URDF, meshes, ros2_control, Gazebo Harmonic launch |
