HOW To:
    1) Ensure Polyscope is active with: 
        ros2 run ur_client_library start_ursim.sh -m ur3e
        Note: Ensure it is powered on
    
    2) Run launch file: 
        ros2 launch motion motion.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 use_fake_hardware:=false use_sim_time:=false launch_rviz:=true
        ros2 launch motion motion.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 use_fake_hardware:=false use_sim_time:=false launch_rviz:=true
        ros2 launch motion motion.launch.py ur_type:=ur3e robot_ip:=192.168.0.191 use_fake_hardware:=false use_sim_time:=false launch_rviz:=true



    3) Load external control program to Polyscope and press play.

    4) publish joint poses to command line (simulating messages sent from linux)











NOTE HOST IP IS: 192.168.56.1

To start PolyScope simulation: 
    ros2 run ur_client_library start_ursim.sh -m ur3e

Starting the driver
    // for sim
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true
    // for real robot
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 launch_rviz:=true
    
    ros2 launch ur_robot_driver ur_control.launch.py \
        ur_type:=ur3e \
        robot_ip:=192.168.56.101 \
        launch_rviz:=true

    Run driver without scaled velocity (no error)
    ros2 launch ur_robot_driver ur_control.launch.py \
        ur_type:=ur3e \
        robot_ip:=192.168.56.101 \
        use_fake_hardware:=false \
        launch_rviz:=false \
        initial_joint_controller:=joint_trajectory_controller

To use launch file
    // for sim
    ros2 launch motion motion.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true use_sim_time:=true
    // for real robot
    ros2 launch motion motion.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 launch_rviz:=true use_sim_time:=true


Commands for ros2
    ros2 topic pub /g8_ee_pose geometry_msgs/msg/PoseStamped "{
        header: {frame_id: 'base_link'},
        pose: {
            position: {x: 0.4, y: 0.0, z: 0.4},
            orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        }
    }"

    ros2 topic pub /g8_joint_states sensor_msgs/msg/JointState "{
        position: [0.0, -1.2, 1.2, -1.5, 1.57, 0.0]
    }"


    ros2 control list_controllers
    ros2 control switch_controllers --activate scaled_joint_trajectory_controller

    ros2 control switch_controllers \
  --activate scaled_joint_trajectory_controller \
  --deactivate joint_trajectory_controller

#works with gripper
  ros2 launch motion motion.launch.py ur_type:=ur3e onrobot_type:=rg2 use_fake_hardware:=true robot_ip:=192.168.56.101