HOW To:
    1) Ensure Polyscope is active with: 
        ros2 run ur_client_library start_ursim.sh -m ur3e
        Note: Ensure it is powered on
    
    2) Run launch file: 
        ros2 launch motion motion.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 use_fake_hardware:=false use_sim_time:=false launch_rviz:=true

    3) Load external control program to Polyscope and press play.

    4) publish joint poses to command line (simulating messages sent from linux)









NOTE HOST IP IS: 192.168.56.1

To start PolyScope simulation: 
    ros2 run ur_client_library start_ursim.sh -m ur3e
    
    ros2 run ur_client_library start_ursim.sh \
        -m ur3e \
        -f "-p 5900:5900 -p 6080:6080 -p 30001-30004:30001-30004 -p 29999:29999"

    Change polyscope port to 30002

Starting the driver
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true
    
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
    ros2 launch motion motion.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true use_sim_time:=true