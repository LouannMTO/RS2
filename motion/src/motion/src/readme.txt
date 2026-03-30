To start PolyScope: 
    ros2 run ur_client_library start_ursim.sh -m ur3e

To use launch file
    ros2 launch motion ur_moveit.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true use_sim_time:=true