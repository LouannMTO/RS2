#include "testmovement.h"

int main(int argc, char **argv){

    
    //Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create an instance of generate_pointcloud
    auto test_motion_node = std::make_shared<TestMotion>();
    // Initialize MoveIt after node is fully created
    test_motion_node->initMoveIt();

    // // Create an instance of the Quadcopter node
    // auto quadcopter_node = std::make_shared<Quadcopter>();
    // // Create an instance of MissionNode
    // auto mission_node = std::make_shared<MissionNode>(quadcopter_node);

    // Create an executor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add nodes to the executor
    executor.add_node(test_motion_node);
    //executor.add_node(pcd_read_node);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); //Wait to allow connection to be established

    // Spin the executor
    executor.spin();

    // Shutdown ROS 2
    rclcpp::shutdown();

}

