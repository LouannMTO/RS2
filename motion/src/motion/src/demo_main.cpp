#include "testmovement.h"
#include <thread>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto test_motion_node = std::make_shared<TestMotion>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(test_motion_node);

    // Spin in background so callbacks work during init
    std::thread spin_thread([&executor]() { executor.spin(); });

    // Wait for MoveIt and controllers to be ready
    std::this_thread::sleep_for(std::chrono::seconds(3));
    test_motion_node->initMoveIt();

    // Run the demo
    RCLCPP_INFO(test_motion_node->get_logger(), "Starting demo movement...");
    test_motion_node->demoMovement();
    RCLCPP_INFO(test_motion_node->get_logger(), "Demo movement complete.");

    spin_thread.join();
    rclcpp::shutdown();

}