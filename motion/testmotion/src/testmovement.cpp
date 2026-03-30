#include "testmovement.h"

TestMotion::TestMotion() : Node("test_motion")
{

    
    auto qos = rclcpp::SensorDataQoS(); // This qos must match unity!!

    //JointStates_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_testingggg", qos);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TestMotion::readQvalues, this));

    // Note : SensorDataQoS() is a preset provided by ROS 2, it’s optimized for sensor data like LIDAR.
    subJointStates_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&TestMotion::jointStateCb, this, std::placeholders::_1));

}

TestMotion::~TestMotion(){

}

void TestMotion::initMoveIt()
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ur_manipulator");
}

TestMotion::jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg){
    
    // Do something under lock 
    {
        std::lock_guard<std::mutex> lock(jointstate_mtx_);
        jointValues = msg->position;
    }

}



void TestMotion::moveusingQ(double q1, double q2,double q3,double q4,double q5,double q6){

    if (!move_group_) {     // if MoveIt initialised correctly
        RCLCPP_ERROR(this->get_logger(), "Move group not initialized!");
        return;
    }
    std::vector<double> target = {q1, q2, q3, q4, q5, q6};
    move_group_->setJointValueTarget(target);
    move_group_->move();

}

void TestMotion::readQvalues(void){

    std::lock_guard<std::mutex> lock(jointstate_mtx_);  // Mutex as data may be updated at same time

    if (jointValues.size() >= 6)                        // Ensure all joint values are assigned 
    {
        RCLCPP_INFO(this->get_logger(),
            "Current joints: %.2f %.2f %.2f %.2f %.2f %.2f",
            jointValues[0], jointValues[1], jointValues[2],
            jointValues[3], jointValues[4], jointValues[5]);
    }
}


