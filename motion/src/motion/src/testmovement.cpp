#include "testmovement.h"

TestMotion::TestMotion() : Node("test_motion")
{

    
    // This qos must match unity!!
    auto qos = rclcpp::QoS(10);     // history depth 10
    qos.best_effort();              // set reliability to Best-Effort

    //JointStates_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_testingggg", qos);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TestMotion::readQvalues, this));

    
    // Subscriber for current joint poses of the robot
    subJointStates_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),                                     // Note : SensorDataQoS() is a preset provided by ROS 2, it’s optimized for sensor data like LIDAR.
      std::bind(&TestMotion::jointStateCb, this, std::placeholders::_1));

    // Subscriber for target joint poses of the robot
    subTargetJointStates_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/target_joint_states", qos,
      std::bind(&TestMotion::targetJointStateCb, this, std::placeholders::_1));

    // Subscriber for target end-effector pose
    subTargetEEPose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_ee_pose", qos,
        std::bind(&TestMotion::targetEEPoseCb, this, std::placeholders::_1));

    // this->declare_parameter("robot_description", std::string());
    // this->declare_parameter("robot_description_semantic", std::string());

}

TestMotion::~TestMotion(){

}

void TestMotion::initMoveIt()
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ur_manipulator");

    move_group_->setMaxVelocityScalingFactor(0.1);   // 10% speed
    move_group_->setMaxAccelerationScalingFactor(0.1);
}

void TestMotion::jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg){
    
    // Do something under lock 
    {
        std::lock_guard<std::mutex> lock(jointstate_mtx_);
        jointValues = msg->position;    // Store current robot joint states locally
    }

}

void TestMotion::targetJointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg){
    
    if (msg->position.size() < 6) {
        RCLCPP_WARN(this->get_logger(), "Received joint positions with less than 6 joints");
        return;
    }
    // Move the robot to target joint positions
    moveusingQ(msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5]);

}

void TestMotion::targetEEPoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    
    if (!move_group_) {     // Check if MoveIt is initialised
        RCLCPP_ERROR(this->get_logger(), "Move group not initialized!");
        return;
    }

    geometry_msgs::msg::Pose target_pose = msg->pose;
    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;  // declare plan object

    // This should check whether path can be planned before moving
    bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        move_group_->execute(my_plan);
    } else {
        RCLCPP_WARN(this->get_logger(), "Planning to target pose failed.");
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


