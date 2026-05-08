#include "testmovement.h"

TestMotion::TestMotion() : Node("test_motion")
{
    
    // This qos must match unity!!
    auto qos = rclcpp::QoS(10);     // history depth 10
    qos.best_effort();              // set reliability to Best-Effort

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&TestMotion::publishJoints, this));

    
    // Subscriber for current joint poses of the robot
    subJointStates_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),                                     // Note : SensorDataQoS() is a preset provided by ROS 2, it’s optimized for sensor data like LIDAR.
      std::bind(&TestMotion::jointStateCb, this, std::placeholders::_1));

    // Subscriber for target joint poses of the robot
    subTargetJointStates_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/g8_joint_states", qos,
      std::bind(&TestMotion::targetJointStateCb, this, std::placeholders::_1));

    // Subscriber for target end-effector pose
    subTargetEEPose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/g8_ee_pose", qos,
        std::bind(&TestMotion::targetEEPoseCb, this, std::placeholders::_1));

    // Publish joint states back to Unity
    jointStatesToUnityPub_ = this->create_publisher<sensor_msgs::msg::JointState>("/g8_target_joint_states", qos);

    // Simulating publishing from Unity
    jointstatesPub_ = this->create_publisher<sensor_msgs::msg::JointState>("/target_joint_states", qos);

    last_ee_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(); // initialise
}

TestMotion::~TestMotion(){
    if (motion_thread_.joinable()) {
        motion_thread_.join();
    }
}

void TestMotion::initMoveIt()
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "ur_onrobot_manipulator");

    move_group_->setMaxVelocityScalingFactor(0.1);   // 10% speed
    move_group_->setMaxAccelerationScalingFactor(0.1);
    // Add these to initMoveIt() to give more tolerance
    move_group_->setGoalPositionTolerance(0.01);      // 1cm
    move_group_->setGoalOrientationTolerance(0.01);   // ~0.57 degrees
    move_group_->setGoalJointTolerance(0.01);
}

void TestMotion::jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg){
    
    // Do something under lock 
    {
        std::lock_guard<std::mutex> lock(jointstate_mtx_);
        jointValues = msg->position;    // Store current robot joint states locally
    }

}

// void TestMotion::targetJointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg){
    
//     if (!move_group_) {     // Check if MoveIt is initialised
//         RCLCPP_ERROR(this->get_logger(), "Move group not initialized!");
//         return;
//     }
    
//     if (msg->position.size() < 6) {
//         RCLCPP_WARN(this->get_logger(), "Received joint positions with less than 6 joints");
//         return;
//     }
//     // Move the robot to target joint positions
//     moveusingQ(msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5]);

// }

void TestMotion::targetJointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg) {

    if (!move_group_) { 
        RCLCPP_ERROR(this->get_logger(), "Move group not initialized!"); 
        return; 
    }
    if (msg->position.size() < 6) { 
        RCLCPP_WARN(this->get_logger(), "Received joint positions with less than 6 joints"); 
        return; 
    }

    // Drop immediately if still moving
    if (motion_busy_.load()) return;

    // return if another thread just claimed it
    if (motion_busy_.exchange(true)) return;

    // Safe to join now, busy was false so thread is finished
    if (motion_thread_.joinable()) motion_thread_.join();

    auto positions = msg->position;
    motion_thread_ = std::thread([this, positions]() {
        if (!move_group_) { 
            motion_busy_ = false; 
            return; 
        }

        std::vector<double> target = {
            positions[0], positions[1], positions[2],
            positions[3], positions[4], positions[5]
        };

        move_group_->setStartStateToCurrentState();
        move_group_->setJointValueTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) ==
            moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group_->execute(my_plan);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        } else {
            RCLCPP_WARN(this->get_logger(), "Planning to joint target failed.");
        }
        motion_busy_ = false;
    });

    // if (!move_group_) { 
    //     RCLCPP_ERROR(this->get_logger(), "Move group not initialized!"); 
    //     return; 
    // }
    // if (msg->position.size() < 6) { 
    //     RCLCPP_WARN(this->get_logger(), "Received joint positions with less than 6 joints"); 
    //     return; 
    // }
    // if (motion_busy_.exchange(true)) return; // drop if already moving

    // if (motion_thread_.joinable()) motion_thread_.join();
    // auto positions = msg->position;
    // motion_thread_ = std::thread([this, positions]() {
    //     moveusingQ(positions[0], positions[1], positions[2],
    //                positions[3], positions[4], positions[5]);
    //     motion_busy_ = false;
    // });
}

// void TestMotion::targetEEPoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    
//     if (!move_group_) {     // Check if MoveIt is initialised
//         RCLCPP_ERROR(this->get_logger(), "Move group not initialized!");
//         return;
//     }

//     geometry_msgs::msg::Pose target_pose = msg->pose;
//     move_group_->setPoseTarget(target_pose);

//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;  // declare plan object

//     // This should check whether path can be planned before moving
//     bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     if (success) {
//         move_group_->execute(my_plan);
//     } else {
//         RCLCPP_WARN(this->get_logger(), "Planning to target pose failed.");
//     }

// }

double TestMotion::normalizeAngle(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void TestMotion::targetEEPoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
//     if (!move_group_) {
//         RCLCPP_ERROR(this->get_logger(), "Move group not initialized!");
//         return;
//     }


//     // Drop immediately if still moving — don't even check the pose
//     if (motion_busy_.load()) return;

//     if (last_ee_pose_) {
//         const auto &a = last_ee_pose_->pose;
//         const auto &b = msg->pose;

//         auto close = [](double a, double b, double tol = 1e-2) {
//             return std::abs(a - b) < tol;
//         };

//         bool same =
//             close(a.position.x, b.position.x) &&
//             close(a.position.y, b.position.y) &&
//             close(a.position.z, b.position.z) &&
//             close(a.orientation.x, b.orientation.x) &&
//             close(a.orientation.y, b.orientation.y) &&
//             close(a.orientation.z, b.orientation.z) &&
//             close(a.orientation.w, b.orientation.w);

//         if (same) return;
//     }

//     // last_ee_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(*msg);

//     // Drop immediately if still moving — never block the callback
//     if (motion_busy_.exchange(true)) return;

//     // Safe to join now — busy_ was false so thread must be finished
//     if (motion_thread_.joinable()) motion_thread_.join();

//     last_ee_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>(*msg);

//     geometry_msgs::msg::Pose target_pose = msg->pose;

//     motion_thread_ = std::thread([this, target_pose]() {
//         // move_group_->stop();
//         // Ensure planner starts from current joint state
//         move_group_->setStartStateToCurrentState();
//         move_group_->setPlanningTime(1.0);
//         //move_group_->setPlannerId("RRTConnectkConfigDefault"); // or try "PTP" if on industrial setup
//         move_group_->setPoseTarget(target_pose);
//         moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//         bool success = (move_group_->plan(my_plan) ==
//             moveit::planning_interface::MoveItErrorCode::SUCCESS);
//         if (success) {
//             move_group_->execute(my_plan);
//             // Give MoveIt's action server time to fully release
//             std::this_thread::sleep_for(std::chrono::milliseconds(500));
//         } else {
//             RCLCPP_WARN(this->get_logger(), "Planning to target pose failed.");
//         }
//         motion_busy_ = false;
//     });
}


void TestMotion::moveusingQ(double q1, double q2,double q3,double q4,double q5,double q6){

    // if (!move_group_) {     // if MoveIt initialised correctly
    //     RCLCPP_ERROR(this->get_logger(), "Move group not initialized!");
    //     return;
    // }
    // std::vector<double> target = {q1, q2, q3, q4, q5, q6};
    // move_group_->setJointValueTarget(target);
    // move_group_->move();

}

void TestMotion::readQvalues(void){

    // std::lock_guard<std::mutex> lock(jointstate_mtx_);  // Mutex as data may be updated at same time

    // if (jointValues.size() >= 6)                        // Ensure all joint values are assigned 
    // {
    //     RCLCPP_INFO(this->get_logger(),
    //         "Current joints: %.2f %.2f %.2f %.2f %.2f %.2f",
    //         jointValues[0], jointValues[1], jointValues[2],
    //         jointValues[3], jointValues[4], jointValues[5]);
    // }
}

// void TestMotion::publishJoints(void){
//     std::vector<double> joints;
//     // Do something under lock 
//     {
//         std::lock_guard<std::mutex> lock(jointstate_mtx_);
//         joints = jointValues;
//     }

//     if (joints.size() < 6) {
//         RCLCPP_WARN(this->get_logger(), "Not enough joint values to publish");
//         return;
//     }

//     shoulder_pan_joint = joints.at(0);
//     shoulder_lift_joint = joints.at(1);
//     elbow_joint = joints.at(2); 
//     wrist_1_joint = joints.at(3);
//     wrist_2_joint = joints.at(4);
//     wrist_3_joint = joints.at(5);

//     msg.header.stamp = this->get_clock()->now();
//     msg.name = {
//         "shoulder_pan_joint",
//         "shoulder_lift_joint",
//         "elbow_joint",
//         "wrist_1_joint",
//         "wrist_2_joint",
//         "wrist_3_joint"
//     };
//     msg.position = {
//         shoulder_pan_joint,
//         shoulder_lift_joint,
//         elbow_joint,
//         wrist_1_joint,
//         wrist_2_joint,
//         wrist_3_joint
//     };
    
//     jointstatesPub_->publish(msg);
// }

void TestMotion::publishJoints(void) {
    // std::vector<double> joints;
    // {
    //     std::lock_guard<std::mutex> lock(jointstate_mtx_);
    //     joints = jointValues;
    // }
    // if (joints.size() < 6) { 
    //     RCLCPP_WARN(this->get_logger(), "Not enough joint values to publish"); 
    //     return; 
    // }

    // sensor_msgs::msg::JointState msg;  // local variable
    // msg.header.stamp = this->get_clock()->now();
    // msg.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    //             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    // msg.position = {joints[0], joints[1], joints[2],
    //                 joints[3], joints[4], joints[5]};
    // jointStatesToUnityPub_->publish(msg);
}


// This function will simulate the joint positions updating as the user moves 
// the EE in unity
void TestMotion::demoMovement(){
    // // joint positions
    // double shoulder_pan_joint = 0;
    // double shoulder_lift_joint = -1.57;
    // double elbow_joint = 0; 
    // double wrist_1_joint = -1.57;
    // double wrist_2_joint = 0;
    // double wrist_3_joint = 0.0;

    //  // Increment each step
    // double increment = 0.05;
    // int steps = 100;

    // for (int i = 0; i < steps; i++)
    // {
    //     // Slightly change each joint each iteration
    //     shoulder_pan_joint += increment;
    //     shoulder_lift_joint += increment * 0.5;
    //     elbow_joint -= increment * 0.3;
    //     wrist_1_joint += increment * 0.2;
    //     wrist_2_joint -= increment * 0.1;
    //     wrist_3_joint += increment * 0.4;

    //     sensor_msgs::msg::JointState msg;
    //     msg.header.stamp = this->get_clock()->now();
    //     msg.name = {
    //         "shoulder_pan_joint",
    //         "shoulder_lift_joint",
    //         "elbow_joint",
    //         "wrist_1_joint",
    //         "wrist_2_joint",
    //         "wrist_3_joint"
    //     };
    //     msg.position = {
    //         shoulder_pan_joint,
    //         shoulder_lift_joint,
    //         elbow_joint,
    //         wrist_1_joint,
    //         wrist_2_joint,
    //         wrist_3_joint
    //     };
    //     jointstatesPub_->publish(msg);
    //     RCLCPP_INFO(this->get_logger(),
    //         "Demo step %d: publishing joints: %.2f %.2f %.2f %.2f %.2f %.2f",
    //         i, shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
    //         wrist_1_joint, wrist_2_joint, wrist_3_joint);

    //     // Wait between steps so the robot has time to reach each position
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // }
}

