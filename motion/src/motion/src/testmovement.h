#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <mutex>
#include <moveit/move_group_interface/move_group_interface.h> // For moving robot


class TestMotion : public rclcpp::Node
{
    public:
        TestMotion();
        ~TestMotion();

        void initMoveIt(); // Function to intialsie MoveIt

        void readQvalues(void); // Subscribes to joint status

        void moveusingQ(double q1, double q2, double q3, double q4, double q5, double q6);

        void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg); //! Callback to read joint states 
        void targetJointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg); //! Callback to read target joint states 
        void targetEEPoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg); //! Callback to read target EE pose 

        bool isAtTargetPose(const geometry_msgs::msg::Pose& target,
                    double pos_tol = 0.1,
                    double ori_tol = 0.05);

        void publishJoints(void); // publish joint states to Unity.

        double normalizeAngle(double angle);

        void demoMovement(void);    // Movement for demo

    private:
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstatesPub_; // Publish joint states to robot
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatesToUnityPub_; // Publish joint states to robot
        
        rclcpp::TimerBase::SharedPtr timer_;  //!< Timer to trigger periodic publishing of joint states

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subJointStates_; //! Subscribes to JointStates ur3e.
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subTargetJointStates_; //! Subscribes to target JointStates from Unity (hopefully).
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subTargetEEPose_; //! Subscribes to target End Effector pose from Unity (hopefully).

        //rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subCloud_; // Subscribes to PointCloud2 of LIDAR data.

        // Joint values
        std::vector<double> jointValues;

        // Last recieved target EE pose
        //geometry_msgs::msg::PoseStamped last_ee_pose_;
        geometry_msgs::msg::PoseStamped::SharedPtr last_ee_pose_;

        std::mutex jointstate_mtx_;
        std::mutex motion_start_mtx_;  // guards join + exchange sequence

        // std::atomic<bool> is_moving_{false};

        std::thread motion_thread_;
        std::atomic<bool> motion_busy_{false};

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

};
