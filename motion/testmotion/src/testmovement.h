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

        void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg); //! Callback to read join states 

    private:
        //rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr JointStates_; // Publish joint states to robot
        rclcpp::TimerBase::SharedPtr timer_;  //!< Timer to trigger periodic publishing of joint states

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subJointStates_; // Subscribes to JointStates ur3e.

        //rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subCloud_; // Subscribes to PointCloud2 of LIDAR data.

        // Joint values
        std::vector<double> jointValues;
        double q1_;
        double q2_; 
        double q3_;
        double q4_; 
        double q5_; 
        double q6_;

        std::mutex jointstate_mtx_;

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

};
