#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <vector>

namespace manip {

    const unsigned int letter_to_value[] =
            {1, 2, 3, 4, 5, 6, 7, 8};

    struct cell_location {
        double x_dist;
        double y_dist;
    };

    class Manipulation : public rclcpp::Node {
    public:
        explicit Manipulation(rclcpp::NodeOptions options);

    private:
        std::optional<std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>>> _engine_move_sub =
                std::nullopt;

        std::optional<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>>>
                _gripper_pub = std::nullopt;

        double _cell_offset{};
        double _grab_height{};
        double _move_height{};
        double _goal_tolerance{};
        std::string _end_effector_link{};
        std::string _reference_link{};
        std::string _moveit_group{};
        geometry_msgs::msg::Quaternion _hand_orientation{};
        geometry_msgs::msg::Pose _starting_position{};
        geometry_msgs::msg::Pose _target_pose{};

        moveit::planning_interface::MoveGroupInterface *move_group_interface = nullptr;

        /// @brief Callback for moves from the engine
        /// @param msg string msg containing the move outputted by the engine, format will be startingLocationEndingLocation ex: e2e4
        void move_cb(std_msgs::msg::String::SharedPtr msg);

        /// @brief Calculates the xyz/quaternion that the end effector of the robot needs to go to and moves arm to that location
        /// @param location cell to move arm to
        void actuate(cell_location location);

        /// @breif grabs piece from board using grippers
        /// @param position whether were grasping or releasing, true = grasping, false = releasing
        void grab(bool position);

        ///@brief Plan and execute pose currently in _target_pose
        void plan_execute();

        /// @brief Converts string moves to xy locations
        /// @param move string move recieved in the move_cb callback
        /// @return tuple of two cell_location structs each containing two doubles for both x and y location
        std::vector<cell_location> parse_move(std::string move);
    };
}
