#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
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

        ///@brief Given a MoveGroupInterface this function will set up that interface to use to move pieces
        ///@param interface the MoveGroupInterface pointer to use for plans
        void setup_moveit(moveit::planning_interface::MoveGroupInterface *interface);


    private:
        std::optional<std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>>> _engine_move_sub =
                std::nullopt;

        std::optional<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>>>
                _gripper_pub = std::nullopt;

        //distance to the middle of a cell, this should be half the cell length
        double _cell_offset{};

        //Height to bring the arm to be able to interact with pieces
        double _grab_height{};

        //Height to bring the arm to be able to move around the board without hitting pieces, should be
        //higher than the height of the tallest piece
        double _move_height{};

        //Tolerance for planning to goals, number is meters radius of a sphere around the end location
        double _goal_tolerance{};

        //Link that will be used to move pieces around
        std::string _end_effector_link{};

        //Reference to calculate position of _end_effector_link with
        std::string _reference_link{};

        //Group to create interface on
        std::string _moveit_group{};

        //fixed orientation of the hand, will end up being the orientation of the end effector link
        geometry_msgs::msg::Quaternion _hand_orientation{};

        //Reference point for calculating where to go to, should be either the A1 cell or one of the corners of the board
        geometry_msgs::msg::Pose _starting_position{};

        //Position to bring the hand to load a new queen from
        geometry_msgs::msg::Pose _queen_loader_position{};

        //Pose to plan to
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
        cell_location parse_move(std::string move);
    };
}
