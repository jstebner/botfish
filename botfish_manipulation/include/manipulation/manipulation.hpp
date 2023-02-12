#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>

namespace manip{

    struct cell_location{
        double x_dist;
        double y_dist;
    };

    class Manipulation : public rclcpp::Node {
        public:
            explicit Manipulation(rclcpp::NodeOptions options);
        private:
            std::optional<std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>>> _engine_move_sub =
                std::nullopt;

            double _cell_offset{};
            std::string _end_effector_link{};
            std::string _reference_link{};
            std::string _moveit_group{};

            moveit::planning_interface::MoveGroupInterface *move_group_interface = nullptr;
            
            void calculate_trajectory(std_msgs::msg::String::SharedPtr msg);
            
            std::tuple<cell_location, cell_location> parse_move(std::string move);
    };
}