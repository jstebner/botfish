#pragma once

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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

        ~Manipulation() override;

        ///@brief Given a MoveGroupInterface this function will set up that interface to use to move pieces
        ///@param interface the MoveGroupInterface pointer to use for plans
        void setup_moveit(moveit::planning_interface::MoveGroupInterface *interface);


    private:
        std::optional<std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>>> _engine_move_sub =
                std::nullopt;

        std::optional<std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>>>
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

        //Percentage of max velocity the arm should move at
        double _max_velocity{};

        //Percentage of max acceleration the arm should accelerate at
        double _max_acceleration{};

        //Time allowed for moveit to plan for
        double _planning_time{};

        //Message for adjusting finger positions to a grabbed state
        //[0.9,  0.989,  0.99, 0.91, 0.92, 0.9, 0.9, 0.9, 0.2]
        const std_msgs::msg::Float64MultiArray GRABBED = [] {
            std_msgs::msg::Float64MultiArray msg;
            msg.data.push_back(0.9);
            msg.data.push_back(0.989);
            msg.data.push_back(0.99);
            msg.data.push_back(0.91);
            msg.data.push_back(0.92);
            msg.data.push_back(0.9);
            msg.data.push_back(0.9);
            msg.data.push_back(0.9);
            msg.data.push_back(0.0);
            return msg;
        }();

        //Message for adjusting fingers to a released state
        //[0.9,  0.989,  0.99, 0.91, 0.92, 0.9, 0.9, 0.9, 0.9]
        const std_msgs::msg::Float64MultiArray RELEASED = [] {
            std_msgs::msg::Float64MultiArray msg;
            msg.data.push_back(0.9);
            msg.data.push_back(0.989);
            msg.data.push_back(0.99);
            msg.data.push_back(0.91);
            msg.data.push_back(0.92);
            msg.data.push_back(0.9);
            msg.data.push_back(0.9);
            msg.data.push_back(0.9);
            msg.data.push_back(0.5);
            return msg;
        }();

        //fixed orientation of the hand, will end up being the orientation of the end effector link
        //right hand quarternion vector [-0.707, -0.000, 0.001, 0.708]
        //left hand quarternion vector [0.654, -0.376, 0.373, 0.540]
        const geometry_msgs::msg::Quaternion HAND_ORIENTATION = [] {
            geometry_msgs::msg::Quaternion msg;
            msg.x = -0.334;//-0.707;
            msg.y = 0.627;//0.000;
            msg.z = -0.618;//0.001;
            msg.w = -0.338;//0.708;
            return msg;
        }();

        //Link that will be used to move pieces around
        std::string _end_effector_link{};

        //Reference to calculate position of _end_effector_link with
        std::string _reference_link{};
        std::string _sub_reference_link{};

        //Reference point for calculating where to go to, should be either the A1 cell or one of the corners of the board
        geometry_msgs::msg::Pose _starting_position{};

        //Position to bring the hand to load a new queen from
        geometry_msgs::msg::Pose _queen_loader_position{};

        //Pose to plan to
        geometry_msgs::msg::Pose _target_pose{};

        //Move group interface class pointer
        moveit::planning_interface::MoveGroupInterface *move_group_interface = nullptr;

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit_msgs::msg::CollisionObject lower_board_collision;

        /// @brief Callback for moves from the engine
        /// @param msg string msg containing the move outputted by the engine, format will be startingLocationEndingLocation ex: e2e4
        void move_cb(std_msgs::msg::String::SharedPtr msg);

        /// @brief Calculates the xyz/quaternion that the end effector of the robot needs to go to and moves arm to that location
        /// @param location cell to move arm to
        void actuate(cell_location location);

        ///@brief Plan and execute movement to pose currently in _target_pose from end effector curren t
        void plan_execute();

        /// @brief Parses a string of cell locations and converts them to x, y offsets to be used by actuate()
        /// @param move string of cell locations received in the move_cb callback, ex: "A1B1C1D1"
        /// @return vector of tuples of two cell_location structs each containing two doubles for both x and y location
        std::vector<cell_location> parse_move(std::string move);
    };
}
