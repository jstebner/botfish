#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "manipulation/manipulation.hpp"

manip::Manipulation::Manipulation(rclcpp::NodeOptions options) : Node("manipulation", options) {
    _cell_offset = this->declare_parameter("cell_offset", 0.05);
    _end_effector_link = this->declare_parameter("end_effector", "right_arm_6_link");
    _reference_link = this->declare_parameter("reference_link", "right_arm_podent_link");
    _moveit_group = this->declare_parameter("move_group", "right_arm");
    _grab_height = this->declare_parameter("grab_height", 0.352);
    _move_height = this->declare_parameter("move_height", 0.1);
    _goal_tolerance = this->declare_parameter("goal_tolerance", 0.0125);

    _hand_orientation.x = 0.004;
    _hand_orientation.y = -0.003;
    _hand_orientation.z = 0.588;
    _hand_orientation.w = 0.809;

    _starting_position.position.x = 0.055;
    _starting_position.position.y = 0.100;
    _starting_position.position.z = 0.512;
    _starting_position.orientation = _hand_orientation;

    _engine_move_sub = this->create_subscription<std_msgs::msg::String>(
            "/engine_move", 10, std::bind(&Manipulation::move_cb, this, std::placeholders::_1));

    _gripper_pub = this->create_publisher<std_msgs::msg::Float64>("/right_gripper/position", 10);

    (*move_group_interface) = moveit::planning_interface::MoveGroupInterface(static_cast<const SharedPtr>(this),
                                                                             _moveit_group);

    move_group_interface->setPoseReferenceFrame(_reference_link);
    move_group_interface->setEndEffector(_end_effector_link);
    move_group_interface->setGoalTolerance(_goal_tolerance);
}

void manip::Manipulation::move_cb(std_msgs::msg::String::SharedPtr msg) {
    std::vector<manip::cell_location> parsed_moves = parse_move(msg->data);

    //Move one piece to an open cell
    bool grasping = true;
    actuate(parsed_moves.at(0));
    /*for (auto i: parsed_moves) {
        actuate(i);
        //grab(grasping);
        //grasping = !grasping;
    }*/
}

std::vector<manip::cell_location> manip::Manipulation::parse_move(std::string move) {
    std::vector<manip::cell_location> parsed_moves;
    std::vector<std::string> locations;

    //Break move string into starting and ending cells
    locations.push_back(move.substr(0, 2));
    locations.push_back(move.substr(2, 3));

    //Convert letter val to number and multiply both letter and number by cell offset to get location
    for (auto i: locations) {
        manip::cell_location loc{};
        char letter = i[0];
        std::string num(1, i[1]);
        letter = toupper(letter);
        const unsigned int index = letter - 'A';
        int value = letter_to_value[index];
        loc.x_dist = (double) value * _cell_offset;
        loc.y_dist = (double) std::stoi(num) * _cell_offset;
        parsed_moves.push_back(loc);
    }
    return parsed_moves;
}

void manip::Manipulation::actuate(manip::cell_location location) {
    moveit::planning_interface::MoveGroupInterface::Plan msg;

    //Calculate cell location to move to
    _target_pose.position.x = this->_starting_position.position.x + location.x_dist;
    //z value is y for our current use case
    _target_pose.position.z = this->_starting_position.position.z + location.y_dist;
    _target_pose.position.y = this->_starting_position.position.y;
    _target_pose.orientation = this->_hand_orientation;

    plan_execute();

}

void manip::Manipulation::grab(bool position) {
    std_msgs::msg::Float64 pos;

    //Ensure that if were grabbing a piece, that the gripper is opened before descending
    if (position) {
        pos.data = 0.0;
        this->_gripper_pub->get()->publish(pos);
        //Now that gripper is open set data to 1.0 to prep for close later
        pos.data = 1.0;
    } else {
        //Set data to 0.0 to prep for open later
        pos.data = 0.0;
    }

    //Move to grab height to be able to interact with pieces
    _target_pose.position.y = _grab_height;
    plan_execute();

    //Open/close the gripper
    this->_gripper_pub->get()->publish(pos);

    //Return to movement height
    _target_pose.position.y = _move_height;
    plan_execute();
}

void manip::Manipulation::plan_execute() {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    move_group_interface->setPoseTarget(_target_pose);

    auto frame_id = move_group_interface->getPlanningFrame();
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 5.0;
    primitive.dimensions[primitive.BOX_Y] = 5.0;
    primitive.dimensions[primitive.BOX_Z] = 0.0;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.25;
    box_pose.position.y = -0.5;
    box_pose.position.z = 0.01;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);

    //Attempt to plan to that position
    RCLCPP_INFO(this->get_logger(), "Planning movement...");
    auto const plan_ok = static_cast<bool>(move_group_interface->plan(msg));
    if (!plan_ok) {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan!!!");
    } else {
        RCLCPP_INFO(this->get_logger(), "Planning Succeeded!");
        //Execute our calculated plan
        RCLCPP_INFO(this->get_logger(), "Executing movement...");
        auto const exec_ok = static_cast<bool>(move_group_interface->execute(msg));
        if (!exec_ok) {
            RCLCPP_ERROR(this->get_logger(), "Execute failed!!!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Execution Succeeded!");
        }
    }
}
