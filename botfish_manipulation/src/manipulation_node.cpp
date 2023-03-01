#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "manipulation/manipulation.hpp"

manip::Manipulation::Manipulation(rclcpp::NodeOptions options) : Node("manipulation", options) {
    _cell_offset = this->declare_parameter("cell_offset", 0.05);
    _end_effector_link = this->declare_parameter("end_effector", "right_hand_base_link");
    _reference_link = this->declare_parameter("reference_link", "right_arm_podest_link");
    _moveit_group = this->declare_parameter("move_group", "right_arm");
    _grab_height = this->declare_parameter("grab_height", 0.352);
    _move_height = this->declare_parameter("move_height", 0.352);
    _goal_tolerance = this->declare_parameter("goal_tolerance", 0.0125);

    _hand_orientation.x = 0.004;
    _hand_orientation.y = -0.003;
    _hand_orientation.z = 0.588;
    _hand_orientation.w = 0.809;
    _starting_position.position.x = -0.095;
    _starting_position.position.y = _move_height;
    _starting_position.position.z = 0.312;
    _starting_position.orientation = _hand_orientation;

    _engine_move_sub = this->create_subscription<std_msgs::msg::String>(
            "/engine_move", 10, std::bind(&Manipulation::move_cb, this, std::placeholders::_1));

    _gripper_pub = this->create_publisher<std_msgs::msg::Float64>("/right_gripper/position", 10);
}

void manip::Manipulation::move_cb(std_msgs::msg::String::SharedPtr msg) {
    manip::cell_location parsed_move = parse_move(msg->data);

    //Move one piece to an open cell
    bool grasping = true;
    actuate(parsed_move);
    /*for (auto i: parsed_moves) {
        actuate(i);
        //grab(grasping);
        //grasping = !grasping;
    }*/
}

manip::cell_location manip::Manipulation::parse_move(std::string move) {
    manip::cell_location loc{};

    //Convert letter val to number and multiply both letter and number by cell offset to get location
    char letter = move[0]; //Grab char in first position, ex: A1 -> A
    std::string num(1, move[1]); //Convert num in second position to a string
    letter = toupper(letter);
    const unsigned int index = letter - 'A'; //Get index of the char from first position in array
    int value = letter_to_value[index]; //Get that number from array
    loc.x_dist = ((double) (value - 1)) *
                 _cell_offset; //Convert to x_dist ensuring that value is reduced by one because 0 indexing
    loc.y_dist = ((double) (std::stoi(num) - 1)) * _cell_offset; //Same as above but convert to num using std::stoi

    RCLCPP_INFO(this->get_logger(), "Cell location: %f, %f", this->_starting_position.position.x + loc.x_dist,
                this->_starting_position.position.z + loc.y_dist);
    return loc;
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
    //TODO: Break this up into separate plan/execute functions, Potentially add async planning
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    move_group_interface->setPoseTarget(_target_pose);

    //Attempt to plan to that position
    RCLCPP_INFO(this->get_logger(), "Planning movement to x: %f, y: %f, z: %f", _target_pose.position.x,
                _target_pose.position.y, _target_pose.position.z);
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

void manip::Manipulation::setup_moveit(moveit::planning_interface::MoveGroupInterface *interface) {
    RCLCPP_INFO(this->get_logger(), "Creating move group interface...");

    move_group_interface = interface;

    RCLCPP_INFO(this->get_logger(), "Setting reference link to %s...", _reference_link.c_str());
    move_group_interface->setPoseReferenceFrame(_reference_link);

    RCLCPP_INFO(this->get_logger(), "Setting end effector to %s...", _end_effector_link.c_str());
    move_group_interface->setEndEffectorLink(_end_effector_link);

    RCLCPP_INFO(this->get_logger(), "Setting goal tolerance to %f...", _goal_tolerance);
    move_group_interface->setGoalTolerance(_goal_tolerance);

    RCLCPP_INFO(this->get_logger(), "Creating collision box...");
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
    _target_pose.position = _starting_position.position;
    _target_pose.orientation = _hand_orientation;
    plan_execute();
}
