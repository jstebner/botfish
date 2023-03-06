#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "manipulation/manipulation.hpp"
#include <regex>

//TODO: Try out orientation constraining to keep end effector straight,
//      Get grippers working over topic,
//      Prep for potential chess playing

manip::Manipulation::Manipulation(rclcpp::NodeOptions options) : Node("manipulation", options) {
    _cell_offset = this->declare_parameter("cell_offset", 0.05);
    _end_effector_link = this->declare_parameter("end_effector", "right_hand_base_link");
    _reference_link = this->declare_parameter("reference_link", "right_arm_podest_link");
    _grab_height = this->declare_parameter("grab_height", 0.202);
    _move_height = this->declare_parameter("move_height", 0.152);
    _goal_tolerance = this->declare_parameter("goal_tolerance", 0.0125);

    //Define constant quaternion to keep the hand at
    //[-0.707, -0.000, 0.001, 0.708]
    _hand_orientation.x = -0.707;
    _hand_orientation.y = -0.000;
    _hand_orientation.z = 0.001;
    _hand_orientation.w = 0.708;

    //Define where the starting position is in coordinate space
    _starting_position.position.x = -0.095;
    _starting_position.position.y = _grab_height;
    _starting_position.position.z = 0.312;
    _starting_position.orientation = _hand_orientation;

    //Define where we are going to grab new queens from relative to that position
    _queen_loader_position.position.x = _starting_position.position.x + 0.42;
    _queen_loader_position.position.y = _starting_position.position.y;
    _queen_loader_position.position.z = _starting_position.position.z;
    _queen_loader_position.orientation = _hand_orientation;

    //Fill grab matrix:
    //[0.9,  0.989,  0.99, 0.91, 0.92, 0.9, 0.9, 0.9, 0.9]
    //TODO: Please god find a better way to do this
    grabbed.data.push_back(0.9);
    grabbed.data.push_back(0.989);
    grabbed.data.push_back(0.99);
    grabbed.data.push_back(0.91);
    grabbed.data.push_back(0.92);
    grabbed.data.push_back(0.9);
    grabbed.data.push_back(0.9);
    grabbed.data.push_back(0.9);
    grabbed.data.push_back(0.2);

    released.data.push_back(0.9);
    released.data.push_back(0.989);
    released.data.push_back(0.99);
    released.data.push_back(0.91);
    released.data.push_back(0.92);
    released.data.push_back(0.9);
    released.data.push_back(0.9);
    released.data.push_back(0.9);
    released.data.push_back(0.9);


    _engine_move_sub = this->create_subscription<std_msgs::msg::String>(
            "/engine_move", 10, std::bind(&Manipulation::move_cb, this, std::placeholders::_1));

    _gripper_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/right_hand/target", 10);
}

void manip::Manipulation::move_cb(std_msgs::msg::String::SharedPtr msg) {
    std::vector<manip::cell_location> parsed_moves = parse_move(msg->data);

    for(auto i : parsed_moves){
        this->_gripper_pub->get()->publish(grabbed);
        actuate(i);
        _target_pose = _queen_loader_position;
        plan_execute();
        this->_gripper_pub->get()->publish(released);
        sleep(5);
    }
}

std::vector<manip::cell_location> manip::Manipulation::parse_move(std::string move) {
    std::vector<manip::cell_location> parsed_moves{};
    std::vector<std::string> moves{};

    const std::regex e("\\D\\d*");
    std::regex_token_iterator<std::string::iterator> rend;
    std::regex_token_iterator<std::string::iterator> a ( move.begin(), move.end(), e );
    while (a!=rend){
        moves.push_back(*a++);
    };
    for(auto i : moves){
        manip::cell_location loc{};
        //Convert letter val to number and multiply both letter and number by cell offset to get location
        char letter = i[0]; //Grab char in first position, ex: A1 -> A
        std::string num(1, i[1]); //Convert num in second position to a string
        letter = toupper(letter);
        const unsigned int index = letter - 'A'; //Get index of the char from first position in array
        int value = letter_to_value[index]; //Get that number from array
        loc.x_dist = ((double) (value - 1)) *
                     _cell_offset; //Convert to x_dist ensuring that value is reduced by one because 0 indexing
        loc.y_dist = ((double) (std::stoi(num) - 1)) * _cell_offset; //Same as above but convert to num using std::stoi

        RCLCPP_INFO(this->get_logger(), "Cell location: %f, %f", this->_starting_position.position.x + loc.x_dist,
                    this->_starting_position.position.z + loc.y_dist);
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
    _target_pose.position.y = this->_move_height;
    _target_pose.orientation = this->_hand_orientation;

    plan_execute();

}

void manip::Manipulation::grab(bool position) {
    std_msgs::msg::Float64 pos;

    //Ensure that if were grabbing a piece, that the gripper is opened before descending
    if (position) {
        pos.data = 0.0;
        //this->_gripper_pub->get()->publish(pos);
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
    //this->_gripper_pub->get()->publish(pos);

    //Return to movement height
    _target_pose.position.y = _move_height;
    plan_execute();
}

void manip::Manipulation::plan_execute() {
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
    move_group_interface->setMaxVelocityScalingFactor(0.4);
    move_group_interface->setMaxAccelerationScalingFactor(0.4);

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
    _target_pose.position = _queen_loader_position.position;
    _target_pose.orientation = _hand_orientation;
    plan_execute();
    this->_gripper_pub->get()->publish(released);
}
