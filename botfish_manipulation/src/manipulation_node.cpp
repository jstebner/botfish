#include <regex>
#include "manipulation/manipulation.hpp"

//TODO: Try out orientation constraining to keep end effector straight,
//      Prep for potential chess playing

manip::Manipulation::Manipulation(rclcpp::NodeOptions options) : Node("manipulation", options) {
    _cell_offset = this->declare_parameter("cell_offset", 0.05);
    _end_effector_link = this->declare_parameter("end_effector", "right_hand_base_link");
    _reference_link = this->declare_parameter("reference_link", "right_arm_podest_link");
    _sub_reference_link = this->declare_parameter("subreference_link", "right_arm_4_link");
    _grab_height = this->declare_parameter("grab_height", 0.202);
    _move_height = this->declare_parameter("move_height", 0.152);
    _goal_tolerance = this->declare_parameter("goal_tolerance", 0.0125);
    _max_velocity = this->declare_parameter("max_velocity", 0.2);
    _max_acceleration = this->declare_parameter("max_acceleration", 0.2);
    _planning_time = this->declare_parameter("planning_time", 10.0);

    //Define where the starting position is in coordinate space
    _starting_position.position.x = -0.095;
    _starting_position.position.y = _grab_height;
    _starting_position.position.z = 0.312;
    _starting_position.orientation = HAND_ORIENTATION;

    //Define where we are going to grab new queens from relative to that position
    _queen_loader_position.position.x = _starting_position.position.x + 0.42;
    _queen_loader_position.position.y = _starting_position.position.y;
    _queen_loader_position.position.z = _starting_position.position.z;
    _queen_loader_position.orientation = HAND_ORIENTATION;

    //Subscribers
    _engine_move_sub = this->create_subscription<std_msgs::msg::String>(
            "/engine_move", 10, std::bind(&Manipulation::move_cb, this, std::placeholders::_1));

    //Publishers
    _gripper_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/right_hand/target", 10);
}

void manip::Manipulation::move_cb(std_msgs::msg::String::SharedPtr msg) {
    std::vector<manip::cell_location> parsed_moves = parse_move(msg->data);

    for (auto i: parsed_moves) {
        for(int j = 0; j < 3; j++){
            this->_gripper_pub->get()->publish(GRABBED);
            sleep(1.0);
        }
        sleep(1.0);
        actuate(i);
        for(int j = 0; j < 3; j++){
            this->_gripper_pub->get()->publish(RELEASED);
            sleep(1.0);
        }
        _target_pose = _queen_loader_position;
        plan_execute();

        //sleep(5);
    }
}

std::vector<manip::cell_location> manip::Manipulation::parse_move(std::string move) {
    std::vector<manip::cell_location> parsed_moves{};
    std::vector<std::string> moves{};

    const std::regex e("\\D\\d*");
    std::regex_token_iterator<std::string::iterator> rend;
    std::regex_token_iterator<std::string::iterator> a(move.begin(), move.end(), e);
    while (a != rend) {
        moves.push_back(*a++);
    }
    for (auto i: moves) {
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
    _target_pose.orientation = this->HAND_ORIENTATION;

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

    RCLCPP_INFO(this->get_logger(), "Setting max velocity to: %f, And max acceleration scaling factor to: %f",
                this->_max_velocity, this->_max_acceleration);
    move_group_interface->setMaxVelocityScalingFactor(this->_max_velocity);
    move_group_interface->setMaxAccelerationScalingFactor(this->_max_acceleration);

    RCLCPP_INFO(this->get_logger(), "Setting planning time to %f", this->_planning_time);
    move_group_interface->setPlanningTime(this->_planning_time);

    RCLCPP_INFO(this->get_logger(), "Creating collision box...");
    auto frame_id = move_group_interface->getPlanningFrame();

    lower_board_collision.header.frame_id = frame_id;
    lower_board_collision.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 5.0;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 5.0;
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.0;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.25;
    box_pose.position.y = -0.55;
    box_pose.position.z = 0.01;

    lower_board_collision.primitives.push_back(primitive);
    lower_board_collision.primitive_poses.push_back(box_pose);
    lower_board_collision.operation = moveit_msgs::msg::CollisionObject::ADD;
    // Add the collision object to the scene
    planning_scene_interface.applyCollisionObject(lower_board_collision);

    _target_pose.position = _queen_loader_position.position;
    _target_pose.orientation = HAND_ORIENTATION;
    plan_execute();


    /*RCLCPP_INFO(this->get_logger(), "Setting orientation constraint...");
    moveit_msgs::msg::OrientationConstraint ocm;
    moveit_msgs::msg::Constraints test_constraints;
    ocm.link_name = _end_effector_link;
    //ocm.header.frame_id = _reference_link;
    ocm.header.frame_id = _sub_reference_link;
    ocm.orientation = HAND_ORIENTATION;
    ocm.absolute_x_axis_tolerance = 0.5;
    ocm.absolute_y_axis_tolerance = 0.5;
    ocm.absolute_z_axis_tolerance = 0.5;
    ocm.weight = 1.0;
    test_constraints.orientation_constraints.emplace_back(ocm);
    move_group_interface->setPlanningTime(this->_planning_time);
    move_group_interface->setPathConstraints(test_constraints);*/


    this->_gripper_pub->get()->publish(RELEASED);
}

manip::Manipulation::~Manipulation() {
    //Stop any current movement trajectories if any are running, forget all targets that were set and shutdown
    move_group_interface->stop();
    move_group_interface->clearPoseTargets();
}
