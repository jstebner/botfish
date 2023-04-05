#include <regex>
#include "manipulation/manipulation.hpp"

manip::Manipulation::Manipulation(rclcpp::NodeOptions options) : Node("manipulation", options) {
    _cell_offset = this->declare_parameter("cell_offset", 0.05);
    _end_effector_link = this->declare_parameter("end_effector", "left_hand_d");
    _reference_link = this->declare_parameter("reference_link", "left_arm_podest_link");
    _grab_height = this->declare_parameter("grab_height", -0.152);//-0.152);//-0.202);//-0.1);
    _move_height = this->declare_parameter("move_height", -0.152);
    _goal_tolerance = this->declare_parameter("goal_tolerance", 0.001);
    _max_velocity = this->declare_parameter("max_velocity", 0.3);
    _max_acceleration = this->declare_parameter("max_acceleration", 0.2);
    _planning_time = this->declare_parameter("planning_time", 10.0);

    //Define where the starting position is in coordinate space
    _starting_position.position.x = -0.095;//-0.106;
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
            "/botfish/engine_move", 10, std::bind(&Manipulation::move_cb, this, std::placeholders::_1));

    //Publishers
    _gripper_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/left_hand/target", 10);
    _finished_move_pub = this->create_publisher<std_msgs::msg::String>("/botfish/current_move", 10);
}

void manip::Manipulation::move_cb(std_msgs::msg::String::SharedPtr msg) {
    std::vector<manip::cell_location> parsed_moves = parse_move(msg->data);

    for (const auto &i: parsed_moves) {
        //Send grab message 3 times to make sure hands get to grabbed position as we had issue with that in the past
        for (int j = 0; j < 3; j++) {
            this->_gripper_pub->get()->publish(GRABBED);
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }

        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        actuate(i);

        for (int j = 0; j < 3; j++) {
            this->_gripper_pub->get()->publish(RELEASED);
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }

        std_msgs::msg::String str_msg;
        RCLCPP_INFO(this->get_logger(), "Publishing move: %s", i.move.c_str());
        str_msg.data = i.move;
        _finished_move_pub->get()->publish(str_msg);

        _target_pose = _queen_loader_position;
        plan_execute();

        sleep(3);
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
        RCLCPP_INFO(this->get_logger(), "Processing move string %s", i.c_str());
        //Convert letter val to number and multiply both letter and number by cell offset to get location
        char letter = i[0]; //Grab char in first position, ex: A1 -> A
        std::string num(1, i[1]); //Convert num in second position to a string
        letter = toupper(letter);
        const unsigned int index = letter - 'A'; //Get index of the char from first position in array
        int value = letter_to_value[index]; //Get that number from array
        loc.x_dist = ((double) (value - 1)) *
                     _cell_offset; //Convert to x_dist ensuring that value is reduced by one because 0 indexing
        loc.y_dist = ((double) (std::stoi(num) - 1)) * _cell_offset; //Same as above but convert to num using std::stoi
        loc.move = i;

        RCLCPP_INFO(this->get_logger(), "Cell location: %f, %f", this->_starting_position.position.x + loc.x_dist,
                    this->_starting_position.position.z + loc.y_dist);
        parsed_moves.push_back(loc);
    }
    return parsed_moves;
}

void manip::Manipulation::actuate(const manip::cell_location &location) {

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
    RCLCPP_INFO(this->get_logger(),
                "Planning to target position x: %f, y: %f, z: %f, With target quaternion: x: %f y: %f z: %f w: %f",
                _target_pose.position.x,
                _target_pose.position.y, _target_pose.position.z, _target_pose.orientation.x,
                _target_pose.orientation.y, _target_pose.orientation.z, _target_pose.orientation.w);
    auto const plan_ok = static_cast<bool>(move_group_interface->plan(msg));
    if (!plan_ok) {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan!!!");
    } else {
        auto const exec_ok = static_cast<bool>(move_group_interface->execute(msg));
        if (!exec_ok) {
            RCLCPP_ERROR(this->get_logger(), "Execute failed!!!");
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
    lower_board_collision.id = "plane1";
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
    box_pose.position.z = 0.0755;

    lower_board_collision.primitives.push_back(primitive);
    lower_board_collision.primitive_poses.push_back(box_pose);
    lower_board_collision.operation = moveit_msgs::msg::CollisionObject::ADD;
    // Add the collision object to the scene
    planning_scene_interface.applyCollisionObject(lower_board_collision);

    upper_ceiling_collision.header.frame_id = frame_id;
    upper_ceiling_collision.id = "plane2";
    shape_msgs::msg::SolidPrimitive primitive2;


    // Define the size of the box in meters
    primitive2.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 5.0;
    primitive2.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 5.0;
    primitive2.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.0;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose2;
    box_pose2.orientation.w = 1.0;
    box_pose2.position.x = 0.25;
    box_pose2.position.y = -0.55;
    box_pose2.position.z = 0.605;

    upper_ceiling_collision.primitives.push_back(primitive2);
    upper_ceiling_collision.primitive_poses.push_back(box_pose2);
    upper_ceiling_collision.operation = moveit_msgs::msg::CollisionObject::ADD;
    // Add the collision object to the scene
    planning_scene_interface.applyCollisionObject(upper_ceiling_collision);


    _target_pose.position = _queen_loader_position.position;
    _target_pose.orientation = HAND_ORIENTATION;
    this->_gripper_pub->get()->publish(RELEASED);
    plan_execute();

/*

    RCLCPP_INFO(this->get_logger(), "Setting orientation constraint...");
    moveit_msgs::msg::OrientationConstraint ocm;
    moveit_msgs::msg::Constraints test_constraints;
    ocm.link_name = _end_effector_link;
    ocm.header.frame_id = _reference_link;
    //ocm.header.frame_id = _sub_reference_link;
    //ocm.orientation = HAND_ORIENTATION;
    //ocm.absolute_x_axis_tolerance = 1.0;//0.5;
    //ocm.absolute_y_axis_tolerance = 1.0;//0.5;
    //ocm.absolute_z_axis_tolerance = 1.0;//0.5;
    ocm.weight = 1.0;
    test_constraints.orientation_constraints.emplace_back(ocm);
    move_group_interface->setPlanningTime(this->_planning_time);
    move_group_interface->setPathConstraints(test_constraints);
*/


}

manip::Manipulation::~Manipulation() {
    //Stop any current movement trajectories if any are running, forget all targets that were set and shutdown
    move_group_interface->stop();
    move_group_interface->clearPoseTargets();
}
