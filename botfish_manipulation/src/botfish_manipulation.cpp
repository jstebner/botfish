#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include <std_msgs/msg/float64_multi_array.hpp>


class Clasp : public rclcpp::Node
{
public:
    Clasp() : Node("Clasp")
    {
        auto pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("left_hand/target", 1);
        
        //float64 multi array 
        //make elements 6 and 7 1.0
        
        //Float64 dat = [0.0,  0.089,  0.29, 0.11, 0.12, 0.0, -0.0, -0.0, -0.0];
        std_msgs::msg::Float64MultiArray msg;
        //for(int i = 0; i < 9; i++)
        msg.data.push_back(0.0);
        msg.data.push_back(0.0089);
        msg.data.push_back(0.29);
        msg.data.push_back(0.11);
        msg.data.push_back(0.12);
        msg.data.push_back(0.0);
        msg.data.push_back(-0.0);
        msg.data.push_back(-0.0);
        msg.data.push_back(-0.0); 
        
        pub_->publish(msg);
        //callbackData(msg);
        
    }
};

class Unclasp : public rclcpp::Node
{
public:
    Unclasp() : Node("Unclasp")
    {
        auto pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("left_hand/target", 1);
        
        //float64 multi array 
        //make elements 6 and 7 1.0
        
        //Float64 dat = [0.0,  0.089,  0.29, 0.11, 0.12, 0.0, -0.0, -0.0, -0.0];
        std_msgs::msg::Float64MultiArray msg;
        //for(int i = 0; i < 9; i++)
        msg.data.push_back(0.0);
        msg.data.push_back(0.0);
        msg.data.push_back(0.0);
        msg.data.push_back(0.0);
        msg.data.push_back(0.0);
        msg.data.push_back(0.0);
        msg.data.push_back(-0.0);
        msg.data.push_back(-0.0);
        msg.data.push_back(0.95); 
        
        pub_->publish(msg);
        //callbackData(msg);
        
    }
};



int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "botfish_manipulation", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("botfish_manipulation");

  // Create the MoveIt MoveGroup Interface

  auto unexNode = Unclasp();
  
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "left_arm");
  // Set a target Pose
  auto const target_A1 = [&move_group_interface] {
    geometry_msgs::msg::Pose msg;

	//left hand A1
	float translation[] = {0.244, -0.21, 0.319};
	float rotationQuaternion[] = {0.708, -0.001, -0.002, 0.707};

    msg.position.x = translation[0];
    msg.position.y = translation[1];
    msg.position.z = translation[2];

    msg.orientation.x = rotationQuaternion[0];
    msg.orientation.y = rotationQuaternion[1];
	msg.orientation.z = rotationQuaternion[2];
	msg.orientation.w = rotationQuaternion[3];
	

    return msg;
  }();
  
  using moveit::planning_interface::MoveGroupInterface;
  // Set a target Pose
  auto const target_H1 = [&move_group_interface] {
    geometry_msgs::msg::Pose msg;
	
	//left hand H1
	float translation[] = {-0.106, -0.15, 0.319};
	float rotationQuaternion[] = {0.708, -0.001, -0.002, 0.707};

    msg.position.x = translation[0];
    msg.position.y = translation[1];
    msg.position.z = translation[2];

    msg.orientation.x = rotationQuaternion[0];
    msg.orientation.y = rotationQuaternion[1];
	msg.orientation.z = rotationQuaternion[2];
	msg.orientation.w = rotationQuaternion[3];
	

    return msg;
  }();
  
  using moveit::planning_interface::MoveGroupInterface;
  // Set a target Pose
  auto const target_H8 = [&move_group_interface] {
    geometry_msgs::msg::Pose msg;
	
	//left hand H8
	float translation[] = {-0.106, -0.15, 0.669};
	float rotationQuaternion[] = {0.708, -0.001, -0.002, 0.707};
	

    msg.position.x = translation[0];
    msg.position.y = translation[1];
    msg.position.z = translation[2];

    msg.orientation.x = rotationQuaternion[0];
    msg.orientation.y = rotationQuaternion[1];
	msg.orientation.z = rotationQuaternion[2];
	msg.orientation.w = rotationQuaternion[3];
	

    return msg;
  }();
  
  using moveit::planning_interface::MoveGroupInterface;
  // Set a target Pose
  auto const target_A8 = [&move_group_interface] {
    geometry_msgs::msg::Pose msg;
	
	
	//left hand A8
	float translation[] = {0.244, -0.20, 0.669};
	float rotationQuaternion[] = {0.708, -0.001, -0.002, 0.707};
	

    msg.position.x = translation[0];
    msg.position.y = translation[1];
    msg.position.z = translation[2];

    msg.orientation.x = rotationQuaternion[0];
    msg.orientation.y = rotationQuaternion[1];
	msg.orientation.z = rotationQuaternion[2];
	msg.orientation.w = rotationQuaternion[3];
	

    return msg;
  }();
  
  move_group_interface.setEndEffectorLink("left_hand_base_link");
  move_group_interface.setPoseReferenceFrame("left_arm_podest_link");
  
    
  move_group_interface.setPoseTarget(target_A1);
  move_group_interface.setGoalTolerance(0.00625);
  move_group_interface.setMaxVelocityScalingFactor(0.4);
  move_group_interface.setMaxAccelerationScalingFactor(0.4);
  
  
    // Create collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 5.0;
    primitive.dimensions[primitive.BOX_Y] = 5.0;
    primitive.dimensions[primitive.BOX_Z] = 0.005;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.25;
    box_pose.position.y = -0.5;
    box_pose.position.z = -0.005;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

	/*TODO: this is really bad practice, please clean this up after the demo*/


  // Execute the plan
  
  if (success)
  {
    move_group_interface.execute(plan);
    auto exNode = Clasp();
    // Update the target pose
    move_group_interface.setPoseTarget(target_H1);
    // Create a plan to that target pose
	  auto const [success, plan] = [&move_group_interface] {
		moveit::planning_interface::MoveGroupInterface::Plan msg;
		auto const ok = static_cast<bool>(move_group_interface.plan(msg));
		return std::make_pair(ok, msg);
	  }();
	  if(success)
	  {
	  	move_group_interface.execute(plan);
	  	
	  	// Update the target pose
	  	move_group_interface.setPoseTarget(target_H8);
	  	// Create a plan to that target pose
		  auto const [success, plan] = [&move_group_interface] {
			moveit::planning_interface::MoveGroupInterface::Plan msg;
			auto const ok = static_cast<bool>(move_group_interface.plan(msg));
			return std::make_pair(ok, msg);
		  }();
		  
		  if(success)
		  {
		  	move_group_interface.execute(plan);
		  	// Update the target pose
		  	move_group_interface.setPoseTarget(target_A8);
		  	// Create a plan to that target pose
			  auto const [success, plan] = [&move_group_interface] {
				moveit::planning_interface::MoveGroupInterface::Plan msg;
				auto const ok = static_cast<bool>(move_group_interface.plan(msg));
				return std::make_pair(ok, msg);
			  }();
			if(success)
		  	{
		  		move_group_interface.execute(plan);
		  	}	  
		  }
	  }
   
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
          /*auto pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("left_hand/target", 1);
        
        //float64 multi array 
        //make elements 6 and 7 1.0
        
        //Float64 dat = [0.0,  0.089,  0.29, 0.11, 0.12, 0.0, -0.0, -0.0, -0.0];
        std_msgs::msg::Float64MultiArray msg; 
        //for(int i = 0; i < 9; i++)
        msg.data.push_back(0.0);
        msg.data.push_back(0.0089);
        msg.data.push_back(0.29);
        msg.data.push_back(0.11);
        msg.data.push_back(0.12);
        msg.data.push_back(0.0);
        msg.data.push_back(-0.0);
        msg.data.push_back(-0.0);
        msg.data.push_back(-0.0); 
        
       	pub_->publish(msg);*/
  
  
  //MinimalPublisher();
  auto neoUnExNode = Unclasp();
  //rclcpp::spin(exNode);
  //rclcpp::spin(unexNode);
  
  
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
