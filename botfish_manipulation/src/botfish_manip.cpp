#include <memory>
#include "manipulation/manipulation.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto manip_node = std::make_shared<manip::Manipulation>(options);
    exec.add_node(manip_node);
    exec.spin();
    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
