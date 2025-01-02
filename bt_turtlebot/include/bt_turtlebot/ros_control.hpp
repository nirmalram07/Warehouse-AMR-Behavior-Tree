#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behavior_control.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

class RosControl : public rclcpp::Node{
    
public:
    explicit RosControl(const std::string &name);
    void setup();
    void initializing_behavior_tree();
    void update_behavior_tree();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree1;
};