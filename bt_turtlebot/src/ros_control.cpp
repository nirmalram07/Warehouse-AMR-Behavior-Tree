#include "bt_turtlebot/ros_control.hpp"
#include <chrono>

const std::string bt_xml_dir = ament_index_cpp::get_package_share_directory("bt_turtlebot") + "/config";

RosControl::RosControl(const std::string &name) : Node(name)
{
    this->declare_parameter("location_file","none");
    
    RCLCPP_INFO(get_logger(), "Initialization done....");
}

void RosControl::setup()
{
    RCLCPP_INFO(get_logger(), "Setting up behavior tree");
    initializing_behavior_tree();
    RCLCPP_INFO(get_logger(), "Behavior tree initialized");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&RosControl::update_behavior_tree, this)
    );

}

void RosControl::initializing_behavior_tree()
{
    BT::BehaviorTreeFactory factory;

    BT::NodeBuilder builder = 
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<GoToPose>(name, config, shared_from_this());
    };
    BT::NodeBuilder builder1 = 
        [=](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<ShipmentLoaded>(name, config, shared_from_this());
    };
    RCLCPP_INFO(get_logger(), "GoToPose node intialized");
    factory.registerBuilder<GoToPose>("GoToPose", builder);
    factory.registerBuilder<ShipmentLoaded>("ShipmentLoaded", builder1);
    factory.registerNodeType<WaitForLoading>("WaitForLoading");

    tree1 = factory.createTreeFromFile(bt_xml_dir + "/bt_tree.xml");
}

void RosControl::update_behavior_tree()
{
    BT::NodeStatus tree_status = tree1.tickOnce(); 

    if(tree_status == BT::NodeStatus::RUNNING){
        return;
    }
    else if(tree_status == BT::NodeStatus::SUCCESS){
        RCLCPP_INFO(this->get_logger(),"This node returned success");
        return;
    }
    else if(tree_status == BT::NodeStatus::FAILURE){
        RCLCPP_INFO(this->get_logger(),"This node returned failure");
        timer_->cancel();
    }
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosControl>("ros_control");
    node->setup();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}