#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "yaml-cpp/yaml.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <string>
#include <vector>
#include <random>

class GoToPose : public BT::StatefulActionNode{
public:
    GoToPose(const std::string &name, const BT::NodeConfiguration &config,
             rclcpp::Node::SharedPtr node_ptr);

    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    rclcpp::Node::SharedPtr node_ptr;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_ptr;

    bool nav_done;

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override{};

    void nav_to_pose_callback(const GoalHandleNav::WrappedResult &result);
};

class ShipmentLoaded : public BT::ConditionNode{
public:
    ShipmentLoaded(const std::string &name);

    BT::NodeStatus tick() override;
};

class WaitForLoading : public BT::SyncActionNode{
public:
    WaitForLoading(const std::string &name);

    BT::NodeStatus tick() override;
};