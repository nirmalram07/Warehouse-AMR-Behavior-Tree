#include "bt_turtlebot/behavior_control.hpp"

GoToPose::GoToPose(const std::string &name,
            const BT::NodeConfiguration &config,
            rclcpp::Node::SharedPtr node_ptr)
            : BT::StatefulActionNode(name, config), node_ptr(node_ptr)
{
    if (!node_ptr) {
        throw std::runtime_error("Invalid node pointer passed to GoToPose");
    }
    
    action_client_ptr = rclcpp_action::create_client<NavigateToPose>(node_ptr,"/navigate_to_pose");
    nav_done = false;
}

BT::PortsList GoToPose::providedPorts()
{
    return {BT::InputPort<std::string>("loc")};
}

BT::NodeStatus GoToPose::onStart()
{
    auto loc = getInput<std::string>("loc");
    const std::string location_file = node_ptr->get_parameter("location_file").as_string();

    YAML::Node locations = YAML::LoadFile(location_file);

    std::vector<float> pose = locations[loc.value()].as<std::vector<float>>();

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = pose[0];
    goal_msg.pose.pose.position.y = pose[1];

    tf2::Quaternion q;
    q.setRPY(0, 0, pose[2]);
    q.normalize();
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    nav_done = false;
    action_client_ptr->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_ptr->get_logger(), "Sent Goal to Nav2\n");

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToPose::onRunning(){
    
    if (nav_done)
    {
        RCLCPP_INFO(node_ptr->get_logger(), "[%s] Goal reached\n", this->name().c_str());
        return BT::NodeStatus::SUCCESS;
    }
    else{
        return BT::NodeStatus::RUNNING;
    }
}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
    if (result.result)
    {
        nav_done = true;
        RCLCPP_INFO(node_ptr->get_logger(),"Reached goal pose");
    }
}

ShipmentLoaded::ShipmentLoaded(const std::string &name)
            :BT::ConditionNode(name, {})
{}

BT::NodeStatus ShipmentLoaded::tick()
{

     std::default_random_engine gener_;
    std::uniform_int_distribution<int> distribution(0,1);

    int random_bool = distribution(gener_);

    if (random_bool==1){
        return BT::NodeStatus::SUCCESS;
    }
    else{
        return BT::NodeStatus::FAILURE;
    }
}

WaitForLoading::WaitForLoading(const std::string &name)
    : BT::SyncActionNode(name, {})
{}

BT::NodeStatus WaitForLoading::tick()
{
    std::this_thread::sleep_for(std::chrono::seconds(5));
    return BT::NodeStatus::SUCCESS;
}

