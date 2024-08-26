#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/plugins.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>

template <typename T>
class LogMessage : public BT::SyncActionNode
{
protected:
    rclcpp::Node::WeakPtr node_;

public:
    LogMessage(const std::string &name, const BT::NodeConfig &config, const BT::RosNodeParams &params)
        : SyncActionNode(name, config), node_{params.nh}
    {
    }

    BT::NodeStatus tick() override
    {
        T value;
        if (getInput("message", value))
        {
            if (auto node = node_.lock())
            {
                std::stringstream stream;
                stream << value;
                RCLCPP_INFO(node->get_logger(), stream.str().c_str());
                return BT::NodeStatus::SUCCESS;
            }
        }
        return BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<T>("message")};
    }
};

BT_REGISTER_ROS_NODES(factory, params)
{
    factory.registerNodeType<LogMessage<int>>("LogInt", params);
    factory.registerNodeType<LogMessage<double>>("LogDouble", params);
    factory.registerNodeType<LogMessage<std::string>>("LogString", params);
}