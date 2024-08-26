#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/plugins.hpp>
#include <drive_action_interfaces/action/drive_for_seconds.hpp>

using namespace BT;

class DriveForSecondsAction : public RosActionNode<drive_action_interfaces::action::DriveForSeconds>
{
public:
    DriveForSecondsAction(const std::string &name,
                          const NodeConfig &conf,
                          const RosNodeParams &params)
        : RosActionNode<drive_action_interfaces::action::DriveForSeconds>(name, conf, params)
    {
    }

    // The specific ports of this Derived class
    // should be merged with the ports of the base class,
    // using RosActionNode::providedBasicPorts()
    static PortsList providedPorts()
    {
        return providedBasicPorts({InputPort<double>("x", "0.0", ""),
                                   InputPort<double>("y", "0.0", ""),
                                   InputPort<double>("z", "0.0", ""),
                                   InputPort<double>("rx", "0.0", ""),
                                   InputPort<double>("ry", "0.0", ""),
                                   InputPort<double>("rz", "0.0", ""),
                                   InputPort<int>("seconds"),
                                   InputPort<std::string>("frame_id")});
    }

    // This is called when the TreeNode is ticked and it should
    // send the request to the action server
    bool setGoal(RosActionNode::Goal &goal) override
    {
        goal.twist.header.stamp = node_.lock()->get_clock()->now();
        // get "order" from the Input port
        getInput("x", goal.twist.twist.linear.x);
        getInput("y", goal.twist.twist.linear.y);
        getInput("x", goal.twist.twist.linear.z);
        getInput("rx", goal.twist.twist.angular.x);
        getInput("ry", goal.twist.twist.angular.y);
        getInput("rz", goal.twist.twist.angular.z);
        getInput("seconds", goal.seconds);
        getInput("frame_id", goal.twist.header.frame_id);
        // return true, if we were able to set the goal correctly.
        return true;
    }

    // Callback executed when the reply is received.
    // Based on the reply you may decide to return SUCCESS or FAILURE.
    NodeStatus onResultReceived(const WrappedResult &wr) override
    {
        auto node = node_.lock();
        std::stringstream ss;
        ss << "Result received: " << wr.result->success;
        RCLCPP_INFO(node->get_logger(), ss.str().c_str());
        return NodeStatus::SUCCESS;
    }

    // Callback invoked when there was an error at the level
    // of the communication between client and server.
    // This will set the status of the TreeNode to either SUCCESS or FAILURE,
    // based on the return value.
    // If not overridden, it will return FAILURE by default.
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override
    {
        RCLCPP_ERROR(node_.lock()->get_logger(), "Error: %d", error);
        return NodeStatus::FAILURE;
    }

    // we also support a callback for the feedback, as in
    // the original tutorial.
    // Usually, this callback should return RUNNING, but you
    // might decide, based on the value of the feedback, to abort
    // the action, and consider the TreeNode completed.
    // In that case, return SUCCESS or FAILURE.
    // The Cancel request will be send automatically to the server.
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Seconds left in movement: " << feedback->seconds_left;
        RCLCPP_INFO(node_.lock()->get_logger(), ss.str().c_str());
        return NodeStatus::RUNNING;
    }
};

CreateRosNodePlugin(DriveForSecondsAction, "DriveForSeconds");