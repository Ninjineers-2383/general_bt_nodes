#include <behaviortree_ros2/bt_action_node.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class FollowJointTrajectoryAction : public BT::RosActionNode<control_msgs::action::FollowJointTrajectory>
{
    using JointTrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;

public:
    FollowJointTrajectoryAction(const std::string &name,
                                const BT::NodeConfig &conf,
                                const BT::RosNodeParams &params)
        : BT::RosActionNode<control_msgs::action::FollowJointTrajectory>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
            {BT::InputPort<std::vector<JointTrajectoryPoint>>("trajectory_points", "List of trajectory points to follow"),
             BT::InputPort<JointTrajectoryPoint>("initial_state", "Current ")});
    }

    bool setGoal(Goal &goal) override
    {
        std::vector<JointTrajectoryPoint> points;
        getInput("trajectory_points", points);
        JointTrajectoryPoint initial_state;
        getInput("initial_state", initial_state);

        points.emplace(points.begin(), initial_state);

        goal.trajectory.points = points;
        goal.trajectory.joint_names = {"pivot_block_to_hammer"};
        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult &wr) override
    {
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override
    {
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
    {
        return BT::NodeStatus::RUNNING;
    }
};