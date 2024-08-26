#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include <control_msgs/srv/query_trajectory_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using QueryTrajectoryState = control_msgs::srv::QueryTrajectoryState;

class GetJointTrajectoryStateService : public BT::RosServiceNode<QueryTrajectoryState>
{
    using TrajectoryPoint = trajectory_msgs::msg::JointTrajectoryPoint;

public:
    GetJointTrajectoryStateService(const std::string &name,
                                   const BT::NodeConfig &conf,
                                   const BT::RosNodeParams &params)
        : BT::RosServiceNode<QueryTrajectoryState>(name, conf, params)
    {
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({BT::OutputPort<TrajectoryPoint>("trajectory_state", "Output port for the state of the trajectory"
                                                                                       " in type TrajectoryPoint")});
    }

    bool setRequest(typename Request::SharedPtr &request) override
    {
        (void)request;
        return true;
    }

    BT::NodeStatus onResponseReceived(const QueryTrajectoryState::Response::SharedPtr &response) override
    {
        auto point = TrajectoryPoint();
        point.positions = response->position;
        point.velocities = response->velocity;
        point.accelerations = response->acceleration;

        setOutput<TrajectoryPoint>("trajectory_state", point);

        return BT::NodeStatus::SUCCESS;
    }
};