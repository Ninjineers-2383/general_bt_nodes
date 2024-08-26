#include <behaviortree_ros2/plugins.hpp>

#include "joint_trajectory_point_type.hpp"
#include "get_joint_trajectory_state.hpp"
#include "follow_joint_trajectory_action.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
    factory.registerNodeType<GetJointTrajectoryStateService>("GetJointTrajectoryState", params);
    factory.registerNodeType<FollowJointTrajectoryAction>("FollowJointTrajectory", params);
}