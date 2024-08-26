#include <behaviortree_ros2/plugins.hpp>
#include "parallel_race_all.hpp"
#include "parallel_deadline_all.hpp"

BT_REGISTER_ROS_NODES(factory, params)
{
    factory.registerNodeType<ParallelDeadlineAllNode>("ParallelDeadlineAll", params);
    factory.registerNodeType<ParallelRaceAllNode>("ParallelRaceAll", params);
}