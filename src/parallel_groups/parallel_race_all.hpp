#include <behaviortree_ros2/plugins.hpp>

using namespace BT;

class ParallelRaceAllNode : public BT::ControlNode
{
public:
    ParallelRaceAllNode(const std::string &name, const NodeConfig &config, const BT::RosNodeParams &params)
        : ControlNode::ControlNode(name, config), failure_threshold_(1), success_threshold_(1)
    {
        (void)params;
    }

    void halt() override
    {
        completed_list_.clear();
        failure_count_ = 0;
        success_count_ = 0;
        ControlNode::halt();
    }

    void setThreshold(int threshold, size_t &member)
    {
        if (threshold < 0)
        {
            member = size_t(std::max(int(children_nodes_.size()) + threshold + 1, 0));
        }
        else
        {
            member = size_t(threshold);
        }
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("max_failures", 1,
                                   "If the number of children returning FAILURE equals or exceeds this "
                                   "value, ParallelAll returns FAILURE"),
                BT::InputPort<int>("max_successes", 1,
                                   "If the number of children returning SUCCESS equals or exceeds this "
                                   "value, ParallelAll returns SUCCESS")};
    }

protected:
    size_t failure_threshold_;
    size_t success_threshold_;

    std::set<size_t> completed_list_;
    size_t failure_count_ = 0;
    size_t success_count_ = 0;

    NodeStatus tick() override
    {
        int max_failures = 0;
        if (!getInput("max_failures", max_failures))
        {
            throw RuntimeError("Missing parameter [max_failures] in ParallelNode");
        }
        int max_successes = 0;
        if (!getInput("max_successes", max_successes))
        {
            throw RuntimeError("Missing parameter [max_successes] in ParallelNode");
        }

        const size_t children_count = children_nodes_.size();
        setThreshold(max_failures, failure_threshold_);
        setThreshold(max_successes, success_threshold_);

        size_t skipped_count = 0;

        if (children_count < failure_threshold_ || children_count < success_threshold_)
        {
            throw LogicError("Number of children is less than threshold. Can either never fail or succeed.");
        }

        setStatus(NodeStatus::RUNNING);

        // Routing the tree according to the sequence node's logic:
        for (size_t index = 0; index < children_count; index++)
        {
            TreeNode *child_node = children_nodes_[index];

            // already completed
            if (completed_list_.count(index) != 0)
            {
                continue;
            }

            NodeStatus const child_status = child_node->executeTick();

            switch (child_status)
            {
            case NodeStatus::SUCCESS:
            {
                completed_list_.insert(index);
                success_count_++;
            }
            break;

            case NodeStatus::FAILURE:
            {
                completed_list_.insert(index);
                failure_count_++;
            }
            break;

            case NodeStatus::RUNNING:
            {
                // Still working. Check the next
            }
            break;

            case NodeStatus::SKIPPED:
            {
                skipped_count++;
            }
            break;

            case NodeStatus::IDLE:
            {
                throw LogicError("[", name(), "]: A children should not return IDLE");
            }
            }
        }

        if (skipped_count == children_count)
        {
            return NodeStatus::SKIPPED;
        }
        if (skipped_count + completed_list_.size() >= children_count || success_count_ >= success_threshold_)
        {
            // DONE
            haltChildren();
            completed_list_.clear();
            auto const status = (failure_count_ >= failure_threshold_) ? NodeStatus::FAILURE : NodeStatus::SUCCESS;
            failure_count_ = 0;
            success_count_ = 0;
            return status;
        }

        // Some children haven't finished, yet.
        return NodeStatus::RUNNING;
    }
};
