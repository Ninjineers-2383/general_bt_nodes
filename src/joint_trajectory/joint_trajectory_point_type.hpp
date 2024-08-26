#pragma once

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <string_view>
#include <behaviortree_cpp/contrib/json.hpp>

namespace BT
{
    template <>
    inline trajectory_msgs::msg::JointTrajectoryPoint convertFromString<trajectory_msgs::msg::JointTrajectoryPoint>(std::string_view str)
    {
        auto message = trajectory_msgs::msg::JointTrajectoryPoint();
        nlohmann::json values = nlohmann::json::parse(str);

        if (values.contains("accelerations"))
        {
            auto accelerations = values["accelerations"].template get<std::vector<double>>();
            message.set__accelerations(accelerations);
        }
        if (values.contains("velocities"))
        {
            auto velocities = values["velocities"].template get<std::vector<double>>();
            message.set__velocities(velocities);
        }
        if (values.contains("positions"))
        {
            auto positions = values["positions"].template get<std::vector<double>>();
            message.set__positions(positions);
        }

        if (values.contains("time_from_start"))
        {
            auto time = builtin_interfaces::msg::Duration();
            auto time_from_start = values["time_from_start"];
            if (time_from_start.contains("sec"))
            {
                auto seconds = time_from_start["sec"].template get<int>();
                time.set__sec(seconds);
            }
            if (time_from_start.contains("nanosec"))
            {
                auto nanoseconds = time_from_start["nanosec"].template get<int>();
                time.set__nanosec(nanoseconds);
            }
            message.set__time_from_start(time);
        }
        else
        {
            throw LogicError("Did not find time_from_start");
        }
        return message;
    }

    template <>
    inline std::vector<trajectory_msgs::msg::JointTrajectoryPoint>
    convertFromString<std::vector<trajectory_msgs::msg::JointTrajectoryPoint>>(std::string_view str)
    {
        nlohmann::json values = nlohmann::json::parse(str);
        if (!values.is_array())
        {
            std::stringstream stream;
            stream << "Value of JointTrajectoryPoint Vector is not a json array it is a ";
            stream << values.type_name();
            throw RuntimeError(stream.str());
        }

        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;

        for (auto &elem : values)
        {
            points.emplace_back(convertFromString<trajectory_msgs::msg::JointTrajectoryPoint>(elem.dump()));
        }

        return points;
    }
}