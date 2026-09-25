#pragma once
#include <string>
#include <optional>
#include <vector>
#include <functional>

namespace mrts
{
    struct Pose2D
    {
        double x{};
        double y{};
    };

    enum class RobotState
    {
        Idle,
        Moving,
        Loading,
        Unloading,
        Fault
    };

    struct Robot
    {
        std::string id;
        RobotState state;
        Pose2D pose;
    };

    struct Task
    {
        int id;
        Pose2D pickup;
        Pose2D dropoff;
    };

    std::optional<std::string> assign_nearest_idle(const std::vector<Robot>& robots, const Task& task,
        std::function<double(const Pose2D&, const Pose2D&)> cost);
    double euclidean_sq(const Pose2D& from, const Pose2D& to);
}
