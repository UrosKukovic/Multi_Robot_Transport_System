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

    enum class Leg
    {
        HeadingToPickup,
        HeadingToDropoff
    };

    struct Robot
    {
        std::string id;
        RobotState state;
        Pose2D pose;
        Leg leg{Leg::HeadingToPickup};
    };

    struct Task
    {
        int id{};
        int failures{};
        Pose2D pickup;
        Pose2D dropoff;
    };

    // cost returns nullopt when no path exists; such robots are skipped.
    std::optional<std::string> assign_nearest_idle(const std::vector<Robot>& robots, const Task& task,
        std::function<std::optional<double>(const Pose2D&, const Pose2D&)> cost);
    double euclidean_sq(const Pose2D& from, const Pose2D& to);
    bool is_legal_transition(RobotState current_state, RobotState new_state);
    const char* to_string(mrts::RobotState);
}
