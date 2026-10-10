#include "mrts_fleet_manager/fleet_core.hpp"
#include <cmath>

namespace mrts
{

    std::optional<std::string> assign_nearest_idle(
        const std::vector<Robot>& robots,
        const Task& task,
        std::function<std::optional<double>(const Robot&, const Pose2D&)> cost)
    {
        std::optional<std::string> best_id;

        // initializing as std::nullopt. Nullopt = no candidate yet, so any finite or infinite cost can win
        std::optional<double> best_val = std::nullopt;

        for (const auto& r : robots)
        {
            if (r.state != RobotState::Idle) continue;
            
            auto cost_val = cost(r, task.pickup);

            if (cost_val == std::nullopt) continue; 

            // first in list wins on equal cost
            if (!best_val || ( *cost_val < *best_val ))
            {
                best_id = r.id;
                best_val = *cost_val;
            }
        }

        return best_id;
    }

    // Wrapper for euclidean_sq
    std::optional<double> euclidean_cost(const Robot& r, const Pose2D& to)
    {
        return euclidean_sq(r.pose, to);
    }

    double euclidean_sq(const Pose2D& from, const Pose2D& to)
    {
        double dx = from.x - to.x;
        double dy = from.y - to.y;
        return (dx * dx + dy * dy);
    }

    double path_length(const std::vector<Pose2D>& points)
    {
        double dist{};

        for (std::size_t i = 1; i < points.size(); ++i)
        {
            double dx = points[i].x - points[i-1].x;
            double dy = points[i].y - points[i-1].y;
            double seg_dist = std::hypot(dx, dy);
            dist+=seg_dist;
        }

        return dist;
    }

    bool is_legal_transition(RobotState current_state, RobotState new_state)
    {
        switch (current_state)
        {   
        case RobotState::Idle:
            return (new_state == RobotState::Moving);

        case RobotState::Moving:
            return (new_state == RobotState::Loading) || (new_state == RobotState::Unloading) || (new_state == RobotState::Fault) || (new_state == RobotState::Idle);
        
        case RobotState::Loading:
            return (new_state == RobotState::Moving) || (new_state == RobotState::Fault);
        
        case RobotState::Unloading:
            return new_state == RobotState::Idle;

        case RobotState::Fault:
            return new_state == RobotState::Idle;

        default:
            return false;
        }
    }

    const char* to_string(mrts::RobotState s)
    {
        switch (s)
        {
        case RobotState::Fault:
            return "Fault";

        case RobotState::Idle:
            return "Idle";

        case RobotState::Loading:
            return "Loading";

        case RobotState::Moving:
            return "Moving";

        case RobotState::Unloading:
            return "Unloading";
        }

        return "Unknown";
    }

    AbortAction decide_abort(bool bad_goal, Leg leg, int current_failures, int max_failures)
    {
        if (leg == mrts::Leg::HeadingToDropoff) return AbortAction::FaultKeepTask;
        if (bad_goal) return AbortAction::IdleDropTask;
        if (current_failures >= max_failures) return AbortAction::FaultDropTask;

        return AbortAction::FaultRequeueTask;
    }
}
