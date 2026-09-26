#include "mrts_fleet_manager/fleet_core.hpp"

namespace mrts
{

    std::optional<std::string> assign_nearest_idle(
        const std::vector<Robot>& robots,
        const Task& task,
        std::function<std::optional<double>(const Pose2D&, const Pose2D&)> cost)
    {
        std::optional<std::string> best_id;

        // initializing as std::nullopt. Nullopt = no candidate yet, so any finite or infinite cost can win
        std::optional<double> best_val = std::nullopt;

        for (const auto& r : robots)
        {
            if (r.state != RobotState::Idle) continue;
            
            auto cost_val = cost(r.pose, task.pickup);

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

    double euclidean_sq(const Pose2D& from, const Pose2D& to)
    {
        double dx = from.x - to.x;
        double dy = from.y - to.y;
        return (dx * dx + dy * dy);
    }

    bool is_legal_transition(RobotState current_state, RobotState new_state)
    {
        switch (current_state)
        {   
        case RobotState::Idle:
            return new_state == RobotState::Moving;

        case RobotState::Moving:
            return (new_state == RobotState::Loading) || (new_state == RobotState::Unloading) || (new_state == RobotState::Fault);
        
        case RobotState::Loading:
            return new_state == RobotState::Moving;
        
        case RobotState::Unloading:
            return new_state == RobotState::Idle;

        case RobotState::Fault:
            return new_state == RobotState::Idle;

        default:
            return false;
        }
    }
}
