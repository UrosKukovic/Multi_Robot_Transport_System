#include "mrts_fleet_manager/fleet_core.hpp"
#include <limits>

namespace mrts
{

    std::optional<std::string> assign_nearest_idle(const std::vector<Robot>& robots, const Task& task,
        std::function<double(const Pose2D&, const Pose2D&)> cost)
    {
        std::optional<std::string> best_id;
        // initializing best val as the max double value and each next best value will be less than max of double
        double best_val = std::numeric_limits<double>::max();

        for (const auto& r : robots)
        {
            if (r.state != RobotState::Idle) continue;
            double cost_val = cost(r.pose, task.pickup);

            // first in list wins on equal cost
            if (!best_id.has_value() || cost_val < best_val)
            {
                best_id = r.id;
                best_val = cost_val;
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
}
