#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <vector>
#include <map>

#include "mrts_fleet_manager/fleet_core.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class FleetManagerNode : public rclcpp::Node
{
public:
    FleetManagerNode() : Node("fleet_manager")
    {
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { dispatch(); });
    }

private:
    void dispatch()
    {
        timer_->cancel();
        if (!client_->wait_for_action_server(std::chrono::milliseconds(500)))
        {
            RCLCPP_INFO(get_logger(), "Nav2 not available");
            return;
        }

        auto best_id = mrts::assign_nearest_idle(robots_, task_, mrts::euclidean_sq);

        if (best_id == std::nullopt)
        {
            RCLCPP_INFO(get_logger(), "No idle robots");
            return;
        }

        auto robot = find_robot(*best_id);

        if (robot == nullptr)
        {
            RCLCPP_ERROR(get_logger(), "No found robots!");
            return;
        }
        
        robot->leg = mrts::Leg::HeadingToPickup;

        bool state_set = set_state(robot->id, mrts::RobotState::Moving);

        if (!state_set)
        {
            RCLCPP_ERROR(get_logger(), "State set unsuccessful, a goal wasn't sent.");
            return;
        }

        send_nav_goal(robot->id, task_.pickup);
    }

    void dropoff(const std::string id)
    {
        // using .at() since it will throw std::out_of_range if the key is missing, since map inserts nullptr by default for shared_ptr,
        // we could call cancel() on a nullptr.
        work_timers_.at(id)->cancel();

        auto robot = find_robot(id);

        if (robot == nullptr)
        {
            RCLCPP_ERROR(get_logger(), "No found robots for dropoff!");
            return;
        }

        if (!client_->wait_for_action_server(std::chrono::milliseconds(500)))
        {
            RCLCPP_INFO(get_logger(), "Nav2 not available");

            // Loading->Fault
            bool state_set = set_state(robot->id, mrts::RobotState::Fault);

            if (!state_set)
            {
                RCLCPP_ERROR(get_logger(), "State set unsuccessful!");
            }
            
            return;
        }

        robot->leg = mrts::Leg::HeadingToDropoff;

        bool state_set = set_state(robot->id, mrts::RobotState::Moving);

        if (!state_set)
        {
            RCLCPP_ERROR(get_logger(), "State set unsuccessful, a goal wasn't sent.");
            return;
        }

        send_nav_goal(robot->id, task_.dropoff);
    }

    mrts::Robot* find_robot(const std::string& id)
    {
        for (auto& r : robots_)
        {
            if (id == r.id)
                return &r;
        }

        return nullptr;
    }

    bool set_state(const std::string& id, mrts::RobotState new_state)
    {
        auto robot = find_robot(id);

        if (robot == nullptr)
        {
            RCLCPP_ERROR(get_logger(), "No found robots!");
            return false;
        }

        auto old_state = robot->state;

        if (!mrts::is_legal_transition(old_state, new_state))
        {
            RCLCPP_ERROR(get_logger(), "Illegal transition: %s -> %s", mrts::to_string(old_state), mrts::to_string(new_state));
            return false;
        }
        
        robot->state = new_state;
        RCLCPP_INFO(get_logger(), "%s: %s -> %s", id.c_str(), mrts::to_string(old_state), mrts::to_string(new_state));
        return true;
    }

    NavigateToPose::Goal make_goal(const mrts::Pose2D& p)
    {
        // Notes for me so that I can understand things:
        // If we want
        // since x = a_x * sin(theta/2), y = a_y * sin(theta/2), z = a_z * sin(theta/2), w = cos(theta/2)
        // if we want to keep rotation 0 about Z axis, we have to set x,y,z = 0 and w = 1.
        // goal.pose.pose.orientation.w is 1.0 as default
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = now();
        goal.pose.pose.position.x = p.x;
        goal.pose.pose.position.y = p.y;
        
        return goal;
    }

    void send_nav_goal(const std::string& id, const mrts::Pose2D& target)
    {
        auto goal = make_goal(target);

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

        options.goal_response_callback = [this, id](GoalHandle::SharedPtr gh)
        {
            if (gh == nullptr)
            {
                RCLCPP_INFO(get_logger(), "Goal rejected");

                auto robot = find_robot(id);
                if (robot == nullptr)
                    return;

                if (robot->leg == mrts::Leg::HeadingToDropoff)
                {
                    // The robot is loaded. Rejected means Moving -> Fault because it was rejected on its way to dropoff
                    set_state(id, mrts::RobotState::Fault);
                }
                else
                {
                    // Robot is not loaded - task was rejected on its way to pickup (Moving -> Idle)
                    set_state(id, mrts::RobotState::Idle);
                }
            }
            else
                RCLCPP_INFO(get_logger(), "Goal accepted");
        };

        options.result_callback = [this, id](const GoalHandle::WrappedResult& result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
            {
                RCLCPP_INFO(get_logger(), "SUCCEEDED");
                auto robot = find_robot(id);
                if (robot == nullptr)
                    break;

                if (robot->leg == mrts::Leg::HeadingToPickup)
                {
                    set_state(id, mrts::RobotState::Loading);
                    work_timers_[id] = create_wall_timer(std::chrono::seconds(3), [this, id](){ dropoff(id); });
                }

                else
                {
                    set_state(id, mrts::RobotState::Unloading);
                    // I'm not sure if i can use the same timer.
                    work_timers_[id] = create_wall_timer(std::chrono::seconds(3), [this, id](){
                        work_timers_.at(id)->cancel();
                        auto robot = find_robot(id);

                        if (robot == nullptr)
                        {
                            RCLCPP_ERROR(get_logger(), "No found robots!");
                            return;
                        }

                        set_state(id, mrts::RobotState::Idle);
                        robot->leg = mrts::Leg::HeadingToPickup;
                    });
                }
                break;
            }
            case rclcpp_action::ResultCode::ABORTED:
            {
                RCLCPP_INFO(get_logger(), "ABORTED");
                set_state(id, mrts::RobotState::Fault);
                break;
            }
            case rclcpp_action::ResultCode::CANCELED:
            {
                RCLCPP_INFO(get_logger(), "CANCELED");

                auto robot = find_robot(id);
                if (robot == nullptr)
                    break;

                if (robot->leg == mrts::Leg::HeadingToDropoff)
                {
                    // The robot is loaded. Cancelled means Moving -> Fault
                    set_state(id, mrts::RobotState::Fault);
                }
                else
                {
                    // Robot is not loaded - task was cancelled on its way to pickup (Moving -> Idle)
                    set_state(id, mrts::RobotState::Idle);
                }
                break;
            }
            default:
                RCLCPP_INFO(get_logger(), "INVALID");
                break;
            }

        };

        options.feedback_callback = [this](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> fb)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "distance remaining: %.4f", fb->distance_remaining);
        };

        client_->async_send_goal(goal, options);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::map<std::string, rclcpp::TimerBase::SharedPtr> work_timers_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    std::vector<mrts::Robot> robots_
    {
        {"robot1", mrts::RobotState::Idle, {0.0, 0.0}}
    };
    mrts::Task task_
    {
        1,
        {2.0, 1.0},
        // {2.2, 3.3},
        {3.5, -1.0}
    };
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FleetManagerNode>();
    RCLCPP_INFO(node->get_logger(), "I'm alive");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}