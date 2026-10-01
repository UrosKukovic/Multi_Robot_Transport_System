#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <mrts_interfaces/srv/submit_task.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <vector>
#include <map>
#include <deque>

#include "mrts_fleet_manager/fleet_core.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using PoseMsg = geometry_msgs::msg::PoseWithCovarianceStamped;
using SubmitTask = mrts_interfaces::srv::SubmitTask;
using PlanResult = nav2_msgs::action::ComputePathToPose::Result;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using ComputePath = nav2_msgs::action::ComputePathToPose;
using PlanGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;

// ANSI terminal colours for INFO lines for readability
constexpr const char* GREEN = "\033[32m";
constexpr const char* CYAN = "\033[36m";
constexpr const char* RESET = "\033[0m";

class FleetManagerNode : public rclcpp::Node
{
public:
    FleetManagerNode() : Node("fleet_manager")
    {
        for (auto& r : robots_)
        {
            clients_[r.id] = rclcpp_action::create_client<NavigateToPose>(this, "/"+ r.id + "/navigate_to_pose");
            compute_path_clients_[r.id] = rclcpp_action::create_client<ComputePath>(this, "/"+ r.id + "/compute_path_to_pose");

            pose_subs_[r.id] = create_subscription<PoseMsg>(
                "/" + r.id + "/amcl_pose",
                rclcpp::QoS(1).transient_local().reliable(),
                [this, id = r.id](const PoseMsg::ConstSharedPtr msg)
                {
                    auto robot = find_robot(id);

                    if (robot == nullptr)
                    {
                        RCLCPP_ERROR(get_logger(), "%s: amcl_pose for unknown robot", id.c_str());
                        return;
                    }

                    robot->pose.x = msg->pose.pose.position.x;
                    robot->pose.y = msg->pose.pose.position.y;
                });
        }
        submit_srv_ = create_service<SubmitTask>(
            "~/submit_task",
            [this](const std::shared_ptr<SubmitTask::Request> req,
                    std::shared_ptr<SubmitTask::Response> res)
            {
                const mrts::Pose2D pickup{req->pickup_x, req->pickup_y};
                const mrts::Pose2D dropoff{req->dropoff_x, req->dropoff_y};
                // closer than this is the same spot for Nav2
                constexpr double min_dist = 0.1;

                if (mrts::euclidean_sq(pickup, dropoff) < min_dist * min_dist)
                {
                    res->accepted = false;
                    res->message = "Pickup and dropoff are closer than 0.1 m. Pickup: (" +
                        std::to_string(req->pickup_x) + ", " + std::to_string(req->pickup_y) +
                        "); Dropoff: (" +
                        std::to_string(req->dropoff_x) + ", " + std::to_string(req->dropoff_y) + ")";
                    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
                    return;
                }

                mrts::Task task;
                task.id = next_task_id_;
                task.pickup = pickup;
                task.dropoff = dropoff;
                pending_tasks_.push_back(task);
                next_task_id_++;

                res->accepted = true;
                res->task_id = task.id;
                res->message = "Queued as task " + std::to_string(task.id) + ", " + std::to_string(pending_tasks_.size()) + " pending";
                RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
            });
        timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { dispatch(); });
    }

private:
    void dispatch()
    {
        // check if any tasks are pending
        if (pending_tasks_.empty())
            return;

        const auto& task = pending_tasks_.front();
        
        // No planners asked yet
        if (!path_request_)
        {
            // ask all planners
            request_paths(task);
            return;
        }

        bool received_all_requests = ( path_request_->sent == path_request_->requests.size() );

        bool timed_out = ( (now() - path_request_->started).seconds() > 2.0 );

        // Didn't recieve from every planners nor timed out
        if (!received_all_requests && !timed_out) return;

        auto lmbd = [this](const mrts::Robot& r, const mrts::Pose2D&) -> std::optional<double>
        {
            auto it = path_request_->requests.find(r.id);
            if (it != path_request_->requests.end())
            {
                return it->second;
            }
            else
            {
                return std::nullopt;
            }
        };

        bool any_answer = !path_request_->requests.empty();

        auto best_id = mrts::assign_nearest_idle(robots_, task, lmbd);
        path_request_.reset();

        if (best_id == std::nullopt)
        {
            if (!any_answer)
                RCLCPP_INFO(get_logger(), "No planner answered");
            
            else
            {
                RCLCPP_ERROR(get_logger(), "Task %d unreachable from every robot", pending_tasks_.front().id);
                pending_tasks_.pop_front();
            }

            return;
        }

        auto robot = find_robot(*best_id);

        if (robot == nullptr)
        {
            RCLCPP_ERROR(get_logger(), "No found robots!");
            return;
        }

        if (!clients_.at(*best_id)->action_server_is_ready())
        {
            RCLCPP_INFO(get_logger(), "Nav2 not available");
            return;
        }
        
        robot->leg = mrts::Leg::HeadingToPickup;

        bool state_set = set_state(robot->id, mrts::RobotState::Moving);

        if (!state_set)
        {
            RCLCPP_ERROR(get_logger(), "State set unsuccessful, a goal wasn't sent.");
            return;
        }

        active_tasks_[robot->id] = task;
        pending_tasks_.pop_front();

        send_nav_goal(robot->id, active_tasks_[robot->id].pickup);
    }

    void dropoff(const std::string id)
    {
        // using .at() since it will throw std::out_of_range if the key is missing, since map inserts nullptr by default for shared_ptr,
        // we could call cancel() on a nullptr.
        work_timers_.at(id)->cancel();
     
        auto robot = find_robot(id);

        if (robot == nullptr)
        {
            RCLCPP_ERROR(get_logger(), "%s: amcl_pose for unknown robot", id.c_str());
            return;
        }

        if (!clients_.at(id)->wait_for_action_server(std::chrono::milliseconds(500)))
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

        send_nav_goal(robot->id, active_tasks_.at(id).dropoff);
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
        
        if (new_state == mrts::RobotState::Fault)
            RCLCPP_ERROR(get_logger(), "%s: %s -> %s", id.c_str(), mrts::to_string(old_state), mrts::to_string(new_state));        

        else
            RCLCPP_INFO(get_logger(), "%s%s: %s -> %s%s", CYAN, id.c_str(), mrts::to_string(old_state), mrts::to_string(new_state), RESET);
        
        return true;
    }

    void request_paths(const mrts::Task& task)
    {
        path_request_ = PathRequest
        {
            .task_id = task.id,
            .started = now(),
            .requests{}
        };

        for (auto& r : robots_)
        {
            if ( (r.state == mrts::RobotState::Idle) && (compute_path_clients_.at(r.id)->action_server_is_ready()) )
            {
                // build a goal
                ComputePath::Goal request;
                request.goal = make_goal(task.pickup).pose;

                rclcpp_action::Client<ComputePath>::SendGoalOptions options;

                options.goal_response_callback = [this, id = r.id, task_id = task.id](PlanGoalHandle::SharedPtr gh)
                {
                    if (!path_request_ || path_request_->task_id != task_id) return;

                    if (gh == nullptr)
                    {
                        RCLCPP_INFO(get_logger(), "%s: plan request rejected", id.c_str());

                        path_request_->requests[id] = std::nullopt;
                    }
                };

                options.result_callback = [this, id = r.id, task_id = task.id](const PlanGoalHandle::WrappedResult& result)
                {
                    if (!path_request_ || path_request_->task_id != task_id) return;

                    if ( (result.code == rclcpp_action::ResultCode::SUCCEEDED) && !result.result->path.poses.empty() )
                    {
                        std::vector<mrts::Pose2D> points;
                        for (const auto& p : result.result->path.poses)
                        {
                            points.push_back(
                                {
                                    .x = p.pose.position.x,
                                    .y = p.pose.position.y
                                }
                            );
                        }

                        double len = mrts::path_length(points);

                        path_request_->requests[id] = len;
                        RCLCPP_INFO(get_logger(), "robot id: %s; len: %f", id.c_str(), len);
                    }

                    else
                    {
                        path_request_->requests[id] = std::nullopt;
                        RCLCPP_WARN(get_logger(), "%s: no path", id.c_str());
                    }
                };

                // Send a goal
                compute_path_clients_.at(r.id)->async_send_goal(request, options);
                path_request_->sent++;
            }
        }

        if (path_request_->sent == 0)
        {
            path_request_.reset();
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "No idle robot with a ready planner");
        }
    }

    NavigateToPose::Goal make_goal(const mrts::Pose2D& p)
    {
        // Notes for me so that I can understand things:
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
                RCLCPP_INFO(get_logger(), "%s: Goal rejected", id.c_str());

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
                    pending_tasks_.push_back(active_tasks_[id]);
                    active_tasks_.erase(id);
                }
            }
            else
                RCLCPP_INFO(get_logger(), "%s: Goal accepted", id.c_str());
        };

        options.result_callback = [this, id](const GoalHandle::WrappedResult& result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
            {
                RCLCPP_INFO(get_logger(), "%s%s: SUCCEEDED%s", GREEN, id.c_str(), RESET);
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
                    work_timers_[id] = create_wall_timer(std::chrono::seconds(3), [this, id](){
                        work_timers_.at(id)->cancel();
                        auto robot = find_robot(id);

                        if (robot == nullptr)
                        {
                            RCLCPP_ERROR(get_logger(), "No found robots!");
                            return;
                        }

                        set_state(id, mrts::RobotState::Idle);
                        RCLCPP_INFO(get_logger(), "%s: Idle at (%.2f, %.2f)", id.c_str(), robot->pose.x, robot->pose.y);
                        active_tasks_.erase(id);
                        robot->leg = mrts::Leg::HeadingToPickup;
                    });
                }
                break;
            }
            case rclcpp_action::ResultCode::ABORTED:
            {
                auto robot = find_robot(id);
                if (robot == nullptr)
                    break;

                RCLCPP_WARN(get_logger(), "%s: ABORTED", id.c_str());
                RCLCPP_WARN(get_logger(), "error_code: %d; error_msg: %s", result.result->error_code, result.result->error_msg.c_str());
                
                if ( ((result.result->error_code == PlanResult::GOAL_OUTSIDE_MAP) || 
                    (result.result->error_code == PlanResult::GOAL_OCCUPIED)) &&
                    robot->leg != mrts::Leg::HeadingToDropoff )
                {
                    // Broken task
                    set_state(id, mrts::RobotState::Idle);
                    RCLCPP_ERROR(get_logger(), "task %d failed: %s", active_tasks_.at(id).id, result.result->error_msg.c_str());
                    active_tasks_.erase(id);
                }    

                else
                {
                    // Robot failed due to being stuck or got NO_VALID_PATH back
                    bool is_state_set = set_state(id, mrts::RobotState::Fault);

                    if (!is_state_set)
                        break;

                    
                    if ( robot->leg == mrts::Leg::HeadingToPickup )
                    {
                        auto& task = active_tasks_.at(id);
                        task.failures++;

                        if (task.failures >= max_task_failures)
                        {
                            // Failed on several robots: blame the task, not the robots
                            RCLCPP_ERROR(get_logger(), "task %d dropped: failed on %d robots, presumed unreachable", task.id, task.failures);
                            active_tasks_.erase(id);
                        }

                        else
                        {
                            // One failure can't tell robot from task: requeue behind the healthy work
                            RCLCPP_WARN(get_logger(), "task %d requeued after failure %d/%d", task.id, task.failures, max_task_failures);
                            pending_tasks_.push_back(task);
                            active_tasks_.erase(id);
                        }
                    }
                }
                break;
            }
            case rclcpp_action::ResultCode::CANCELED:
            {
                RCLCPP_INFO(get_logger(), "%s: CANCELED", id.c_str());

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
                    pending_tasks_.push_back(active_tasks_[id]);
                    active_tasks_.erase(id);
                }
                break;
            }
            default:
                RCLCPP_INFO(get_logger(), "INVALID");
                break;
            }

        };

        options.feedback_callback = [this, id](GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> fb)
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s: distance remaining: %.4f", id.c_str(), fb->distance_remaining);
        };

        clients_.at(id)->async_send_goal(goal, options);
    }

    struct PathRequest
    {
        int task_id{};
        rclcpp::Time started;
        std::size_t sent{};
        std::map<std::string, std::optional<double>> requests;
    };

    rclcpp::TimerBase::SharedPtr timer_;
    std::map<std::string, rclcpp::TimerBase::SharedPtr> work_timers_;
    std::map<std::string, rclcpp_action::Client<NavigateToPose>::SharedPtr> clients_;
    std::map<std::string, rclcpp_action::Client<ComputePath>::SharedPtr> compute_path_clients_;
    std::map<std::string, rclcpp::Subscription<PoseMsg>::SharedPtr> pose_subs_;
    rclcpp::Service<SubmitTask>::SharedPtr submit_srv_;
    int next_task_id_{1};
    // A task is dropped once it has failed on this many robots
    static constexpr int max_task_failures = 2;
    std::deque<mrts::Task> pending_tasks_;
    std::map<std::string, mrts::Task> active_tasks_;
    std::optional<PathRequest> path_request_;
    std::vector<mrts::Robot> robots_
    {
        {"robot1", mrts::RobotState::Idle, {0.0, 0.0}},
        {"robot2", mrts::RobotState::Idle, {1.5, 0.0}}
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