#include <gtest/gtest.h>

#include "mrts_fleet_manager/fleet_core.hpp"

// Helpers
mrts::Task make_task(int id, int failures, mrts::Pose2D pickup, mrts::Pose2D dropoff)
{
  return mrts::Task{
    .id = id,
    .failures = failures,
    .pickup = pickup,
    .dropoff = dropoff
  };
}

TEST(AssignNearestIdle, PicksRobotOnPickup)
{
  std::vector<mrts::Robot> robots
  {
    {"r1", mrts::RobotState::Idle, {0.0, 0.0}},
    {"r2", mrts::RobotState::Idle, {5.0, 0.0}},
    {"r3", mrts::RobotState::Idle, {1.0, 0.0}}
  };
  
  mrts::Task task = make_task(1, 0, {0.0, 0.0}, {10.0, 10.0});

  auto id = mrts::assign_nearest_idle(robots, task, mrts::euclidean_cost);
  ASSERT_TRUE(id.has_value());
  // Should fail with the intentional bug
  EXPECT_EQ(*id, "r1");
}

TEST(AssignNearestIdle, TieGoesToFirstInList)
{
  std::vector<mrts::Robot> robots
  {
    {"r1", mrts::RobotState::Idle, {5.0, 0.0}},
    {"r2", mrts::RobotState::Idle, {5.0, 0.0}}
  };
  
  mrts::Task task = make_task(1, 0, {0.0, 0.0}, {10.0, 10.0});

  auto id = mrts::assign_nearest_idle(robots, task, mrts::euclidean_cost);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(*id, "r1");
}

TEST(AssignNearestIdle, PicksNearerOfTwoIdle)
{
  std::vector<mrts::Robot> robots
  {
    {"r1", mrts::RobotState::Idle, {5.0, 0.0}},
    {"r2", mrts::RobotState::Idle, {1.0, 0.0}}
  };
  
  mrts::Task task = make_task(1, 0, {0.0, 0.0}, {10.0, 10.0});

  auto id = mrts::assign_nearest_idle(robots, task, mrts::euclidean_cost);
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(*id, "r2");
}

TEST(AssignNearestIdle, SkipNearerMoving)
{
  std::vector<mrts::Robot> robots
  {
    {"r1", mrts::RobotState::Idle, {5.0, 0.0}},
    {"r2", mrts::RobotState::Moving, {1.0, 0.0}}
  };
  
  mrts::Task task = make_task(1, 0, {0.0, 0.0}, {10.0, 10.0});

  auto id = mrts::assign_nearest_idle(robots, task, mrts::euclidean_cost);
  ASSERT_TRUE(id.has_value());
  // skip r2 since its state is "Moving"
  EXPECT_EQ(*id, "r1");
}

TEST(AssignNearestIdle, SkipNearerFault)
{
  std::vector<mrts::Robot> robots
  {
    {"r1", mrts::RobotState::Idle, {5.0, 0.0}},
    {"r2", mrts::RobotState::Fault, {1.0, 0.0}}
  };
  
  mrts::Task task = make_task(1, 0, {0.0, 0.0}, {10.0, 10.0});

  auto id = mrts::assign_nearest_idle(robots, task, mrts::euclidean_cost);
  ASSERT_TRUE(id.has_value());
  // skip r2 since its state is "Fault"
  EXPECT_EQ(*id, "r1");
}

TEST(AssignNearestIdle, NoIdleRobotReturnsNullopt)
{
  std::vector<mrts::Robot> robots
  {
    {"r1", mrts::RobotState::Moving, {5.0, 0.0}},
    {"r2", mrts::RobotState::Moving, {1.0, 0.0}}
  };
  
  mrts::Task task = make_task(1, 0, {0.0, 0.0}, {10.0, 10.0});

  auto id = mrts::assign_nearest_idle(robots, task, mrts::euclidean_cost);
  // id will be empty
  ASSERT_FALSE(id.has_value());
}

TEST(AssignNearestIdle, SkipEmptyFleet)
{
  std::vector<mrts::Robot> robots
  {};
  
  mrts::Task task = make_task(1, 0, {0.0, 0.0}, {10.0, 10.0});

  auto id = mrts::assign_nearest_idle(robots, task, mrts::euclidean_cost);
  // id will be empty -> empty fleet
  ASSERT_FALSE(id.has_value());
}

TEST(AssignNearestIdle, SkipUnreachableNearer)
{
  std::vector<mrts::Robot> robots
  {
    {"r1", mrts::RobotState::Idle, {5.0, 0.0}},
    {"r2", mrts::RobotState::Idle, {1.0, 0.0}}
  };
  
  mrts::Task task = make_task(1, 0, {0.0, 0.0}, {10.0, 10.0});

  auto lmbd = [](const mrts::Robot& r, const mrts::Pose2D& to) -> std::optional<double>
  {
    if (r.id == "r2") return std::nullopt;

    auto cost_val = mrts::euclidean_sq(r.pose, to);
    return cost_val;
  };

  auto id = mrts::assign_nearest_idle(robots, task, lmbd);
  ASSERT_TRUE(id.has_value());
  // skip r2 since it is "unreachable" even if closer -> returns nullopt
  EXPECT_EQ(*id, "r1");
}

TEST(AssignNearestIdle, AllUnreachableReturnsNullopt)
{
  std::vector<mrts::Robot> robots
  {
    {"r1", mrts::RobotState::Idle, {5.0, 0.0}},
    {"r2", mrts::RobotState::Idle, {1.0, 0.0}}
  };
  
  mrts::Task task = make_task(1, 0, {0.0, 0.0}, {10.0, 10.0});

  auto lmbd = [](const mrts::Robot&, const mrts::Pose2D&) -> std::optional<double>
  {
    // both Idle robots are unreachable
    return std::nullopt;
  };

  auto id = mrts::assign_nearest_idle(robots, task, lmbd);
  // return is nullopt -> no idle robots were assigned --> both unreachable
  ASSERT_FALSE(id.has_value());
}

TEST(PathLength, LShapeIsSumOfSegments)
{
  std::vector<mrts::Pose2D> points
  {
    {0, 0},
    {3, 0},
    {3, 4}
  };

  double dist = mrts::path_length(points);

  EXPECT_DOUBLE_EQ(dist, 7.0);
}

TEST(IsTransitionLegal, TransitionLegal)
{
  auto is_legal_Idle_Moving = mrts::is_legal_transition(mrts::RobotState::Idle, mrts::RobotState::Moving);
  auto is_legal_Moving_Fault = mrts::is_legal_transition(mrts::RobotState::Moving, mrts::RobotState::Fault);
  auto is_legal_Moving_Idle = mrts::is_legal_transition(mrts::RobotState::Moving, mrts::RobotState::Idle);  
  auto is_legal_Loading_Fault = mrts::is_legal_transition(mrts::RobotState::Loading, mrts::RobotState::Fault);  

  EXPECT_TRUE(is_legal_Idle_Moving);
  EXPECT_TRUE(is_legal_Moving_Fault);
  EXPECT_TRUE(is_legal_Moving_Idle);
  EXPECT_TRUE(is_legal_Loading_Fault);
}

TEST(IsTransitionLegal, TransitionIllegalLoadingUnloading)
{
  auto is_legal_Loading_Unloading = mrts::is_legal_transition(mrts::RobotState::Loading, mrts::RobotState::Unloading);
  auto is_legal_Idle_Idle = mrts::is_legal_transition(mrts::RobotState::Idle, mrts::RobotState::Idle);

  EXPECT_FALSE(is_legal_Loading_Unloading);
  EXPECT_FALSE(is_legal_Idle_Idle);
}

TEST(IsTransitionLegal, TransitionIllegalFaultMoving)
{
  auto is_legal = mrts::is_legal_transition(mrts::RobotState::Fault, mrts::RobotState::Moving);

  ASSERT_FALSE(is_legal);
}
