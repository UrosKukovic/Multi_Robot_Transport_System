#include <gtest/gtest.h>

#include "mrts_fleet_manager/fleet_core.hpp"

TEST(AssignNearestIdle, PicksRobotOnPickup)
{
  std::vector<mrts::Robot> robots
  {
    {"r1", mrts::RobotState::Idle, {0.0, 0.0}},
    {"r2", mrts::RobotState::Idle, {5.0, 0.0}},
    {"r3", mrts::RobotState::Idle, {1.0, 0.0}}
  };
  
  mrts::Task task
  {
    1,
    {0.0, 0.0},
    {10.0, 10.0}
  };

  auto id = mrts::assign_nearest_idle(robots, task, mrts::euclidean_sq);
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
  
  mrts::Task task
  {
    1,
    {0.0, 0.0},
    {10.0, 10.0}
  };

  auto id = mrts::assign_nearest_idle(robots, task, mrts::euclidean_sq);
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
  
  mrts::Task task
  {
    1,
    {0.0, 0.0},
    {10.0, 10.0}
  };

  auto id = mrts::assign_nearest_idle(robots, task, mrts::euclidean_sq);
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
  
  mrts::Task task
  {
    1,
    {0.0, 0.0},
    {10.0, 10.0}
  };

  auto id = mrts::assign_nearest_idle(robots, task, mrts::euclidean_sq);
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
  
  mrts::Task task
  {
    1,
    {0.0, 0.0},
    {10.0, 10.0}
  };

  auto id = mrts::assign_nearest_idle(robots, task, mrts::euclidean_sq);
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
  
  mrts::Task task
  {
    1,
    {0.0, 0.0},
    {10.0, 10.0}
  };

  auto id = mrts::assign_nearest_idle(robots, task, mrts::euclidean_sq);
  // id will be empty
  ASSERT_FALSE(id.has_value());
}

TEST(AssignNearestIdle, SkipEmptyFleet)
{
  std::vector<mrts::Robot> robots
  {};
  
  mrts::Task task
  {
    1,
    {0.0, 0.0},
    {10.0, 10.0}
  };

  auto id = mrts::assign_nearest_idle(robots, task, mrts::euclidean_sq);
  // id will be empty -> empty fleet
  ASSERT_FALSE(id.has_value());
}
