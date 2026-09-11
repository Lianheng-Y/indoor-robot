#include <gtest/gtest.h>
#include "robot_tasks/task_policy.hpp"

TEST(TaskPolicy, PatrolPauseResumeAndStop) {
  using robot_tasks::State;
  EXPECT_EQ(robot_tasks::transition(State::IDLE, "patrol"), State::PATROL);
  EXPECT_EQ(robot_tasks::transition(State::PATROL, "patrol"), State::PATROL);
  EXPECT_EQ(robot_tasks::transition(State::PATROL, "pause"), State::PAUSED);
  EXPECT_EQ(robot_tasks::transition(State::PAUSED, "resume"), State::PATROL);
  EXPECT_EQ(robot_tasks::transition(State::PATROL, "stop"), State::IDLE);
}
TEST(TaskPolicy, HomePreemptsAnyState) {
  using robot_tasks::State;
  EXPECT_EQ(robot_tasks::transition(State::PATROL, "home"), State::RETURNING_HOME);
  EXPECT_EQ(robot_tasks::transition(State::PAUSED, "home"), State::RETURNING_HOME);
}
