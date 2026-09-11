#pragma once
#include <string>
namespace robot_tasks {
enum class State { IDLE, PATROL, PAUSED, RETURNING_HOME };
inline State transition(State state, const std::string & command) {
  if (command == "patrol") return (state == State::PATROL || state == State::PAUSED) ? state : State::PATROL;
  if (command == "pause") return state == State::PATROL ? State::PAUSED : state;
  if (command == "resume") return state == State::PAUSED ? State::PATROL : state;
  if (command == "home") return State::RETURNING_HOME;
  if (command == "stop") return State::IDLE;
  return state;
}
}
