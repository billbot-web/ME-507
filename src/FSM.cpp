#include "FSM.h"

FSM::FSM(State** states, std::size_t count) noexcept
{
  states_        = states;
  count_         = count;
  // Default to state 0 when initialized (if available)
  if (states_ && count_ > 0) {
    curState_ = states_[0];
    curIdx_ = 0;
  } else {
    curState_ = nullptr;
    curIdx_ = -1;
  }
}

