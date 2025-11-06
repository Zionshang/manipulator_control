#include "fsm_state.hpp"

FSMState::FSMState(std::shared_ptr<ControlContext> ctrlContext,
                   FSMStateName state_name,
                   std::string state_name_string)
    : ctx_(ctrlContext),
      state_name(state_name),
      state_name_string(state_name_string) {}