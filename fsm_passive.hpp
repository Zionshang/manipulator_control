#pragma once

#include "fsm_state.hpp"

class StatePassive : public FSMState
{
public:
    StatePassive(std::shared_ptr<ControlContext> ctrl_context);
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    VectorDoF kd_;
};
