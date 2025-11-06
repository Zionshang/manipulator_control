#pragma once

#include "control_context.hpp"
#include <string>
#include <memory>

class FSMState
{
public:
    FSMState(std::shared_ptr<ControlContext> ctrlContext,
             FSMStateName state_name,
             std::string state_name_string);

    virtual void enter() = 0;
    virtual void run() = 0;
    virtual void exit() = 0;
    virtual FSMStateName checkChange() { return FSMStateName::PASSIVE; }

    FSMStateName state_name;
    std::string state_name_string;

protected:
    std::shared_ptr<ControlContext> ctx_;
    FSMStateName next_state_name_;
};