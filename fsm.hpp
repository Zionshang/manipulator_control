#pragma once

#include "fsm_state.hpp"
#include "fsm_home.hpp"
#include "fsm_passive.hpp"

enum class FSMMode
{
    NORMAL,
    CHANGE
};

struct FSMStateList
{
    StatePassive *state_passive;
    StateHome *state_home;

    void deletePtr()
    {
        delete state_passive;
        delete state_home;
    }
};

class FSM
{
public:
    FSM(std::shared_ptr<ControlContext> control_context);
    ~FSM();
    void initialize();
    void run();

private:
    FSMState *getNextState(FSMStateName stateName);
    std::shared_ptr<ControlContext> cxt_;
    FSMState *_currentState;
    FSMState *_nextState;
    FSMStateName _nextStateName;
    FSMStateList _stateList;
    FSMMode _mode;
    long long _startTime;
    int count;
};
