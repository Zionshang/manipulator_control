#include "fsm.hpp"
#include <iostream>

FSM::FSM(std::shared_ptr<ControlContext> control_context)
    : cxt_(control_context)
{

    _stateList.state_passive = new StatePassive(control_context);
    _stateList.state_home = new StateHome(control_context);

    initialize();
}

FSM::~FSM()
{
    _stateList.deletePtr();
}

void FSM::initialize()
{
    _currentState = _stateList.state_passive;
    _currentState->enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

void FSM::run()
{
    if (_mode == FSMMode::NORMAL)
    {
        _currentState->run();
        _nextStateName = _currentState->checkChange();
        if (_nextStateName != _currentState->state_name)
        {
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
            std::cout << "Switched from " << _currentState->state_name_string
                      << " to " << _nextState->state_name_string << std::endl;
        }
    }
    else if (_mode == FSMMode::CHANGE)
    {
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();
    }
}

FSMState *FSM::getNextState(FSMStateName stateName)
{
    switch (stateName)
    {
    case FSMStateName::PASSIVE:
        return _stateList.state_passive;
        break;
    case FSMStateName::RETURN_HOME:
        return _stateList.state_home;
        break;
    }
    // Fallback
    return _stateList.state_passive;
}