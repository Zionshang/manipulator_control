#pragma once

#include "fsm_state.hpp"
#include "joint_state_interpolator.hpp"

class StateHome : public FSMState
{
public:
    StateHome(std::shared_ptr<ControlContext> ctrl_context);
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:
    VectorDoF target_pos;
    VectorDoF middle_pos;
    JointStateInterpolator interpolator_;

    double duration_;
    double min_duration_ = 0.5; // s, lower bound of duration
    double t_start_ = 0.0;      // start timestamp of homing

    Gain start_gain_;
    Gain target_gain_;
};
