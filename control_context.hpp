#pragma once
#include <chrono>
#include "common.hpp"

#define sleep_ms(x) std::this_thread::sleep_for(std::chrono::milliseconds(x))
#define sleep_us(x) std::this_thread::sleep_for(std::chrono::microseconds(x))
#define get_time_us() std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()

class ControlContext
{
public:
    inline ControlContext(int dof);
    inline double get_timestamp();

    // sent to hardware
    JointState cmd_joint_state;
    Gain cmd_gain;

    // read from hardware
    JointState crt_joint_state;

    // from user
    JointState ref_joint_state;
    EEFState ref_eef_state;
    FSMStateName ref_fsm_state;

    int dof;

private:
    long int start_time_us_;
};

inline ControlContext::ControlContext(int dof)
    : cmd_joint_state(dof), cmd_gain(dof),
      crt_joint_state(dof), ref_joint_state(dof),
      ref_fsm_state(FSMStateName::PASSIVE),
      dof(dof)
{
    start_time_us_ = get_time_us();
}

inline double ControlContext::get_timestamp()
{
    return double(get_time_us() - start_time_us_) / 1e6;
}