#include "fsm_home.hpp"
#include <algorithm>

StateHome::StateHome(std::shared_ptr<ControlContext> ctrl_context)
    : interpolator_(6, "cubic"),
      start_gain_(6),
      target_gain_(6),
      FSMState(ctrl_context, FSMStateName::RETURN_HOME, "return home")
{
    target_pos.setZero(6);
    middle_pos = target_pos;
    middle_pos(2) += 0.03;

    // Set desired gains for homing
    target_gain_.kp << 80.0, 70.0, 70.0, 30.0, 30.0, 20.0;
    target_gain_.kd << 2.0, 2.0, 2.0, 1.0, 1.0, 0.7;
    target_gain_.gripper_kp = 5.0;
    target_gain_.gripper_kd = 0.2;
}

void StateHome::enter()
{
    // Compute duration from max joint error
    const VectorDoF &q_now = ctx_->crt_joint_state.pos;
    VectorDoF err = (target_pos - q_now).cwiseAbs();
    duration_ = std::max(min_duration_, err.maxCoeff());

    // Create waypoints for interpolation
    t_start_ = ctx_->get_timestamp();
    JointState mid_state(6);
    mid_state.pos = middle_pos;
    mid_state.timestamp = t_start_ + duration_ * 0.8;
    JointState target_state(6);
    target_state.pos = target_pos;
    target_state.timestamp = t_start_ + duration_;

    std::vector<JointState> waypoints;
    waypoints.push_back(ctx_->crt_joint_state);
    waypoints.push_back(mid_state);
    waypoints.push_back(target_state);
    interpolator_.init(waypoints);

    // Capture start gains for interpolation
    start_gain_ = ctx_->cmd_gain;
}

void StateHome::run()
{
    double t_now = ctx_->get_timestamp();
    double s = std::clamp((t_now - t_start_) / duration_, 0.0, 1.0);

    // Interpolate command gains linearly
    ctx_->cmd_gain = start_gain_ * (1.0 - s) + target_gain_ * s;

    // Interpolate joint state
    ctx_->cmd_joint_state = interpolator_.interpolate(t_now);
}

void StateHome::exit() {}

FSMStateName StateHome::checkChange()
{
    if (ctx_->ref_fsm_state == FSMStateName::PASSIVE)
        return FSMStateName::PASSIVE;
    else
        return FSMStateName::RETURN_HOME;
}