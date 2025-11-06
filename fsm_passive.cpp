#include "fsm_passive.hpp"

StatePassive::StatePassive(std::shared_ptr<ControlContext> ctrl_context)
    : FSMState(ctrl_context, FSMStateName::PASSIVE, "passive")
{
    kd_.resize(ctx_->dof);
    kd_ << 2.0, 2.0, 2.0, 1.0, 1.0, 0.7;
}

void StatePassive::enter()
{
    Gain passive_gain(ctx_->dof);
    passive_gain.kp.setZero();
    passive_gain.kd = kd_;
    passive_gain.gripper_kd = 0.2;
    ctx_->cmd_gain = passive_gain;
}

void StatePassive::run()
{
    // Keep commanding current position to avoid motion
    ctx_->cmd_joint_state.pos = ctx_->crt_joint_state.pos;
    ctx_->cmd_joint_state.vel = VectorDoF::Zero(ctx_->dof);
}

void StatePassive::exit() {}

FSMStateName StatePassive::checkChange()
{
    if (ctx_->ref_fsm_state == FSMStateName::RETURN_HOME)
        return FSMStateName::RETURN_HOME;
    return FSMStateName::PASSIVE;
}