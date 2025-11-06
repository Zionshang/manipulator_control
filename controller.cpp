#include "controller.hpp"
#include "fsm.hpp"
#include <pinocchio/algorithm/rnea.hpp>

Controller::Controller(std::shared_ptr<ControlContext> ctrl_context,
					   pinocchio::Model &model)
	: ctx_(std::move(ctrl_context)),
	  fsm_(std::make_unique<FSM>(ctx_)),
	  model_(model),
	  data_(model)
{
}

Controller::~Controller() = default;

void Controller::run()
{
	// Controller is now only responsible for advancing FSM
	fsm_->run();
	if (ctx_->ref_fsm_state != FSMStateName::PASSIVE)
	{
		gravityCompensation();
	}
}

void Controller::gravityCompensation()
{
	// Compute gravity compensation torques
	pinocchio::computeGeneralizedGravity(model_, data_, ctx_->crt_joint_state.pos);
	ctx_->cmd_joint_state.torque = data_.g;
}
