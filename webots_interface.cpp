#include "webots_interface.hpp"
#include <limits>

WebotsInterface::WebotsInterface(std::shared_ptr<ControlContext> ctx)
    : ctx_(ctx)
{
    supervisor_ = new webots::Supervisor();
    timestep_ = (int)supervisor_->getBasicTimeStep();
    std::cout << "timeStep in simulation is :" << timestep_ << std::endl;

    num_joints_ = joint_motor_name_.size();
    joint_motor_.assign(num_joints_, nullptr);
    joint_sensor_.assign(num_joints_, nullptr);

    last_q_.setZero(num_joints_);

    initRecv();
    initSend();

    last_key_ = -1;
}

WebotsInterface::~WebotsInterface()
{
    delete supervisor_;
}

// legacy raw IO removed; use readContext()/writeCommand() instead

void WebotsInterface::readContext()
{
    // Read joint sensors (first 6 as arm), update ctx_
    int dof = ctx_ ? ctx_->crt_joint_state.pos.size() : 6;
    Eigen::VectorXd q(num_joints_), v(num_joints_);
    for (int i = 0; i < num_joints_; ++i)
    {
        q(i) = joint_sensor_[i]->getValue();
        v(i) = (q(i) - last_q_(i)) / double(timestep_) * 1000.0; // rad/s
        last_q_(i) = q(i);
    }
    if (ctx_)
    {
        for (int i = 0; i < std::min(dof, num_joints_); ++i)
        {
            ctx_->crt_joint_state.pos(i) = q(i);
            ctx_->crt_joint_state.vel(i) = v(i);
        }
        ctx_->crt_joint_state.timestamp = ctx_->get_timestamp();
    }

    // Keyboard to switch FSM state
    key_ = keyboard_->getKey();
    if (key_ != last_key_)
    {
        if (key_ == 'H' || key_ == 'h')
        {
            if (ctx_)
                ctx_->ref_fsm_state = FSMStateName::RETURN_HOME;
            std::cout << "Keyboard: switch to RETURN_HOME" << std::endl;
        }
        else if (key_ == 'P' || key_ == 'p')
        {
            if (ctx_)
                ctx_->ref_fsm_state = FSMStateName::PASSIVE;
            std::cout << "Keyboard: switch to PASSIVE" << std::endl;
        }
        last_key_ = key_;
    }
}

void WebotsInterface::writeCommand()
{
    // MIT mode torque control: tau = kp*(q_des - q) + kd*(dq_des - dq) + tau_ff
    const auto &q = ctx_->crt_joint_state.pos;
    const auto &dq = ctx_->crt_joint_state.vel;
    const auto &q_des = ctx_->cmd_joint_state.pos;
    const auto &dq_des = ctx_->cmd_joint_state.vel;
    const auto &tau_ff = ctx_->cmd_joint_state.torque;
    const auto &kp = ctx_->cmd_gain.kp;
    const auto &kd = ctx_->cmd_gain.kd;

    for (int i = 0; i < joint_motor_.size(); ++i)
    {
        double tau = kp(i) * (q_des(i) - q(i)) + kd(i) * (dq_des(i) - dq(i)) + tau_ff(i);
        joint_motor_[i]->setTorque(tau);
    }
}

bool WebotsInterface::isRunning()
{
    if (supervisor_->step(timestep_) != -1)
        return true;
    else
        return false;
}

void WebotsInterface::initRecv()
{
    // supervisor init
    robot_node_ = supervisor_->getFromDef(robot_name_);
    if (robot_node_ == NULL)
    {
        printf("error supervisor\n");
        exit(1);
    }

    for (int i = 0; i < num_joints_; i++)
    {
        joint_sensor_[i] = supervisor_->getPositionSensor(joint_sensor_name_[i]);
        joint_sensor_[i]->enable(timestep_);
    }
    keyboard_ = supervisor_->getKeyboard();
    keyboard_->enable(timestep_);
}

void WebotsInterface::initSend()
{
    for (int i = 0; i < num_joints_; i++)
    {
        joint_motor_[i] = supervisor_->getMotor(joint_motor_name_[i]);
        // Disable position control to allow torque (MIT) control
        joint_motor_[i]->setPosition(std::numeric_limits<double>::infinity());
        // Remove velocity limit to avoid clamping torque-driven motion
        joint_motor_[i]->setVelocity(std::numeric_limits<double>::infinity());
    }
}