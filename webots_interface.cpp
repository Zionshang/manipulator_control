#include "webots_interface.hpp"

WebotsInterface::WebotsInterface()
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

void WebotsInterface::recvState(Eigen::VectorXd &state_vector)
{
    current_time_ = supervisor_->getTime();
    Eigen::VectorXd q(num_joints_), v(num_joints_);

    // sensor data
    for (int i = 0; i < num_joints_; i++)
    {
        q(i) = joint_sensor_[i]->getValue();
        v(i) = (q(7 + i) - last_q_(i)) / double(timestep_) * 1000;
        last_q_(i) = q(i);
    }
    state_vector << q, v;
}

void WebotsInterface::sendCmd(const Eigen::VectorXd &q)
{
    for (int i = 0; i < num_joints_; i++)
    {
        joint_motor_[i]->setPosition(q(i));
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

    camera_ = supervisor_->getCamera("camera");
    camera_->enable(timestep_);
}

void WebotsInterface::initSend()
{
    for (int i = 0; i < num_joints_; i++)
        joint_motor_[i] = supervisor_->getMotor(joint_motor_name_[i]);
}