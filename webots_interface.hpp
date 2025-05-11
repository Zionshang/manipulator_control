#pragma once
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Gyro.hpp>
#include <webots/Supervisor.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <webots/Keyboard.hpp>
#include <webots/Joystick.hpp>

class WebotsInterface
{
public:
    WebotsInterface();
    ~WebotsInterface();
    void recvState(Eigen::VectorXd &state_vector); // second
    void sendCmd(const Eigen::VectorXd &tau);
    bool isRunning();
    double current_time() { return current_time_; }
    void resetSim() { supervisor_->simulationReset(); }
    double timestep() { return double(timestep_) / 1000; }

private:
    void initRecv();
    void initSend();

    int timestep_;
    double current_time_;

    Eigen::VectorXd last_q_;

    // webots interface
    webots::Supervisor *supervisor_;
    webots::Node *robot_node_;
    std::vector<webots::Motor *> joint_motor_;
    std::vector<webots::PositionSensor *> joint_sensor_;
    webots::Keyboard *keyboard_;

    std::string robot_name_ = "X5";
    std::vector<std::string> joint_motor_name_ = {"joint1", "joint2", "joint3", "joint4",
                                                  "joint5", "joint6", "joint7", "joint8"};
    std::vector<std::string> joint_sensor_name_ = {"joint1_sensor", "joint2_sensor", "joint3_sensor", "joint4_sensor",
                                                   "joint5_sensor", "joint6_sensor", "joint7_sensor", "joint8_sensor"};

    int num_joints_;
    int key_, last_key_;
};