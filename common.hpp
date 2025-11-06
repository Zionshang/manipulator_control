#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

enum class FSMStateName
{
    CARTESIAN_CONTROL,
    RETURN_HOME,
    PASSIVE,
};

using Pose6d = Eigen::Vector<double, 6>;
using VectorDoF = Eigen::VectorXd;

struct JointState
{
    double timestamp = 0.0f;
    VectorDoF pos;
    VectorDoF vel;
    VectorDoF torque;
    double gripper_pos = 0.0f;    // m; 0 for close, GRIPPER_WIDTH for fully open
    double gripper_vel = 0.0f;    // s^{-1}; currently not supported
    double gripper_torque = 0.0f; // Nm; currently not supported
    JointState(int dof) : pos(VectorDoF::Zero(dof)), vel(VectorDoF::Zero(dof)), torque(VectorDoF::Zero(dof)) {}
    JointState(VectorDoF pos, VectorDoF vel, VectorDoF torque, double gripper_pos)
        : pos(pos), vel(vel), torque(torque), gripper_pos(gripper_pos) {}
    JointState(VectorDoF pos, VectorDoF vel, VectorDoF torque, double gripper_pos, double gripper_vel, double gripper_torque)
        : pos(pos), vel(vel), torque(torque), gripper_pos(gripper_pos), gripper_vel(gripper_vel), gripper_torque(gripper_torque) {}

    JointState operator+(const JointState &other) const
    {
        return JointState(pos + other.pos, vel + other.vel, torque + other.torque, gripper_pos + other.gripper_pos,
                          gripper_vel + other.gripper_vel, gripper_torque + other.gripper_torque);
    }
    JointState operator-(const JointState &other) const
    {
        return JointState(pos - other.pos, vel - other.vel, torque - other.torque, gripper_pos - other.gripper_pos,
                          gripper_vel - other.gripper_vel, gripper_torque - other.gripper_torque);
    }
    JointState operator*(const double &scalar) const
    {
        return JointState(pos * scalar, vel * scalar, torque * scalar, gripper_pos * scalar, gripper_vel * scalar,
                          gripper_torque * scalar);
    }
    JointState operator/(const double &scalar) const
    {
        return JointState(pos / scalar, vel / scalar, torque / scalar, gripper_pos / scalar, gripper_vel / scalar,
                          gripper_torque / scalar);
    }
};

struct Gain
{
    VectorDoF kp;
    VectorDoF kd;
    float gripper_kp = 0.0f;
    float gripper_kd = 0.0f;
    Gain(int dof) : kp(VectorDoF::Zero(dof)), kd(VectorDoF::Zero(dof)) {}
    Gain(VectorDoF kp, VectorDoF kd, float gripper_kp, float gripper_kd)
        : kp(kp), kd(kd), gripper_kp(gripper_kp), gripper_kd(gripper_kd)
    {
        if (kp.size() != kd.size())
            throw std::invalid_argument("Length of kp is not equal to kd.");
    }

    Gain operator+(const Gain &other) const
    {
        return Gain(kp + other.kp, kd + other.kd, gripper_kp + other.gripper_kp, gripper_kd + other.gripper_kd);
    }
    Gain operator*(const double &scalar) const
    {
        return Gain(kp * scalar, kd * scalar, gripper_kp * scalar, gripper_kd * scalar);
    }
};

struct EEFState
{
    double timestamp = 0.0f;
    Pose6d pose_6d;               // x, y, z, roll, pitch, yaw
    double gripper_pos = 0.0f;    // m; 0 for close, GRIPPER_WIDTH for fully open
    double gripper_vel = 0.0f;    // s^{-1}
    double gripper_torque = 0.0f; // Nm
    EEFState() : pose_6d(Pose6d::Zero()) {}
    EEFState(Pose6d pose_6d, double gripper_pos) : pose_6d(pose_6d), gripper_pos(gripper_pos) {}
    EEFState(Pose6d pose_6d, double gripper_pos, double gripper_vel, double gripper_torque)
        : pose_6d(pose_6d), gripper_pos(gripper_pos), gripper_vel(gripper_vel), gripper_torque(gripper_torque) {}

    EEFState operator+(const EEFState &other) const
    {
        return EEFState(pose_6d + other.pose_6d, gripper_pos + other.gripper_pos, gripper_vel + other.gripper_vel,
                        gripper_torque + other.gripper_torque);
    }
    EEFState operator*(const double &scalar) const
    {
        return EEFState(pose_6d * scalar, gripper_pos * scalar, gripper_vel * scalar, gripper_torque * scalar);
    }
};
