#pragma once
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>

using Eigen::VectorXd;
using Vector6d = Eigen::Vector<double, 6>;

class InverseKinematics
{
public:
    InverseKinematics(pinocchio::Model &model);
    VectorXd update(const pinocchio::SE3 &target_pose, const VectorXd &q_init);
    void wrapToLimits(VectorXd &q, const VectorXd &q_lower, const VectorXd &q_upper);

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex gripper_id_;
    int nq_, nv_;

    const double EPS = 1e-4;
    const int IT_MAX = 50;
    const double DT = 5e-1;
    const double DAMP = 1e-6;
};
