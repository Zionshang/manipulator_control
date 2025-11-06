#include "inverse_kinematics.hpp"
#include <cmath>
#include <iostream>

InverseKinematics::InverseKinematics(pinocchio::Model &model)
    : model_(model), data_(model)
{
    gripper_id_ = model.getFrameId("gripper");
    nq_ = model.nq;
    nv_ = model.nv;

    std::cout << "nq: " << nq_ << std::endl;
    std::cout << "nv: " << nv_ << std::endl;
}

VectorXd InverseKinematics::update(const pinocchio::SE3 &target_pose,
                                   const VectorXd &q_init)
{
    Eigen::VectorXd q = q_init;

    pinocchio::Data::Matrix6x J(6, nv_);
    J.setZero();

    bool success = false;
    Vector6d ee_err;
    Eigen::VectorXd v(nv_);
    for (int i = 0; i < IT_MAX; i++)
    {
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacement(model_, data_, gripper_id_);
        const pinocchio::SE3 fMd = data_.oMf[gripper_id_].actInv(target_pose); // err in link local frame
        ee_err = pinocchio::log6(fMd).toVector();                              // vector space
        if (ee_err.norm() < EPS)
        {
            success = true;
            break;
        }
        pinocchio::computeFrameJacobian(model_, data_, q, gripper_id_, J); // J in link local frame
        pinocchio::Data::Matrix6 Jlog;
        pinocchio::Jlog6(fMd.inverse(), Jlog);
        J = -Jlog * J;
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += DAMP;
        v.noalias() = -J.transpose() * JJt.ldlt().solve(ee_err);
        q = pinocchio::integrate(model_, q, v * DT);
    }

    if (!success)
    {
        std::cout << "Warning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
    }

    wrapToLimits(q, model_.lowerPositionLimit, model_.upperPositionLimit);
    return q;
}

void InverseKinematics::wrapToLimits(VectorXd &q, const VectorXd &q_lower, const VectorXd &q_upper)
{
    // 行为约定：
    // 1) 尝试将每个关节值以 2π 为周期折返到区间 [lower, upper]；
    // 2) 若区间宽度 < 2π 导致无法折返进入，则对“原值”执行上下界裁剪。

    const double two_pi = 2.0 * M_PI;

    // 返回 x ∈ [0, m) 的正模（m>0）
    auto positive_mod = [](double x, double m) -> double
    {
        double r = std::fmod(x, m);
        return (r < 0) ? (r + m) : r;
    };

    // 以 2π 周期围绕下界 lower 折返，返回值位于 [lower, lower+2π)
    auto fold_by_2pi = [&](double lower, double x) -> double
    {
        double offset = positive_mod(x - lower, two_pi); // [0, 2π)
        return lower + offset;
    };

    auto clamp = [](double x, double lo, double hi) -> double
    {
        return x < lo ? lo : (x > hi ? hi : x);
    };

    for (int j = 0; j < nq_; ++j)
    {
        const double lower = q_lower[j];
        const double upper = q_upper[j];
        const double x = q[j];

        // 步骤1：按 2π 折返
        const double folded = fold_by_2pi(lower, x); // ∈ [lower, lower+2π)

        if (folded <= upper)
        {
            // 成功落入 [lower, upper]
            q[j] = folded;
        }
        else
        {
            // 步骤2：无法折返进入（区间宽度 < 2π）→ 对原值裁剪
            q[j] = clamp(x, lower, upper);
        }
    }
}
