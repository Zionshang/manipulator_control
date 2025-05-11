#include "inverse_kinematics.hpp"

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
    for (int i = 0;; i++)
    {
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacement(model_, data_, gripper_id_);
        const pinocchio::SE3 iMd = data_.oMf[gripper_id_].actInv(target_pose); // err in SE3 space
        ee_err = pinocchio::log6(iMd).toVector();                              // err in vector space
        if (ee_err.norm() < EPS)
        {
            success = true;
            break;
        }
        if (i >= IT_MAX)
        {
            success = false;
            break;
        }
        pinocchio::computeFrameJacobian(model_, data_, q, gripper_id_, J); // J in joint frame
        pinocchio::Data::Matrix6 Jlog;
        pinocchio::Jlog6(iMd.inverse(), Jlog);
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

    return q;
}

void InverseKinematics::wrapToLimits(VectorXd &q)
{
    for (int j = 0; j < nq_; ++j)
    {
        const double lower = model_.lowerPositionLimit[j];
        const double upper = model_.upperPositionLimit[j];
        const double range = upper - lower;

        if (range >= 2.0 * M_PI - 1e-6)
        {
            // 如果关节限位允许超过2pi，则只需 wrap 到 [-pi, pi]
            q[j] = std::atan2(std::sin(q[j]), std::cos(q[j]));
        }
        else
        {
            // 将 q[j] 归一化后补偿到 joint limit 范围内
            double two_pi = 2.0 * M_PI;
            double q_wrapped = std::atan2(std::sin(q[j]), std::cos(q[j]));

            // 找到最接近原始 q[j] 的在 [lower, upper] 范围内的周期补偿值
            double best_q = q_wrapped;
            double min_diff = std::abs(q[j] - best_q);

            for (int k = -10; k <= 10; ++k)
            {
                double candidate = q_wrapped + k * two_pi;
                if (candidate >= lower && candidate <= upper)
                {
                    double diff = std::abs(candidate - q[j]);
                    if (diff < min_diff)
                    {
                        best_q = candidate;
                        min_diff = diff;
                    }
                }
            }

            q[j] = best_q;
        }
    }
}
