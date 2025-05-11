#include <iostream>
#include <pinocchio/parsers/urdf.hpp>

#include "inverse_kinematics.hpp"
#include "webots_interface.hpp"

using Eigen::Quaterniond;
using Eigen::Vector3d;

int main(int, char **)
{
    std::string urdf_path = "/home/zishang/cpp_workspace/manipulator_control/robot/x5_reduced.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);

    Vector3d target_position(0.5, 0, 0.5);
    Quaterniond target_orientation(1, 0, 0, 0);
    pinocchio::SE3 target_pose(target_orientation, target_position);
    InverseKinematics ik(model);
    Eigen::VectorXd q_current = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd q_target = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd q_cmd = Eigen::VectorXd::Zero(model.nq + 2); // 包含了夹爪的关节

    //---------------------webots仿真 ---------------------
    WebotsInterface wb_interface;
    Eigen::VectorXd state_vector = Eigen::VectorXd::Zero(8 + 8); // 所有关节的位置和速度，包括夹爪关节
    while (wb_interface.isRunning())
    {
        wb_interface.recvState(state_vector);
        q_current = state_vector.head(model.nq);
        
        q_target = ik.update(target_pose, q_current);
        ik.wrapToLimits(q_target);

        q_cmd.head(model.nq) = q_target;
        wb_interface.sendCmd(q_cmd);
    }
}
