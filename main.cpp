#include <iostream>
#include "controller.hpp"
#include "webots_interface.hpp"
#include <pinocchio/parsers/urdf.hpp>

int main(int, char **)
{
    constexpr int dof = 6; // arm DOF
    auto ctx = std::make_shared<ControlContext>(dof);

    std::string urdf_path = "/home/zishang/cpp_workspace/manipulator_control/robot/x5_reduced.urdf";
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    Controller controller(ctx, model);
    WebotsInterface webots(ctx);

    // Main control loop: IO -> FSM -> IO
    while (webots.isRunning())
    {
        webots.readContext();
        controller.run();
        webots.writeCommand();
    }

    return 0;
}
