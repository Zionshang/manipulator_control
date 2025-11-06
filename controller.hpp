#pragma once
#include "control_context.hpp"
#include <memory>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

class Controller
{
public:
    explicit Controller(std::shared_ptr<ControlContext> ctrl_context,
                        pinocchio::Model &model);
    ~Controller();

    // Advance one control step
    void run();

private:
    void gravityCompensation();
    
    pinocchio::Model model_;
    pinocchio::Data data_;

    std::shared_ptr<ControlContext> ctx_;
    std::unique_ptr<class FSM> fsm_;
};
