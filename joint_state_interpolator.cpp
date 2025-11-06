
#include "joint_state_interpolator.hpp"
#include <cstdarg>
#include <cstdio>

JointStateInterpolator::JointStateInterpolator(int dof, std::string method)
{
    if (method != "linear" && method != "cubic")
    {
        throw std::invalid_argument("Invalid interpolation method: " + method +
                                    ". Currently available: 'linear' or 'cubic'");
    }
    dof_ = dof;
    method_ = method;
    initialized_ = false;
    traj_ = std::vector<JointState>();
}

void JointStateInterpolator::init(JointState start_state, JointState end_state)
{
    if (end_state.timestamp < start_state.timestamp)
    {
        throw std::invalid_argument("End time must be no less than start time");
    }
    else if (end_state.timestamp == start_state.timestamp)
    {
        throw std::invalid_argument("Start and end time are the same, plsease use init_fixed() instead");
    }
    if (start_state.pos.size() != dof_ || end_state.pos.size() != dof_)
    {
        throw std::invalid_argument("Joint state dimension mismatch");
    }
    traj_.clear();
    traj_.push_back(start_state);
    traj_.push_back(end_state);
    initialized_ = true;
}

void JointStateInterpolator::init(JointState start_state)
{
    if (start_state.pos.size() != dof_)
    {
        throw std::invalid_argument("Joint state dimension mismatch");
    }
    traj_.clear();
    traj_.push_back(start_state);
    initialized_ = true;
}

void JointStateInterpolator::init(const std::vector<JointState> &waypoints)
{
    if (waypoints.empty())
    {
        throw std::invalid_argument("Waypoints should not be empty");
    }

    for (const auto &wp : waypoints)
    {
        if (wp.pos.size() != dof_)
        {
            throw std::invalid_argument("Joint state dimension mismatch");
        }
    }

    if (waypoints.size() == 1)
    {
        traj_.clear();
        traj_.push_back(waypoints[0]);
        initialized_ = true;
        return;
    }

    for (size_t i = 1; i < waypoints.size(); ++i)
    {
        if (waypoints[i].timestamp <= waypoints[i - 1].timestamp)
        {
            throw std::invalid_argument("Waypoints timestamps must be strictly increasing");
        }
    }

    traj_ = waypoints;
    initialized_ = true;
}

void JointStateInterpolator::append_waypoint(double current_time, JointState end_state)
{
    if (!initialized_)
    {
        throw std::runtime_error("Interpolator not initialized");
    }

    if (end_state.pos.size() != dof_)
    {
        throw std::invalid_argument("Joint state dimension mismatch");
    }

    if (end_state.timestamp <= current_time)
    {
        throw std::invalid_argument("End time must be no less than current time");
    }

    JointState current_state{dof_};

    if (current_time < traj_[0].timestamp)
    {
        throw std::runtime_error("Current time must be no less than start time");
    }
    else
    {
        current_state = interpolate(current_time);
    }

    std::vector<JointState> prev_traj = traj_;
    traj_.clear();
    traj_.push_back(current_state);
    for (int i = 0; i < prev_traj.size(); i++)
    {
        if (prev_traj[i].timestamp > current_time)
        {
            if (prev_traj[i].timestamp > end_state.timestamp)
            {
                traj_.push_back(end_state);
                break;
            }
            else
            {
                traj_.push_back(prev_traj[i]);
            }
        }
        if (i == prev_traj.size() - 1)
        {
            traj_.push_back(end_state);
        }
    }
}

void JointStateInterpolator::override_waypoint(double current_time, JointState end_state)
{
    if (!initialized_)
    {
        throw std::runtime_error("Interpolator not initialized");
    }

    if (end_state.pos.size() != dof_)
    {
        throw std::invalid_argument("Joint state dimension mismatch");
    }

    if (end_state.timestamp <= current_time)
    {
        throw std::invalid_argument("End time must be no less than current time");
    }

    JointState current_state{dof_};

    if (current_time < traj_[0].timestamp)
    {
        throw std::runtime_error("Current time must be no less than start time");
    }
    else
    {
        current_state = interpolate(current_time);
    }

    std::vector<JointState> prev_traj = traj_;
    traj_.clear();
    traj_.push_back(current_state);
    traj_.push_back(end_state);
}

JointState JointStateInterpolator::interpolate(double time)
{

    if (!initialized_)
    {
        throw std::runtime_error("Interpolator not initialized");
    }
    if (time <= 0)
    {
        throw std::invalid_argument("Interpolate time must be greater than 0");
    }

    if (traj_.size() == 0)
    {
        throw std::runtime_error("Empty trajectory");
    }
    if (traj_.size() == 1)
    {
        JointState interp_state = traj_[0];
        interp_state.timestamp = time;
        return interp_state;
    }

    if (time <= traj_[0].timestamp)
    {
        JointState interp_state = traj_[0];
        interp_state.timestamp = time;
        return interp_state;
    }
    else if (time >= traj_.back().timestamp)
    {
        JointState interp_state = traj_.back();
        interp_state.timestamp = time;
        return interp_state;
    }

    for (int i = 0; i <= traj_.size() - 2; i++)
    {
        JointState start_state = traj_[i];
        JointState end_state = traj_[i + 1];
        if (time >= start_state.timestamp && time <= end_state.timestamp)
        {
            if (method_ == "linear")
            {
                JointState interp_result = start_state + (end_state - start_state) * (time - start_state.timestamp) /
                                                             (end_state.timestamp - start_state.timestamp);
                interp_result.timestamp = time;
                return interp_result;
            }
            else if (method_ == "cubic")
            {
                // Torque and gripper pos will still be linearly interpolated
                JointState interp_result = start_state + (end_state - start_state) * (time - start_state.timestamp) /
                                                             (end_state.timestamp - start_state.timestamp);
                interp_result.timestamp = time;

                // Cubic interpolation for pos and vel
                double t = (time - start_state.timestamp) / (end_state.timestamp - start_state.timestamp);
                double t2 = t * t;
                double t3 = t2 * t;
                double pos_a = 2 * t3 - 3 * t2 + 1;
                double pos_b = t3 - 2 * t2 + t;
                double pos_c = -2 * t3 + 3 * t2;
                double pos_d = t3 - t2;
                interp_result.pos =
                    pos_a * start_state.pos + pos_b * start_state.vel + pos_c * end_state.pos + pos_d * end_state.vel;

                double vel_a = 6 * t2 - 6 * t;
                double vel_b = 3 * t2 - 4 * t + 1;
                double vel_c = -6 * t2 + 6 * t;
                double vel_d = 3 * t2 - 2 * t;
                interp_result.vel =
                    vel_a * start_state.pos + vel_b * start_state.vel + vel_c * end_state.pos + vel_d * end_state.vel;
                return interp_result;
            }
        }
        if (i == traj_.size() - 2)
        {
            throw std::runtime_error("Interpolation failed");
        }
    }
    throw std::runtime_error("Interpolation fell through without result");
}