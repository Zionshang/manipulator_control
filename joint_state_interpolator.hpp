#pragma once
#include "common.hpp"
#include <vector>

class JointStateInterpolator
{
public:
    /**
     * @param dof 自由度（关节数量）
     * @param method 插值方法标识（例如 "linear", "cubic" 等）
     */
    JointStateInterpolator(int dof, std::string method);

    /**
     * 初始化轨迹（从 start_state 到end_state）。 
     * @param start_state 起始关节状态（包含时间和关节位置）
     * @param end_state 目标关节状态（包含时间和关节位置）
     */
    void init(JointState start_state, JointState end_state);

    /**
     * 初始化一个固定（静止）轨迹，只有起始状态（目标与起始相同）。常用于保持当前关节位置不变的情形。
     * @param start_state 起始/固定关节状态
     */
    void init(JointState start_state);

    /**
     * 使用一组路点初始化轨迹（多段轨迹）。
     * 要求 waypoints.size() >= 1；
     * - 若 size()==1，则等价于固定轨迹（保持该状态不变）。
     * - 若 size()>=2，则按时间顺序连接各路点。
     * @param waypoints 按时间排序的一系列 JointState 路点(包含当前状态)
     */
    void init(const std::vector<JointState> &waypoints);

    /**
     * 在当前轨迹末尾追加一个新的路点（waypoint）。
     * 路点的时间应大于当前轨迹的最后时间，函数负责将新段拼接进内部轨迹描述中。
     * @param current_time 当前时间（用于与已有轨迹时间比较或计算偏移）
     * @param end_state 要追加的目标关节状态（包含时间和位置）
     */
    void append_waypoint(double current_time, JointState end_state);

    /**
     * 替换在 current_time 之后的轨迹为新的 end_state 路段。
     * 与 append_waypoint 不同，override 会替换后续已有的路点，常用于在线调整轨迹。
     * @param current_time 当前时间（覆盖发生的时间点）
     * @param end_state 覆盖后的目标关节状态
     */
    void override_waypoint(double current_time, JointState end_state);

    /**
     * 返回给定时间点的插值关节状态。 如果 time 在轨迹范围之外，则返回边界值。
     * @param time 查询时间
     * @return 对应时间的 JointState
     */
    JointState interpolate(double time);

private:
    int dof_;
    bool initialized_ = false;
    std::string method_;
    std::vector<JointState> traj_;
};
