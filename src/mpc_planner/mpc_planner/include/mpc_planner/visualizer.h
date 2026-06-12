#ifndef MPC_PLANNER_H
#define MPC_PLANNER_H

#include <mpc_planner_types/data_types.h>
#include <mpc_planner_types/module_data.h>
#include <mpc_planner/state.h>

#include <memory>
#include <vector>

namespace RosTools
{
    class DataSaver;
    class Timer;
}

namespace MPCPlanner
{
    class RealTimeData;
    class State;
    class ExperimentUtil;

    struct PlannerOutput
    {
        Trajectory trajectory;
        bool success{false};

        PlannerOutput(double dt, int N) : trajectory(dt, N) {}

        PlannerOutput() = default;
    };

    class Visualizer
    {
    public:
        Visualizer();

    public:
        
        void visualize(const State &state, const RealTimeData &data, std::vector<Eigen::Vector2d> &mppi_plan);
        void reset(State &state, RealTimeData &data, bool success = true);

        RosTools::DataSaver &getDataSaver() const;

    private:
        bool _is_data_ready{false}, _was_reset{true};

        std::shared_ptr<ExperimentUtil> _experiment_util;
        PlannerOutput _output;
        PlannerOutput _mppi_output;


        Trajectory _warmstart;

        ModuleData _module_data;

        std::unique_ptr<RosTools::Timer> _startup_timer;

    };

}

#endif