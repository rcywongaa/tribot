#include "drake_util.hpp"

#include "drake/systems/analysis/simulator.h"

using namespace drake;

void start_simulation(drake::systems::Diagram<double>& diagram, std::unique_ptr<drake::systems::Context<double>> diagram_context, double target_realtime_rate)
{
    systems::Simulator<double> simulator(diagram, std::move(diagram_context));
    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(target_realtime_rate);
    simulator.Initialize();
    simulator.StepTo(std::numeric_limits<double>::infinity());
}
