#pragma once

#include <memory>

#include "drake/systems/framework/diagram.h"

void start_simulation(drake::systems::Diagram<double>& diagram, std::unique_ptr<drake::systems::Context<double>> diagram_context, double target_realtime_rate);
