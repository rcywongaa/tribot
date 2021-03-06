#pragma once

#include <memory>

#include "drake/systems/framework/diagram.h"
#include "drake/multibody/plant/multibody_plant.h"

/**
 * @brief Creates a plant and scene graph and connects scene graph to visualizer
 * @return The plant created
 */
drake::multibody::MultibodyPlant<double>& create_default_plant(std::string model_filename, drake::systems::DiagramBuilder<double>& builder, double ground_offset = 0.0);

void start_simulation(drake::systems::Diagram<double>& diagram, std::unique_ptr<drake::systems::Context<double>> diagram_context, double target_realtime_rate);
