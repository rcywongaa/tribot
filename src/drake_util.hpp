#pragma once

#include <memory>

#include "drake/systems/framework/diagram.h"
#include "drake/multibody/plant/multibody_plant.h"


/**
 * @brief Uses the pose_output_port and model_file to generate visuals for the plant
 *
 * Creates a MultibodyPlant from the model_file
 * Connects pose_output_port MultibodyPositionToGeometryPose and then to SceneGraph
 */
void add_plant_visuals(
        drake::systems::DiagramBuilder<double>* builder,
        drake::geometry::SceneGraph<double>* scene_graph,
        drake::multibody::MultibodyPlant<double>& mbp,
        const std::string model_file,
        const drake::systems::OutputPort<double>& pose_output_port);
/**
 * @brief Creates a plant and scene graph and connects scene graph to visualizer
 * @return The plant created
 */
drake::multibody::MultibodyPlant<double>& create_default_plant(std::string model_filename, drake::systems::DiagramBuilder<double>& builder, double ground_offset = 0.0);

void start_simulation(drake::systems::Diagram<double>& diagram, std::unique_ptr<drake::systems::Context<double>> diagram_context, double target_realtime_rate);

/**
 * @brief Prints the mass matrix of the given plant in the given context's configuration
 *
 * Example:
 *     std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();
 *     Context<double>& context = diagram->GetMutableSubsystemContext(plant, diagram_context.get());
 *
 *     print_mass_matrix(plant, context);
 */
void print_mass_matrix(drake::multibody::MultibodyPlant<double>& plant, drake::systems::Context<double>& context);
