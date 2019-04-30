#include "drake_util.hpp"

#include "drake/systems/analysis/simulator.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/parsing/parser.h"

using namespace drake;

drake::multibody::MultibodyPlant<double>& create_default_plant(std::string model_filename, drake::systems::DiagramBuilder<double>& builder)
{

    // If greater than zero, the plant is modeled as a system with
    // discrete updates and period equal to this time_step.
    // If 0, the plant is modeled as a continuous system.
    auto pair = drake::multibody::AddMultibodyPlantSceneGraph(&builder, std::make_unique<drake::multibody::MultibodyPlant<double>>(0));

    drake::multibody::MultibodyPlant<double>& plant = pair.plant;
    drake::geometry::SceneGraph<double>& scene_graph = pair.scene_graph;

    // Make and add the model.
    drake::multibody::Parser(&plant, &scene_graph).AddModelFromFile(model_filename);

    // Connect scene graph to visualizer
    drake::geometry::ConnectDrakeVisualizer(&builder, scene_graph);
    return plant;
}

void start_simulation(drake::systems::Diagram<double>& diagram, std::unique_ptr<drake::systems::Context<double>> diagram_context, double target_realtime_rate)
{
    systems::Simulator<double> simulator(diagram, std::move(diagram_context));
    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(target_realtime_rate);
    simulator.Initialize();
    simulator.StepTo(std::numeric_limits<double>::infinity());
}
