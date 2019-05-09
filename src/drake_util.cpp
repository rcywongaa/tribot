#include "drake_util.hpp"

#include "drake/systems/analysis/simulator.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/common/eigen_types.h"
#include <Eigen/Dense>

using namespace drake;

drake::multibody::MultibodyPlant<double>& create_default_plant(std::string model_filename, drake::systems::DiagramBuilder<double>& builder, double ground_offset)
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

    Eigen::Vector3d point_W(0, 0, ground_offset);

    const drake::multibody::CoulombFriction<double> surface_friction(
            0.99 /* static friction */, 0.99 /* dynamic friction */);

    // A half-space for the ground geometry.
    plant.RegisterCollisionGeometry(
            plant.world_body(),
            math::RigidTransformd(point_W),
            geometry::HalfSpace(),
            "GroundCollisionGeometry",
            surface_friction);
    // Add visual for the ground.
    plant.RegisterVisualGeometry(
            plant.world_body(), math::RigidTransformd(point_W),
            drake::geometry::HalfSpace(), "visual");

    // Add gravity to the model.
    plant.AddForceElement<drake::multibody::UniformGravityFieldElement>();

    // Now the model is complete.
    plant.Finalize();

    plant.set_penetration_allowance(0.001);

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
