#include <memory>
#include <math.h>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/common/eigen_types.h"

inline const std::string getSrcDir()
{
    return std::string(__FILE__).erase(std::string(__FILE__).rfind('/')) + "/";
}

using namespace drake;

using geometry::SceneGraph;
using drake::lcm::DrakeLcm;

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::multibody::UniformGravityFieldElement;
using drake::geometry::HalfSpace;
using drake::multibody::CoulombFriction;
using Eigen::Vector3d;
using Eigen::Vector2d;
using systems::Context;

DEFINE_double(target_realtime_rate, 1.0,
        "Desired rate relative to real time.  See documentation for "
        "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
        "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 0,
        "If greater than zero, the plant is modeled as a system with "
        "discrete updates and period equal to this time_step. "
        "If 0, the plant is modeled as a continuous system.");

int do_main() {
    systems::DiagramBuilder<double> builder;

    auto pair = AddMultibodyPlantSceneGraph(&builder, std::make_unique<MultibodyPlant<double>>(FLAGS_time_step));

    MultibodyPlant<double>& plant = pair.plant;

    SceneGraph<double>& scene_graph = pair.scene_graph;
    scene_graph.set_name("scene_graph");

    // Make and add the tribot model.
    const std::string tribot_model_filename = getSrcDir() + "/../res/wheel.sdf";
    Parser(&plant, &scene_graph).AddModelFromFile(tribot_model_filename);

    //const std::string ground_model_filename = getSrcDir() + "/../res/ground.sdf";
    //Parser(&plant, &scene_graph).AddModelFromFile(ground_model_filename);

    Vector3<double> normal_W(0, 0, 1);
    Vector3<double> point_W(0, 0, -0.2);

    const CoulombFriction<double> surface_friction(
            0.9 /* static friction */, 0.9 /* dynamic friction */);

    // A half-space for the ground geometry.
    plant.RegisterCollisionGeometry(
            plant.world_body(), HalfSpace::MakePose(normal_W, point_W),
            HalfSpace(), "collision", surface_friction);

    // Add visual for the ground.
    plant.RegisterVisualGeometry(
            plant.world_body(), HalfSpace::MakePose(normal_W, point_W),
            HalfSpace(), "visual");

    // Add gravity to the model.
    plant.AddForceElement<UniformGravityFieldElement>();

    // Now the model is complete.
    plant.Finalize();

    plant.set_penetration_allowance(0.0001);

    // Sanity check on the availability of the optional source id before using it.
    DRAKE_DEMAND(plant.geometry_source_is_registered());

    geometry::ConnectDrakeVisualizer(&builder, scene_graph);
    auto diagram = builder.Build();

    std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();
    diagram->SetDefaultContext(diagram_context.get());
    systems::Context<double>& plant_context = diagram->GetMutableSubsystemContext(plant, diagram_context.get());

    plant_context.FixInputPort(plant.get_actuation_input_port().get_index(), Vector1d(10.0));

    // Get joints so that we can set initial conditions.
    const RevoluteJoint<double>& roll = plant.GetJointByName<RevoluteJoint>("roll");
    const RevoluteJoint<double>& tilt = plant.GetJointByName<RevoluteJoint>("tilt");
    const RevoluteJoint<double>& turn = plant.GetJointByName<RevoluteJoint>("turn");

    // Set initial state.
    tilt.set_angle(&plant_context, 0.2);
    turn.set_angle(&plant_context, 0.5);
    roll.set_angular_rate(&plant_context, 10);

    systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
    simulator.Initialize();
    simulator.StepTo(FLAGS_simulation_time);

    return 0;
}

int main(int argc, char* argv[]) {
    gflags::SetUsageMessage(
            "A tribot demo using Drake's MultibodyPlant,"
            "with SceneGraph visualization. "
            "Launch drake-visualizer before running this example.");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::logging::HandleSpdlogGflags();
    return do_main();
}

