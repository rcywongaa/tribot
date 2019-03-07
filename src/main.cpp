#include <memory>

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
#include "drake/systems/framework/diagram_builder.h"

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

DEFINE_double(target_realtime_rate, 1.0,
        "Desired rate relative to real time.  See documentation for "
        "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
        "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 0,
        "If greater than zero, the plant is modeled as a system with "
        "discrete updates and period equal to this time_step. "
        "If 0, the plant is modeled as a continuous system.");

int do_main() {
    systems::DiagramBuilder<double> builder;

    SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
    scene_graph.set_name("scene_graph");

    // Make and add the tribot model.
    const std::string full_name = getSrcDir() + "/../res/tribot.sdf";

    MultibodyPlant<double>& tribot =
        *builder.AddSystem<MultibodyPlant>(FLAGS_time_step);
    Parser(&tribot, &scene_graph).AddModelFromFile(full_name);

    // Add gravity to the model.
    tribot.AddForceElement<UniformGravityFieldElement>();

    // Now the model is complete.
    tribot.Finalize();

    // Sanity check on the availability of the optional source id before using it.
    DRAKE_DEMAND(tribot.geometry_source_is_registered());

    builder.Connect(
            tribot.get_geometry_poses_output_port(),
            scene_graph.get_source_pose_port(tribot.get_source_id().value()));

    geometry::ConnectDrakeVisualizer(&builder, scene_graph);
    auto diagram = builder.Build();

    // Create a context for this system:
    std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();
    diagram->SetDefaultContext(diagram_context.get());
    systems::Context<double>& tribot_context = diagram->GetMutableSubsystemContext(tribot, diagram_context.get());

    // There is no input actuation in this example for the passive dynamics.
    tribot_context.FixInputPort(tribot.get_actuation_input_port().get_index(), Vector1d(0));

    // Get joints so that we can set initial conditions.
    const PrismaticJoint<double>& cart_slider = tribot.GetJointByName<PrismaticJoint>("CartSlider");
    const RevoluteJoint<double>& mid_joint = tribot.GetJointByName<RevoluteJoint>("MidJoint");

    // Set initial state.
    cart_slider.set_translation(&tribot_context, 0.0);
    mid_joint.set_angle(&tribot_context, 2.0);

    systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
    simulator.Initialize();
    simulator.StepTo(FLAGS_simulation_time);

    return 0;
}

int main(int argc, char* argv[]) {
    gflags::SetUsageMessage(
            "A simple cart pole demo using Drake's MultibodyPlant,"
            "with SceneGraph visualization. "
            "Launch drake-visualizer before running this example.");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::logging::HandleSpdlogGflags();
    return do_main();
}
