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

#include "cart_pole_util.hpp"
#include "mip_util.hpp"

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

std::string getResDir()
{
    return getSrcDir() + "/../res/";
}

int do_main() {
    systems::DiagramBuilder<double> builder;

    auto pair = AddMultibodyPlantSceneGraph(&builder, std::make_unique<MultibodyPlant<double>>(FLAGS_time_step));

    MultibodyPlant<double>& plant = pair.plant;

    SceneGraph<double>& scene_graph = pair.scene_graph;
    scene_graph.set_name("scene_graph");

    // Make and add the segway model.
    const std::string segway_model_filename = getSrcDir() + "/../res/segway.sdf";
    Parser(&plant, &scene_graph).AddModelFromFile(segway_model_filename);

    Vector3<double> normal_W(0, 0, 1);
    Vector3<double> point_W(0, 0, -0.25);

    const CoulombFriction<double> surface_friction(
            0.8 /* static friction */, 0.3 /* dynamic friction */);

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

    plant.set_penetration_allowance(0.001);

    // Sanity check on the availability of the optional source id before using it.
    DRAKE_DEMAND(plant.geometry_source_is_registered());

    auto controller = builder.AddSystem(MakeCartPoleLQRController(getResDir() + "segway_cart_pole.sdf"));
    controller->set_name("controller");
    auto mip_to_cart_pole_state_converter = builder.AddSystem(std::make_unique<MIPToCartPoleStateConverter>());
    mip_to_cart_pole_state_converter->set_name("mip_to_cart_pole_state_converter");
    auto cart_pole_to_mip_torque_converter = builder.AddSystem(std::make_unique<CartPoleToMIPTorqueConverter>());
    cart_pole_to_mip_torque_converter->set_name("cart_pole_to_mip_torque_converter");
    builder.Connect(plant.get_continuous_state_output_port(), mip_to_cart_pole_state_converter->mip_state_input());
    builder.Connect(mip_to_cart_pole_state_converter->cart_pole_state_output(), controller->get_input_port());
    builder.Connect(controller->get_output_port(), cart_pole_to_mip_torque_converter->cart_pole_torque_input());
    builder.Connect(cart_pole_to_mip_torque_converter->mip_torque_output(), plant.get_actuation_input_port());

    printf("plant.get_continuous_state_output_port().size() = %d\n", plant.get_continuous_state_output_port().size());
    printf("plant num positions = %d\n", plant.num_positions());
    printf("plant num velocities = %d\n", plant.num_velocities());
    geometry::ConnectDrakeVisualizer(&builder, scene_graph);
    auto diagram = builder.Build();

    // Create a context for this system:
    std::unique_ptr<systems::Context<double>> diagram_context =
        diagram->CreateDefaultContext();
    diagram->SetDefaultContext(diagram_context.get());
    systems::Context<double>& context =
        diagram->GetMutableSubsystemContext(plant, diagram_context.get());

    //context.FixInputPort(plant.get_actuation_input_port().get_index(), Vector1d::Constant(50.0));

    // Get joints so that we can set initial conditions.
    const RevoluteJoint<double>& wheel_pole_joint = plant.GetJointByName<RevoluteJoint>("wheel_pole_joint");

    // Set initial state.
    wheel_pole_joint.set_angle(&context, 1.0);

    systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
    simulator.Initialize();
    simulator.StepTo(FLAGS_simulation_time);

    return 0;
}

int main(int argc, char* argv[]) {
    gflags::SetUsageMessage(
            "A segway demo using Drake's MultibodyPlant,"
            "with SceneGraph visualization. "
            "Launch drake-visualizer before running this example.");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::logging::HandleSpdlogGflags();
    return do_main();
}

