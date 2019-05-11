#include <memory>
#include <math.h>

#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/affine_system.h"

#include "meta.hpp"
#include "mip_util.hpp"
#include "drake_util.hpp"
#include "StateConverter.hpp"

using namespace drake;

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::MultibodyPlant;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::systems::Context;

DEFINE_double(target_realtime_rate, 1.0,
        "Desired rate relative to real time.  See documentation for "
        "Simulator::set_target_realtime_rate() for details.");

template <typename T>
void to_mip_state(const Eigen::VectorBlock<const VectorX<T>>& state, Eigen::VectorBlock<VectorX<T>>& output)
{
    output[0] = state[2]; // theta (pole angle)
    output[1] = state[2] + state[3]; // phi (wheel angle) = theta + pole_wheel_angle
    output[2] = state[6]; // theta_dot
    output[3] = state[6] + state[7]; // phi_dot
}

int do_main() {
    systems::DiagramBuilder<double> builder;

    MultibodyPlant<double>& plant = create_default_plant(getResDir() + "mip.sdf", builder, -0.25);

    printf("plant.get_state_output_port().size() = %d\n", plant.get_state_output_port().size());
    printf("plant num positions = %d\n", plant.num_positions());
    printf("plant num velocities = %d\n", plant.num_velocities());

    // Create MIP LQR and connect simulated MIP to LQR
    auto controller = builder.AddSystem(std::make_unique<MIPController<double>>());
    controller->set_name("MIP_controller");

    ConversionFunc func(
            to_mip_state<double>,
            to_mip_state<drake::AutoDiffXd>,
            to_mip_state<drake::symbolic::Expression>);

    auto converter = builder.AddSystem(std::make_unique<StateConverter<double>>(func, 8, 4));
    converter->set_name("MIP_converter");
    builder.Connect(plant.get_state_output_port(), converter->get_input_port());
    builder.Connect(converter->get_output_port(), controller->mip_state_input());
    builder.Connect(controller->torque_output(), plant.get_actuation_input_port());

    auto diagram = builder.Build();

    // Create a context for this system:
    std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();
    systems::Context<double>& context = diagram->GetMutableSubsystemContext(plant, diagram_context.get());

    // Get joints so that we can set initial conditions.
    const RevoluteJoint<double>& theta = plant.GetJointByName<RevoluteJoint>("theta");
    theta.set_angle(&context, 0.2);

    start_simulation(*diagram, std::move(diagram_context), FLAGS_target_realtime_rate);

    return 0;
}

int main(int argc, char* argv[]) {
    gflags::SetUsageMessage(
            "A mip demo using Drake's MultibodyPlant,"
            "with SceneGraph visualization. "
            "Launch drake-visualizer before running this example.");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::logging::HandleSpdlogGflags();
    return do_main();
}

