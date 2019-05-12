#include <gflags/gflags.h>
#include "drake/common/text_logging_gflags.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/tree/revolute_joint.h"

#include "meta.hpp"
#include "drake_util.hpp"
#include "unibot_util.hpp"
#include "acrobot_util.hpp"
#include "StateConverter.hpp"

using namespace Eigen;
using namespace drake;
using namespace drake::systems;
using namespace drake::multibody;

DEFINE_double(target_realtime_rate, 1.0,
        "Desired rate relative to real time.  See documentation for "
        "Simulator::set_target_realtime_rate() for details.");

template <typename T>
void unibot_to_acrobot_state(const Eigen::VectorBlock<const VectorX<T>>& state, Eigen::VectorBlock<VectorX<T>>& output)
{
    output[0] = state[4]; // roll of the rod
    output[1] = state[6]; // phi (angle between link1 and link2)
    output[2] = state[12]; // roll_dot
    output[3] = state[14]; // phi_dot
}

int main(int argc, char* argv[])
{
    gflags::SetUsageMessage(
            "A acrobot demo using Drake's MultibodyPlant,"
            "with SceneGraph visualization. "
            "Launch drake-visualizer before running this example.");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::logging::HandleSpdlogGflags();

    DiagramBuilder<double> builder;

    MultibodyPlant<double>& plant = create_default_plant(getResDir() + "unibot.sdf", builder, -0.20);

    printf("plant.get_state_output_port().size() = %d\n", plant.get_state_output_port().size());
    printf("plant num positions = %d\n", plant.num_positions());
    printf("plant num velocities = %d\n", plant.num_velocities());

    ConversionFunc func(
            unibot_to_acrobot_state<double>,
            unibot_to_acrobot_state<drake::AutoDiffXd>,
            unibot_to_acrobot_state<drake::symbolic::Expression>);

    auto unibot_acrobot_converter = builder.AddSystem(std::make_unique<StateConverter<double>>(func, 16, 4));
    unibot_acrobot_converter->set_name("unibot_acrobot_converter");
    auto acrobot_controller = builder.AddSystem(MakeAcrobotLQRController());
    acrobot_controller->set_name("acrobot_controller");
    auto torque_converter = builder.AddSystem(std::make_unique<TorqueCombiner<double>>());
    torque_converter->set_name("torque_converter");

    builder.Connect(plant.get_state_output_port(), unibot_acrobot_converter->get_input_port());
    builder.Connect(unibot_acrobot_converter->get_output_port(), acrobot_controller->get_input_port());
    builder.Connect(acrobot_controller->get_output_port(), torque_converter->get_acrobot_input_port());

    builder.Connect(torque_converter->get_torque_output_port(), plant.get_actuation_input_port());

    auto diagram = builder.Build();

    std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();

    Context<double>& torque_converter_context = diagram->GetMutableSubsystemContext(*torque_converter, diagram_context.get());
    torque_converter_context.FixInputPort(torque_converter->get_mip_input_port().get_index(), Vector1d::Zero());

    // Create a plant_context for this system:
    Context<double>& plant_context = diagram->GetMutableSubsystemContext(plant, diagram_context.get());

    // Get joints so that we can set initial conditions.
    const RevoluteJoint<double>& phi = plant.GetJointByName<RevoluteJoint>("phi");
    phi.set_angle(&plant_context, 0.20);

    // Sanity check to make sure unibot falls forward and sideways correctly
    //const RevoluteJoint<double>& theta = plant.GetJointByName<RevoluteJoint>("theta");
    //theta.set_angle(&plant_context, 0.30);
    //const RevoluteJoint<double>& roll = plant.GetJointByName<RevoluteJoint>("roll");
    //roll.set_angle(&plant_context, 0.30);
    //const RevoluteJoint<double>& yaw = plant.GetJointByName<RevoluteJoint>("yaw");
    //yaw.set_angle(&plant_context, 0.30);

    start_simulation(*diagram, std::move(diagram_context), FLAGS_target_realtime_rate);

    return 0;
}

