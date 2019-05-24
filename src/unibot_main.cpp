#include <cmath>

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
#include "mip_util.hpp"
#include "StateConverter.hpp"
#include "Inspector.hpp"

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

template <typename T>
void unibot_to_mip_state(const Eigen::VectorBlock<const VectorX<T>>& state, Eigen::VectorBlock<VectorX<T>>& output)
{
    drake::math::RollPitchYaw<T> rpy(Eigen::Quaternion<T>(state[0], state[1], state[2], state[3]));
    output[0] = rpy.pitch_angle(); // theta (pitch of the rod)
    //printf("pitch of rod = %f\n", output[0]);
    output[1] = output[0] + state[8]; // psi (wheel angle) = pitch of rod + rodwheel angle
    //printf("wheel angle = %f\n", output[1]);
    output[2] = state[9]; // theta_dot
    output[3] = output[2] + state[16]; // psi_dot
}

// state[0] = q1
// state[1] = q2
// state[2] = q3
// state[3] = q4
// state[4] = x
// state[5] = y
// state[6] = z
// state[7] = phi
// state[8] = wheel_joint
// state[9] = roll_dot
// state[12] = x_dot
// state[13] = y_dot
// state[14] = z_dot
// state[15] = phi_dot
// state[16] = wheel_dot
template <typename T>
void inspect_unibot(const Eigen::VectorBlock<const VectorX<T>>& state)
{
    printf("x = %f\n", state[4]);
    printf("y = %f\n", state[5]);
    printf("z = %f\n", state[6]);
    //drake::math::RollPitchYaw<T> rpy(Eigen::Quaternion<T>(state[0], state[1], state[2], state[3]));
    //printf("roll = %f\n", rpy.roll_angle());
    //printf("pitch = %f\n", rpy.pitch_angle());
    //printf("yaw = %f\n", rpy.yaw_angle());
}

int main(int argc, char* argv[])
{
    gflags::SetUsageMessage(
            "A unibot demo using Drake's MultibodyPlant,"
            "with SceneGraph visualization. "
            "Launch drake-visualizer before running this example.");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::logging::HandleSpdlogGflags();

    DiagramBuilder<double> builder;

    MultibodyPlant<double>& plant = create_default_plant(getResDir() + "unibot.sdf", builder, -0.20);

    printf("plant.get_state_output_port().size() = %d\n", plant.get_state_output_port().size());
    printf("plant num positions = %d\n", plant.num_positions());
    printf("plant num velocities = %d\n", plant.num_velocities());

    ConversionFunc unibot_to_acrobot_func(
            unibot_to_acrobot_state<double>,
            unibot_to_acrobot_state<drake::AutoDiffXd>,
            unibot_to_acrobot_state<drake::symbolic::Expression>);

    ConversionFunc unibot_to_mip_func(
            unibot_to_mip_state<double>,
            unibot_to_mip_state<drake::AutoDiffXd>,
            unibot_to_mip_state<drake::symbolic::Expression>);
    auto unibot_acrobot_converter = builder.AddSystem(std::make_unique<StateConverter<double>>(unibot_to_acrobot_func, 16, 4));
    unibot_acrobot_converter->set_name("unibot_acrobot_converter");
    auto unibot_mip_converter = builder.AddSystem(std::make_unique<StateConverter<double>>(unibot_to_mip_func, 17, 4));
    unibot_mip_converter->set_name("unibot_mip_converter");

    auto acrobot_controller = builder.AddSystem(MakeAcrobotLQRController(getResDir() + "unibot_acrobot.sdf"));
    acrobot_controller->set_name("acrobot_controller");

    InspectionFunc inspect_unibot_func(
            inspect_unibot<double>,
            inspect_unibot<drake::AutoDiffXd>,
            inspect_unibot<drake::symbolic::Expression>);
    auto unibot_inspector = builder.AddSystem(std::make_unique<Inspector<double>>(inspect_unibot_func, 17));
    unibot_inspector->set_name("unibot_inspector");

    // Must be consistent with mip.rsdf
    const double m_r = 0.1; // rod mass
    const double l_l = 0.5; // load_position
    const double m_l = 0.4; // load mass
    const double rod_length = 1.0; // rod length

    MIPConfiguration cfg;
    cfg.M_w = 0.2; // wheel mass
    cfg.M_r = m_r + m_l; // total rod mass
    cfg.R = 0.2; // wheel radius
    cfg.L = rod_length/2.0; // half rod length
    cfg.I_w = 0.5*cfg.M_w*cfg.R*cfg.R; // wheel inertia
    cfg.I_r = m_r*rod_length*rod_length/3.0 + m_l*l_l*l_l; // rod inertia
    auto mip_controller = builder.AddSystem(std::make_unique<MIPController<double>>(cfg, 0.0));
    mip_controller->set_name("mip_controller");

    auto torque_converter = builder.AddSystem(std::make_unique<TorqueCombiner<double>>());
    torque_converter->set_name("torque_converter");

    builder.Connect(plant.get_state_output_port(), unibot_acrobot_converter->get_input_port());
    builder.Connect(unibot_acrobot_converter->get_output_port(), acrobot_controller->get_input_port());
    builder.Connect(acrobot_controller->get_output_port(), torque_converter->get_acrobot_input_port());

    builder.Connect(plant.get_state_output_port(), unibot_inspector->get_input_port());
    builder.Connect(unibot_inspector->get_output_port(), unibot_mip_converter->get_input_port());
    builder.Connect(unibot_mip_converter->get_output_port(), mip_controller->mip_state_input());
    builder.Connect(mip_controller->torque_output(), torque_converter->get_mip_input_port());

    builder.Connect(torque_converter->get_torque_output_port(), plant.get_actuation_input_port());

    auto diagram = builder.Build();

    std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();

    Context<double>& torque_converter_context = diagram->GetMutableSubsystemContext(*torque_converter, diagram_context.get());
    //torque_converter_context.FixInputPort(torque_converter->get_mip_input_port().get_index(), Vector1d::Zero());
    torque_converter_context.FixInputPort(torque_converter->get_acrobot_input_port().get_index(), Vector1d::Zero());

    // Create a plant_context for this system:
    Context<double>& plant_context = diagram->GetMutableSubsystemContext(plant, diagram_context.get());

    // Sanity check to make sure unibot falls forward and sideways correctly
    const RevoluteJoint<double>& theta = plant.GetJointByName<RevoluteJoint>("theta"); // pitch
    theta.set_angle(&plant_context, 0.10*M_PI);
    //const RevoluteJoint<double>& roll = plant.GetJointByName<RevoluteJoint>("roll");
    //roll.set_angle(&plant_context, 0.02*M_PI);
    //const RevoluteJoint<double>& yaw = plant.GetJointByName<RevoluteJoint>("yaw");
    //yaw.set_angle(&plant_context, 0.30);

    start_simulation(*diagram, std::move(diagram_context), FLAGS_target_realtime_rate);

    return 0;
}

