#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include <Eigen/Dense>

#include "meta.hpp"
#include "drake_util.hpp"

using namespace Eigen;
using namespace drake;
using namespace drake::systems;
using namespace drake::multibody;

DEFINE_double(target_realtime_rate, 1.0,
        "Desired rate relative to real time.  See documentation for "
        "Simulator::set_target_realtime_rate() for details.");

// This helper method makes an LQR controller to balance an acrobot model
// specified in the SDF file `file_name`.
std::unique_ptr<systems::AffineSystem<double>> MakeAcrobotLQRController()
{
    // LinearQuadraticRegulator() below requires the controller's model of the
    // plant to only have a single input port corresponding to the actuation.
    // Therefore we create a new model that meets this requirement. (a model
    // created along with a SceneGraph for simulation would also have input ports
    // to interact with that SceneGraph).
    MultibodyPlant<double> acrobot;
    Parser parser(&acrobot);
    parser.AddModelFromFile(getResDir() + "acrobot.sdf");
    // Add gravity to the model.
    acrobot.AddForceElement<UniformGravityFieldElement>();
    // We are done defining the model.
    acrobot.Finalize();

    const RevoluteJoint<double>& theta = acrobot.GetJointByName<RevoluteJoint>("theta");
    const RevoluteJoint<double>& phi = acrobot.GetJointByName<RevoluteJoint>("phi");
    std::unique_ptr<Context<double>> context = acrobot.CreateDefaultContext();

    // Set nominal actuation torque to zero.
    const int actuation_port_index = acrobot.get_actuation_input_port().get_index();
    context->FixInputPort(actuation_port_index, Vector1d::Constant(0.0));
    context->FixInputPort(
            acrobot.get_applied_generalized_force_input_port().get_index(),
            Vector2d::Constant(0.0));

    theta.set_angle(context.get(), 0.0);
    theta.set_angular_rate(context.get(), 0.0);
    phi.set_angle(context.get(), 0.0);
    phi.set_angular_rate(context.get(), 0.0);

    // Setup LQR Cost matrices (penalize position error 10x more than velocity
    // to roughly address difference in units, using sqrt(g/l) as the time
    // constant.
    Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
    Q(0, 0) = 10;
    Q(1, 1) = 10;
    Q(2, 2) = 1;
    Q(3, 3) = 1;
    Vector1d R = Vector1d::Constant(0.1);

    return systems::controllers::LinearQuadraticRegulator(
            acrobot, *context, Q, R,
            Eigen::Matrix<double, 0, 0>::Zero() /* No cross state/control costs */,
            actuation_port_index);
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

    MultibodyPlant<double>& plant = create_default_plant(getResDir() + "acrobot.sdf", builder, -100.0);

    printf("plant.get_continuous_state_output_port().size() = %d\n", plant.get_continuous_state_output_port().size());
    printf("plant num positions = %d\n", plant.num_positions());
    printf("plant num velocities = %d\n", plant.num_velocities());

    auto controller = builder.AddSystem(MakeAcrobotLQRController());
    controller->set_name("acrobot_controller");
    builder.Connect(plant.get_continuous_state_output_port(),
            controller->get_input_port());
    builder.Connect(controller->get_output_port(),
            plant.get_actuation_input_port());

    auto diagram = builder.Build();

    // Create a context for this system:
    std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();
    Context<double>& context = diagram->GetMutableSubsystemContext(plant, diagram_context.get());

    //context.FixInputPort(plant.get_actuation_input_port().get_index(), Vector1d::Zero());
    // Get joints so that we can set initial conditions.
    const RevoluteJoint<double>& theta = plant.GetJointByName<RevoluteJoint>("theta");
    theta.set_angle(&context, 0.20);

    start_simulation(*diagram, std::move(diagram_context), FLAGS_target_realtime_rate);

    return 0;
}
