#include <gflags/gflags.h>

#include "drake/common/text_logging_gflags.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/common/eigen_types.h"
#include <Eigen/Dense>

#include "meta.hpp"
#include "drake_util.hpp"
#include "acrobot_util.hpp"

using namespace Eigen;
using namespace drake;
using namespace drake::systems;
using namespace drake::multibody;

DEFINE_double(target_realtime_rate, 1.0,
        "Desired rate relative to real time.  See documentation for "
        "Simulator::set_target_realtime_rate() for details.");

DEFINE_bool(is_controlled, true, "Whether robot is controlled");

int main(int argc, char* argv[])
{
    gflags::SetUsageMessage(
            "A acrobot demo using Drake's MultibodyPlant,"
            "with SceneGraph visualization. "
            "Launch drake-visualizer before running this example.");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::logging::HandleSpdlogGflags();

    DiagramBuilder<double> builder;

    std::string acrobot_model_filename = getResDir() + "unibot_acrobot.sdf";
    MultibodyPlant<double>& plant = create_default_plant(acrobot_model_filename, builder, -100.0);

    printf("plant.get_state_output_port().size() = %d\n", plant.get_state_output_port().size());
    printf("plant num positions = %d\n", plant.num_positions());
    printf("plant num velocities = %d\n", plant.num_velocities());

    auto controller = builder.AddSystem(MakeAcrobotLQRController(acrobot_model_filename));
    controller->set_name("acrobot_controller");
    builder.Connect(plant.get_state_output_port(),
            controller->get_input_port());
    builder.Connect(controller->get_output_port(),
            plant.get_actuation_input_port());

    auto diagram = builder.Build();

    // Create a context for this system:
    std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();
    Context<double>& context = diagram->GetMutableSubsystemContext(plant, diagram_context.get());

    if (!FLAGS_is_controlled) context.FixInputPort(plant.get_actuation_input_port().get_index(), Vector1d::Zero());
    // Get joints so that we can set initial conditions.
    const RevoluteJoint<double>& theta = plant.GetJointByName<RevoluteJoint>("theta");
    theta.set_angle(&context, 0.01*M_PI);

    start_simulation(*diagram, std::move(diagram_context), FLAGS_target_realtime_rate);

    return 0;
}
