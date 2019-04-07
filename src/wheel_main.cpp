#include <memory>
#include <math.h>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
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
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/solvers/solve.h"

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

typedef trajectories::PiecewisePolynomial<double> PiecewisePolynomialType;

inline const std::string getSrcDir()
{
    return std::string(__FILE__).erase(std::string(__FILE__).rfind('/')) + "/";
}

DEFINE_double(target_realtime_rate, 1.0,
        "Desired rate relative to real time.  See documentation for "
        "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
        "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 0,
        "If greater than zero, the plant is modeled as a system with "
        "discrete updates and period equal to this time_step. "
        "If 0, the plant is modeled as a continuous system.");

// LQR won't work as this involves non-continuous contact dynamics
// Use trajectory optimization instead
int do_main() {
    systems::DiagramBuilder<double> builder;

    auto pair = AddMultibodyPlantSceneGraph(&builder, std::make_unique<MultibodyPlant<double>>(FLAGS_time_step));

    MultibodyPlant<double>& plant = pair.plant;

    SceneGraph<double>& scene_graph = pair.scene_graph;
    scene_graph.set_name("scene_graph");

    // Make and add the wheel model.
    const std::string wheel_model_filename = getSrcDir() + "/../res/wheel.sdf";
    Parser(&plant, &scene_graph).AddModelFromFile(wheel_model_filename);

    Vector3<double> normal_W(0, 0, 1);
    Vector3<double> point_W(0, 0, -0.19);

    const CoulombFriction<double> surface_friction(
            10.0 /* static friction */, 0.0 /* dynamic friction */);

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

    // Must be called after Finalize()
    plant.set_penetration_allowance(0.001);
    //plant.set_stiction_tolerance(1.0e-9);

    // Sanity check on the availability of the optional source id before using it.
    DRAKE_DEMAND(plant.geometry_source_is_registered());

    std::unique_ptr<systems::Context<double>> plant_context = plant.CreateDefaultContext();

    const int kNumTimeSamples = 21;
    const double kMinimumTimeStep = 0.2;
    const double kMaximumTimeStep = 0.5;
    systems::trajectory_optimization::DirectCollocation dircol(
        &plant, *plant_context, kNumTimeSamples, kMinimumTimeStep,
        kMaximumTimeStep);

    dircol.AddEqualTimeIntervalsConstraints();

    const double kTorqueLimit = 10;
    auto u = dircol.input();
    dircol.AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
    dircol.AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

    const Eigen::Matrix<double, 6, 1> x0;
    const Eigen::Matrix<double, 6, 1> xG;
    xG(0) = 10.0;
    dircol.AddLinearConstraint(dircol.initial_state() == x0);
    dircol.AddLinearConstraint(dircol.final_state() == xG);

    const double R = 10;  // Cost on input "effort".
    dircol.AddRunningCost((R * u) * u);

    const double timespan_init = 4;
    auto traj_init_x =
        PiecewisePolynomialType::FirstOrderHold({0, timespan_init}, {x0, xG});
    dircol.SetInitialTrajectory(PiecewisePolynomialType(), traj_init_x);
    const auto result = solvers::Solve(dircol);
    if (!result.is_success()) {
      std::cerr << "No solution found.\n";
      return 1;
    }

    const trajectories::PiecewisePolynomial<double> pp_xtraj =
        dircol.ReconstructStateTrajectory(result);
    auto state_source = builder.AddSystem<systems::TrajectorySource>(pp_xtraj);

    DrakeLcm lcm;
    auto publisher = builder.AddSystem<systems::DrakeVisualizer>(plant, &lcm);
    // By default, the simulator triggers a publish event at the end of each time
    // step of the integrator. However, since this system is only meant for
    // playback, there is no continuous state and the integrator does not even get
    // called. Therefore, we explicitly set the publish frequency for the
    // visualizer.
    publisher->set_publish_period(1.0 / 60.0);

    builder.Connect(state_source->get_output_port(),
                    publisher->get_input_port(0));

    geometry::ConnectDrakeVisualizer(&builder, scene_graph);

    auto diagram = builder.Build();

    systems::Simulator<double> simulator(*diagram);

    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
    simulator.Initialize();
    simulator.StepTo(FLAGS_simulation_time);

    return 0;
}

int main(int argc, char* argv[]) {
    gflags::SetUsageMessage(
            "A wheel demo using Drake's MultibodyPlant,"
            "with SceneGraph visualization. "
            "Launch drake-visualizer before running this example.");
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::logging::HandleSpdlogGflags();
    return do_main();
}

