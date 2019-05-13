#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include <Eigen/Dense>

#include "meta.hpp"
#include "drake_util.hpp"
#include "acrobot_util.hpp"

using namespace Eigen;
using namespace drake;
using namespace drake::systems;
using namespace drake::multibody;


// This helper method makes an LQR controller to balance an acrobot model
// specified in the SDF file `file_name`.
std::unique_ptr<systems::AffineSystem<double>> MakeAcrobotLQRController(std::string filename)
{
    // LinearQuadraticRegulator() below requires the controller's model of the
    // plant to only have a single input port corresponding to the actuation.
    // Therefore we create a new model that meets this requirement. (a model
    // created along with a SceneGraph for simulation would also have input ports
    // to interact with that SceneGraph).
    MultibodyPlant<double> acrobot;
    Parser parser(&acrobot);
    parser.AddModelFromFile(filename);
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

