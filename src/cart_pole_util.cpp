#include "cart_pole_util.hpp"

#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"

using namespace drake;

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::PrismaticJoint;
using drake::multibody::RevoluteJoint;
using drake::multibody::UniformGravityFieldElement;
using systems::Context;

std::unique_ptr<systems::AffineSystem<double>> MakeCartPoleLQRController(const std::string full_model_filename)
{
    MultibodyPlant<double> plant;
    Parser parser(&plant);
    parser.AddModelFromFile(full_model_filename);

    // Add gravity to the model.
    plant.AddForceElement<UniformGravityFieldElement>();

    // Now the model is complete.
    plant.Finalize();

    std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();
    const int actuation_port_index = plant.get_actuation_input_port().get_index();
    context->FixInputPort(actuation_port_index, Vector1d::Constant(0.0));
    context->FixInputPort(
            plant.get_applied_generalized_force_input_port().get_index(),
            Eigen::Vector2d::Constant(0.0));
    printf("Plant q size: %d\n", plant.num_positions());
    printf("Plant v size: %d\n", plant.num_velocities());

    // Get joints so that we can set initial conditions.
    const PrismaticJoint<double>& cart_slider =
        plant.GetJointByName<PrismaticJoint>("CartSlider");
    const RevoluteJoint<double>& pole_pin =
        plant.GetJointByName<RevoluteJoint>("PolePin");

    // Set target state
    cart_slider.set_translation(context.get(), 0.0);
    cart_slider.set_translation_rate(context.get(), 0.0);
    pole_pin.set_angle(context.get(), M_PI);
    pole_pin.set_angular_rate(context.get(), 0.0);

    Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
    Q(0, 0) = 10;
    Q(1, 1) = 10;
    Vector1d R = Vector1d::Constant(1);

    return systems::controllers::LinearQuadraticRegulator(
            plant, *context, Q, R,
            Eigen::Matrix<double, 0, 0>::Zero() /* No cross state/control costs */,
            actuation_port_index);
}


