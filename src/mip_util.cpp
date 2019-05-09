#include "mip_util.hpp"

#include <Eigen/Dense>
#include "drake/math/saturate.h"

using namespace Eigen;
using namespace drake;

// Must be consistent with mip.rsdf
const double g = 9.81; // gravity
const double M_w = 0.2; // wheel mass
const double M_r = 0.5; // rod mass
const double R = 0.2; // wheel radius
const double L = 1.0; // rod length
const double l = L/2.0; // half rod length
const double I_w = 0.5*M_w*R*R; // wheel inertia
const double I_r = M_r*L*L/3.0; // rod inertia

// Taken from
// http://renaissance.ucsd.edu/courses/mae143c/MIPdynamics.pdf
// Wheel angular acceleration (phi_dd) linearized coefficients
const double phi_dd_common_coeff = 1.0/(
        M_r*R*l -
        ((I_w + (M_w + M_r) * R*R) * (I_r + M_r*l*l)) / (M_r*R*l));
const double phi_dd_theta_coeff = phi_dd_common_coeff * M_r*g*l;
const double phi_dd_torque_coeff = phi_dd_common_coeff * (-(I_r + M_r*l*l) / (M_r*R*l) - 1);
// Rod angular acceleration (theta_dd) linearized coefficients
const double theta_dd_phi_dd_coeff = -(I_w + (M_w + M_r)*R*R)/(M_r*R*l);
const double theta_dd_torque_coeff = 1.0/(M_r*R*l) + theta_dd_phi_dd_coeff * phi_dd_torque_coeff;
const double theta_dd_theta_coeff = theta_dd_phi_dd_coeff * phi_dd_theta_coeff;

const Eigen::Matrix4d A((Eigen::Matrix4d() <<
            0, 0, 1, 0,
            0, 0, 0, 1,
            theta_dd_theta_coeff, 0, 0, 0,
            phi_dd_theta_coeff, 0, 0, 0).finished());
const Eigen::Vector4d B((Eigen::Vector4d() <<
            0,
            0,
            theta_dd_torque_coeff,
            phi_dd_torque_coeff).finished());

template <typename T>
MIPController<T>::MIPController() :
    systems::LeafSystem<T>(systems::SystemTypeTag<MIPController>{}),
    input_idx(this->DeclareVectorInputPort("MIP_state", systems::BasicVector<T>(4)).get_index()),
    output_idx(this->DeclareVectorOutputPort("torque", systems::BasicVector<T>(1), &MIPController::getOutputTorque).get_index())
{

    Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
    Q(0, 0) = 10; // theta
    Q(1, 1) = 1; // phi
    Q(2, 2) = 1; // theta_dot
    Q(3, 3) = 1; // phi_dot

    // Setting this too small will cause the linearization to fail due to overaggressive movement
    Vector1d R = Vector1d::Constant(1.0);

    systems::controllers::LinearQuadraticRegulatorResult lqr_result = systems::controllers::LinearQuadraticRegulator(A, B, Q, R);
    S = lqr_result.S;
    K = lqr_result.K;
}

template <typename T>
const drake::systems::OutputPort<T>& MIPController<T>::torque_output() const
{
    return drake::systems::System<T>::get_output_port(output_idx);
}

template <typename T>
const drake::systems::InputPort<T>& MIPController<T>::mip_state_input() const
{
    return drake::systems::System<T>::get_input_port(input_idx);
}

template <typename T>
void MIPController<T>::getOutputTorque(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const
{
    const auto x = this->EvalVectorInput(context, input_idx)->get_value();
    auto mutable_output = output->get_mutable_value();
    const Vector4<T> x0(0, 10, 0, 0);
    const Vector1<T> u_v = K * (x0 - x);
    mutable_output[0] = math::saturate(u_v(0), T(-1.0), T(1.0));
    //printf("theta = %f\n", x(0));
    //const T cost = (x - x0).dot(S * (x - x0));
    //printf("cost = %f\n", cost);
    //printf("output_torque = %f\n", mutable_output[0]);
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class MIPController)

template <typename T>
MIPStateSimplifier<T>::MIPStateSimplifier() :
    systems::LeafSystem<T>(systems::SystemTypeTag<MIPStateSimplifier>{}),
    input_idx(this->DeclareVectorInputPort("full_state", systems::BasicVector<T>(8)).get_index()),
    output_idx(this->DeclareVectorOutputPort("simplified_state", systems::BasicVector<T>(4), &MIPStateSimplifier::convert).get_index())
{}

template <typename T>
void MIPStateSimplifier<T>::convert(const drake::systems::Context<T>& context, systems::BasicVector<T>* output) const
{
    const auto state = this->EvalVectorInput(context, input_idx)->get_value();
    auto mutable_output = output->get_mutable_value();
    mutable_output[0] = state(2); // theta (pole angle)
    mutable_output[1] = state(2) + state(3); // phi (wheel angle) = theta + pole_wheel_angle
    mutable_output[2] = state(6); // theta_dot
    mutable_output[3] = state(6) + state(7); // phi_dot
}

template <typename T>
const drake::systems::InputPort<T>& MIPStateSimplifier<T>::get_full_state_input() const
{
    return drake::systems::System<T>::get_input_port(input_idx);
}

template <typename T>
const drake::systems::OutputPort<T>& MIPStateSimplifier<T>::get_simplified_state_output() const
{
    return drake::systems::System<T>::get_output_port(output_idx);
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class MIPStateSimplifier)

template <typename T>
MobileInvertedPendulumPlant<T>::MobileInvertedPendulumPlant() :
    systems::LeafSystem<T>(systems::SystemTypeTag<MobileInvertedPendulumPlant>{}),
    state_port_idx(this->DeclareVectorOutputPort("state_output", systems::BasicVector<T>(4), &MobileInvertedPendulumPlant::copyStateOut).get_index()),
    torque_port_idx(this->DeclareVectorInputPort("torque_input", systems::BasicVector<T>(1)).get_index())
{
    this->DeclareContinuousState(4);
}

template <typename T>
const drake::systems::OutputPort<T>& MobileInvertedPendulumPlant<T>::get_state_output() const
{
    return drake::systems::System<T>::get_output_port(state_port_idx);
}

template <typename T>
const drake::systems::InputPort<T>& MobileInvertedPendulumPlant<T>::get_torque_input() const
{
    return drake::systems::System<T>::get_input_port(torque_port_idx);
}

template <typename T>
void MobileInvertedPendulumPlant<T>::DoCalcTimeDerivatives(
        const systems::Context<T> &context,
        systems::ContinuousState<T> *derivatives) const
{
    Vector4<T> x = context.get_continuous_state_vector().CopyToVector();
    const Vector1<T> u = this->EvalVectorInput(context, torque_port_idx)->get_value();
    Vector4<T> x_d = A*x + B*u;
    derivatives->SetFromVector(x_d);
}

template <typename T>
void MobileInvertedPendulumPlant<T>::SetDefaultState(const systems::Context<T>&, systems::State<T>* state) const
{
    systems::VectorBase<T>& mutable_state = state->get_mutable_continuous_state().get_mutable_vector();
    mutable_state[0] = 0.0;
    mutable_state[1] = 0.0;
    mutable_state[2] = 0.0;
    mutable_state[3] = 0.0;
}

template <typename T>
void MobileInvertedPendulumPlant<T>::copyStateOut(const systems::Context<T> &context,
        systems::BasicVector<T> *output) const
{
    output->set_value(context.get_continuous_state_vector().CopyToVector());
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class MobileInvertedPendulumPlant)
