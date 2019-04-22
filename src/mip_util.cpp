#include "mip_util.hpp"

#include <Eigen/Dense>

using namespace Eigen;
using namespace drake;

const double g = 9.81; // gravity
const double M_w = 10.0; // wheel mass
const double M_r = 50.0; // rod mass
const double R = 0.2; // wheel radius
const double L = 0.5; // half rod length
const double I_w = 0.5*M_w*R*R; // wheel inertia
const double I_r = M_r*L*L/3.0; // rod inertia

// Taken from
// http://renaissance.ucsd.edu/courses/mae143c/MIPdynamics.pdf
// Wheel angular acceleration (phi_dd) linearized coefficients
const double phi_dd_common_coeff = 1.0/(
        M_r*R*L -
        ((I_w + (M_w + M_r) * R*R) * (I_r + M_r*L*L)) / (M_r*R*L));
const double phi_dd_theta_coeff = phi_dd_common_coeff * M_r*g*L;
const double phi_dd_torque_coeff = phi_dd_common_coeff * (-(I_r + M_r*L*L) / (M_r*R*L) - 1);
// Rod angular acceleration (theta_dd) linearized coefficients
const double theta_dd_phi_dd_coeff = -(I_w + (M_w + M_r)*R*R)/(M_r*R*L);
const double theta_dd_torque_coeff = 1.0/(M_r*R*L) + theta_dd_phi_dd_coeff * phi_dd_torque_coeff;
const double theta_dd_theta_coeff = theta_dd_phi_dd_coeff * phi_dd_theta_coeff;

std::unique_ptr<systems::AffineSystem<double>> MakeMIPLQRController()
{
    MobileInvertedPendulumPlant<double> plant;
    auto context = plant.CreateDefaultContext();
    Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
    context->FixInputPort(plant.get_torque_input().get_index(), Vector1d::Constant(0.0));
    Q(0, 0) = 10;
    Q(1, 1) = 1;
    Q(2, 2) = 2;
    Q(3, 3) = 4;
    Vector1d R = Vector1d::Constant(1);

    return systems::controllers::LinearQuadraticRegulator(plant, *context, Q, R);
}

template <typename T>
MIPStateSimplifier<T>::MIPStateSimplifier() :
    systems::LeafSystem<T>(systems::SystemTypeTag<MIPStateSimplifier>{}),
    input_idx(this->DeclareVectorInputPort("MIP_state", systems::BasicVector<T>(8)).get_index()),
    output_idx(this->DeclareVectorOutputPort("cartpole_state", systems::BasicVector<T>(4), &MIPStateSimplifier::convert).get_index())
{}

template <typename T>
void MIPStateSimplifier<T>::convert(const drake::systems::Context<T>& context, systems::BasicVector<T>* output) const
{
    const auto state = this->EvalVectorInput(context, input_idx)->get_value();
    auto mutable_output = output->get_mutable_value();
    mutable_output[0] = state[2]; // theta (pole angle)
    mutable_output[1] = state[2] + state[3]; // phi (wheel angle) = theta + pole_wheel_angle
    mutable_output[2] = state[6]; // theta_dot
    mutable_output[3] = state[6] + state[7]; // phi_dot
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
    Matrix4<T> A;
    A <<
        0, 0, 1, 0,
        0, 0, 0, 1,
        theta_dd_theta_coeff, 0, 0, 0,
        phi_dd_theta_coeff, 0, 0, 0;
    Vector4<T> B;
    B <<
        0,
        0,
        theta_dd_torque_coeff,
        phi_dd_torque_coeff;
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

