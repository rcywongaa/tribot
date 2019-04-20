#include "mip_util.hpp"

#include <Eigen/Dense>

using namespace Eigen;
using namespace drake;

const double g = 9.8; // gravity
const double M_w = 10.0; // wheel mass
const double M_r = 50.0; // rod mass
const double R = 0.2; // wheel radius
const double L = 1.0; // rod length
const double I_w = 0.5*M_w*R*R; // wheel inertia
const double I_r = M_r*L*L/3.0; // rod inertia

// Wheel angular acceleration (phi_dd) linearized coefficients
const double phi_dd_common_coeff = 1.0/(
        M_r*R*L -
        ((I_w + (M_w + M_r) * R*R) * (I_r + M_r*L*L)) / (M_r*R*L));
const double phi_dd_theta_coeff = phi_dd_common_coeff * M_r*g*L;
const double phi_dd_torque_coeff = phi_dd_common_coeff * ((I_r + M_r*L*L) / (M_r*R*L) - 1);

// Rod angular acceleration (theta_dd) linearized coefficients
const double theta_dd_phi_dd_coeff = -(I_w + (M_w + M_r)*R*R)/(M_r*R*L);
const double theta_dd_torque_coeff = 1.0/(M_r*R*L) + theta_dd_phi_dd_coeff * phi_dd_torque_coeff;
const double theta_dd_theta_coeff = theta_dd_phi_dd_coeff * phi_dd_theta_coeff;

std::unique_ptr<systems::controllers::PidController<double>> MakeMIPLQRController()
{
    Matrix4d A;
    A <<
        0, 0, 1, 0,
        0, 0, 0, 1,
        theta_dd_theta_coeff, 0, 0, 0,
        phi_dd_theta_coeff, 0, 0, 0;
    Vector4d B;
    B <<
        0,
        0,
        theta_dd_torque_coeff,
        phi_dd_torque_coeff;

    Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
    Q(0, 0) = 1;
    Q(1, 1) = 100;
    Vector1d R = Vector1d::Constant(1);

    auto result = systems::controllers::LinearQuadraticRegulator(A, B, Q, R);
    return std::make_unique<systems::controllers::PidController<double>>(result.K, Vector4d::Zero(), Vector4d::Zero());
}

MobileInvertedPendulumPlant::MobileInvertedPendulumPlant() :
    state_port_idx(DeclareVectorOutputPort("state_output", systems::BasicVector<double>(4), &MobileInvertedPendulumPlant::copyStateOutput).get_index()),
    torque_port_idx(DeclareVectorInputPort("torque_input", systems::BasicVector<double>(1)))
{
    DeclareContinuousState(4);
}

const drake::systems::OutputPort<double>& MobileInvertedPendulumPlant::get_state_output() const
{
    return drake::systems::System<double>::get_output_port(state_port_idx);
}

const drake::systems::InputPort<double>& MIPToCartPoleStateConverter::get_torque_input() const
{
    return drake::systems::System<double>::get_input_port(torque_port_idx);
}

void MobileInvertedPendulumPlant::DoCalcTimeDerivatives(
        const systems::Context<double> &context,
        systems::ContinuousState<double> *derivatives) const
{
    Vector4d x = context.get_continuous_state_vector().CopyToVector();
    const Vector1d u = this->EvalVectorInput(context, torque_port_idx)->get_value();
    Matrix4d A;
    A <<
        0, 0, 1, 0,
        0, 0, 0, 1,
        theta_dd_theta_coeff, 0, 0, 0,
        phi_dd_theta_coeff, 0, 0, 0;
    Vector4d B;
    B <<
        0,
        0,
        theta_dd_torque_coeff,
        phi_dd_torque_coeff;
    Vector4d x_d = A*x + B*u;
    derivatives->setFromVector(x_d);
}

void MobileInvertedPendulum::copyStateOut(const systems::Context<double> &context,
        systems::BasicVector<double> *output) const
{
    output->set_value(context.get_continuous_state_vector().CopyToVector());
}
