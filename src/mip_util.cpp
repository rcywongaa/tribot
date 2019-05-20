#include "mip_util.hpp"

#include <Eigen/Dense>
#include "drake/math/saturate.h"

using namespace Eigen;
using namespace drake;

const double g = 9.81; // gravity

template <typename T>
MIPController<T>::MIPController(MIPConfiguration mip_config, double target_wheel_rotation) :
    systems::LeafSystem<T>(systems::SystemTypeTag<MIPController>{}),
    input_idx(this->DeclareVectorInputPort("MIP_state", systems::BasicVector<T>(4)).get_index()),
    output_idx(this->DeclareVectorOutputPort("torque", systems::BasicVector<T>(1), &MIPController::getOutputTorque).get_index()),
    config(mip_config),
    M_w(config.M_w),
    M_r(config.M_r),
    R(config.R),
    L(config.L),
    I_w(config.I_w),
    I_r(config.I_r),
    target_wheel_rotation(target_wheel_rotation)
{
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
    const Vector4<T> x0(0, target_wheel_rotation, 0, 0);
    const Vector1<T> u_v = K * (x0 - x);
    mutable_output[0] = math::saturate(u_v(0), T(-1.0), T(1.0));
    //printf("theta = %f\n", x(0));
    //const T cost = (x - x0).dot(S * (x - x0));
    //printf("cost = %f\n", cost);
    //printf("output_torque = %f\n", mutable_output[0]);
}

template <typename T>
const MIPConfiguration MIPController<T>::getConfig() const
{
    return config;
}

template <typename T>
const double MIPController<T>::getTargetWheelRotation() const
{
    return target_wheel_rotation;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class MIPController)
