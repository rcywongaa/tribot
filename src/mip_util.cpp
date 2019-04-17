#include "mip_util.hpp"

#include <Eigen/Dense>

using namespace Eigen;
using namespace drake;

MIPToCartPoleStateConverter::MIPToCartPoleStateConverter() :
    input_idx(this->DeclareVectorInputPort("MIP_state", systems::BasicVector<double>(8)).get_index()),
    output_idx(this->DeclareVectorOutputPort("cartpole_state", systems::BasicVector<double>(4), &MIPToCartPoleStateConverter::convert).get_index())
{}

void MIPToCartPoleStateConverter::convert(const drake::systems::Context<double>& context, systems::BasicVector<double>* output) const
{
    const auto state = this->EvalVectorInput(context, input_idx)->get_value();
    printf("X rotation = %f\n", state[2]);
    printf("X translation = %f\n", state[1]);
    auto mutable_output = output->get_mutable_value();
    mutable_output[0] = state[1]; // Use X translation as CartSlider position value
    mutable_output[1] = state[2]; // Use theta as PolePin position value
    mutable_output[2] = state[5]; // Use Xdot as CartSlider velocity value
    mutable_output[3] = state[6]; // Use theta_dot as PolePin velocity value
}

const drake::systems::InputPort<double>& MIPToCartPoleStateConverter::mip_state_input() const
{
    return drake::systems::System<double>::get_input_port(input_idx);
}

const drake::systems::OutputPort<double>& MIPToCartPoleStateConverter::cart_pole_state_output() const
{
    return drake::systems::System<double>::get_output_port(output_idx);
}

CartPoleToMIPTorqueConverter::CartPoleToMIPTorqueConverter() :
    input_idx(this->DeclareVectorInputPort("cartpole_torque", systems::BasicVector<double>(1)).get_index()),
    output_idx(this->DeclareVectorOutputPort("MIP_torque", systems::BasicVector<double>(1), &CartPoleToMIPTorqueConverter::convert).get_index())
{}

void CartPoleToMIPTorqueConverter::convert(const drake::systems::Context<double>& context, systems::BasicVector<double>* output) const
{
    const auto state = this->EvalVectorInput(context, input_idx)->get_value();
    printf("cart pole torque = %f\n", state[0]);
    auto mutable_output = output->get_mutable_value();
    mutable_output[0] = state[0];
}

const drake::systems::InputPort<double>& CartPoleToMIPTorqueConverter::cart_pole_torque_input() const
{
    return drake::systems::System<double>::get_input_port(input_idx);
}

const drake::systems::OutputPort<double>& CartPoleToMIPTorqueConverter::mip_torque_output() const
{
    return drake::systems::System<double>::get_output_port(output_idx);
}

