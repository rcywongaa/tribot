#include "unibot_util.hpp"

using namespace drake;

template <typename T>
TorqueCombiner<T>::TorqueCombiner() :
    systems::LeafSystem<T>(systems::SystemTypeTag<TorqueCombiner>{}),
    input_acrobot_idx(this->DeclareVectorInputPort("acrobot_torque", systems::BasicVector<T>(1)).get_index()),
    input_mip_idx(this->DeclareVectorInputPort("mip_torque", systems::BasicVector<T>(1)).get_index()),
    output_idx(this->DeclareVectorOutputPort("torques", systems::BasicVector<T>(2), &TorqueCombiner::convert).get_index())
{}

template <typename T>
void TorqueCombiner<T>::convert(const drake::systems::Context<T>& context, systems::BasicVector<T>* output) const
{
    //const auto mip_torque = this->EvalVectorInput(context, input_mip_idx)->get_value();
    const auto acrobot_torque = this->EvalVectorInput(context, input_acrobot_idx)->get_value();
    auto mutable_output = output->get_mutable_value();
    mutable_output[0] = acrobot_torque[0];
    //mutable_output[1] = mip_torque[0];
    mutable_output[1] = 0.0;
}

template <typename T>
const drake::systems::InputPort<T>& TorqueCombiner<T>::get_acrobot_input_port() const
{
    return drake::systems::System<T>::get_input_port(input_acrobot_idx);
}

template <typename T>
const drake::systems::InputPort<T>& TorqueCombiner<T>::get_mip_input_port() const
{
    return drake::systems::System<T>::get_input_port(input_mip_idx);
}

template <typename T>
const drake::systems::OutputPort<T>& TorqueCombiner<T>::get_torque_output_port() const
{
    return drake::systems::System<T>::get_output_port(output_idx);
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class TorqueCombiner)
