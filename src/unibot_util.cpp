#include "unibot_util.hpp"

using namespace drake;

template <typename T>
UnibotToAcrobotStateConverter<T>::UnibotToAcrobotStateConverter() :
    systems::LeafSystem<T>(systems::SystemTypeTag<UnibotToAcrobotStateConverter>{}),
    input_idx(this->DeclareVectorInputPort("unibot_state", systems::BasicVector<T>(8)).get_index()),
    output_idx(this->DeclareVectorOutputPort("acrobot_state", systems::BasicVector<T>(4), &UnibotToAcrobotStateConverter::convert).get_index())
{}

template <typename T>
void UnibotToAcrobotStateConverter<T>::convert(const drake::systems::Context<T>& context, systems::BasicVector<T>* output) const
{
    const auto state = this->EvalVectorInput(context, input_idx)->get_value();
    auto mutable_output = output->get_mutable_value();
    mutable_output[0] = state(4); // roll of the rod
    mutable_output[1] = state(6); // phi (angle between link1 and link2)
    mutable_output[2] = state(12); // roll_dot
    mutable_output[3] = state(14); // phi_dot
}

template <typename T>
const drake::systems::InputPort<T>& UnibotToAcrobotStateConverter<T>::get_unibot_state_input() const
{
    return drake::systems::System<T>::get_input_port(input_idx);
}

template <typename T>
const drake::systems::OutputPort<T>& UnibotToAcrobotStateConverter<T>::get_acrobot_state_output() const
{
    return drake::systems::System<T>::get_output_port(output_idx);
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class UnibotToAcrobotStateConverter)
