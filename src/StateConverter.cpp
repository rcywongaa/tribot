#include "StateConverter.hpp"

using namespace drake;

template struct ConversionFuncImpl<drake::symbolic::Expression>;
template struct ConversionFuncImpl<double>;
template struct ConversionFuncImpl<drake::AutoDiffXd>;

template <typename T>
StateConverter<T>::StateConverter(ConversionFunc<T> func) :
    systems::LeafSystem<T>(systems::SystemTypeTag<StateConverter>{}),
    input_idx(this->DeclareVectorInputPort("input_port", systems::BasicVector<T>(8)).get_index()),
    output_idx(this->DeclareVectorOutputPort("output_port", systems::BasicVector<T>(4), &StateConverter::convert).get_index())
{
    convert_func = func;
}

template <typename T>
void StateConverter<T>::convert(const drake::systems::Context<T>& context, systems::BasicVector<T>* output) const
{
    const auto state = this->EvalVectorInput(context, input_idx)->get_value();
    auto mutable_output = output->get_mutable_value();
    convert_func(state, mutable_output);
}

template <typename T>
const drake::systems::InputPort<T>& StateConverter<T>::get_input_port() const
{
    return drake::systems::System<T>::get_input_port(input_idx);
}

template <typename T>
const drake::systems::OutputPort<T>& StateConverter<T>::get_output_port() const
{
    return drake::systems::System<T>::get_output_port(output_idx);
}

template <typename T>
const ConversionFunc<T> StateConverter<T>::getConvertFunc() const
{
    return convert_func;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class StateConverter)
