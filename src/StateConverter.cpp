#include "StateConverter.hpp"

using namespace drake;

template <typename T>
StateConverter<T>::StateConverter(ConversionFunc func, const unsigned int input_size, const unsigned int output_size) :
    systems::LeafSystem<T>(systems::SystemTypeTag<StateConverter>{}),
    input_idx(this->DeclareVectorInputPort("input_port", systems::BasicVector<T>(input_size)).get_index()),
    output_idx(this->DeclareVectorOutputPort("output_port", systems::BasicVector<T>(output_size), &StateConverter::convert).get_index())
{
    convert_func = func;
}

template <>
void StateConverter<double>::convert(const drake::systems::Context<double>& context, systems::BasicVector<double>* output) const
{
    const auto state = this->EvalVectorInput(context, input_idx)->get_value();
    auto mutable_output = output->get_mutable_value();
    convert_func.double_impl(state, mutable_output);
}

template <>
void StateConverter<drake::AutoDiffXd>::convert(const drake::systems::Context<drake::AutoDiffXd>& context, systems::BasicVector<drake::AutoDiffXd>* output) const
{
    const auto state = this->EvalVectorInput(context, input_idx)->get_value();
    auto mutable_output = output->get_mutable_value();
    convert_func.autodiff_impl(state, mutable_output);
}

template <>
void StateConverter<drake::symbolic::Expression>::convert(const drake::systems::Context<drake::symbolic::Expression>& context, systems::BasicVector<drake::symbolic::Expression>* output) const
{
    const auto state = this->EvalVectorInput(context, input_idx)->get_value();
    auto mutable_output = output->get_mutable_value();
    convert_func.symbolic_impl(state, mutable_output);
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
const ConversionFunc StateConverter<T>::getConvertFunc() const
{
    return convert_func;
}

template <typename T>
const unsigned int StateConverter<T>::getInputSize() const
{
    return input_size;
}

template <typename T>
const unsigned int StateConverter<T>::getOutputSize() const
{
    return output_size;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class StateConverter)
