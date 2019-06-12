#include "Inspector.hpp"

using namespace drake;

template <typename T>
Inspector<T>::Inspector(const InspectionFunc& func, const unsigned int port_size) :
    systems::LeafSystem<T>(systems::SystemTypeTag<Inspector>{}),
    port_size(port_size),
    input_idx(this->DeclareVectorInputPort("input_port", systems::BasicVector<T>(port_size)).get_index()),
    output_idx(this->DeclareVectorOutputPort("output_port", systems::BasicVector<T>(port_size), &Inspector::inspect).get_index()),
    inspect_func(func)
{}

template <>
void Inspector<double>::inspect(const drake::systems::Context<double>& context, systems::BasicVector<double>* output) const
{
    const auto input = this->EvalVectorInput(context, input_idx)->get_value();
    auto mutable_output = output->get_mutable_value();
    inspect_func.double_impl(input);
    mutable_output = input;
}

template <>
void Inspector<drake::AutoDiffXd>::inspect(const drake::systems::Context<drake::AutoDiffXd>& context, systems::BasicVector<drake::AutoDiffXd>* output) const
{
    const auto input = this->EvalVectorInput(context, input_idx)->get_value();
    auto mutable_output = output->get_mutable_value();
    inspect_func.autodiff_impl(input);
    mutable_output = input;
}

template <>
void Inspector<drake::symbolic::Expression>::inspect(const drake::systems::Context<drake::symbolic::Expression>& context, systems::BasicVector<drake::symbolic::Expression>* output) const
{
    const auto input = this->EvalVectorInput(context, input_idx)->get_value();
    auto mutable_output = output->get_mutable_value();
    inspect_func.symbolic_impl(input);
    mutable_output = input;
}

template <typename T>
const drake::systems::InputPort<T>& Inspector<T>::get_input_port() const
{
    return drake::systems::System<T>::get_input_port(input_idx);
}

template <typename T>
const drake::systems::OutputPort<T>& Inspector<T>::get_output_port() const
{
    return drake::systems::System<T>::get_output_port(output_idx);
}

template <typename T>
const InspectionFunc& Inspector<T>::getInspectFunc() const
{
    return inspect_func;
}

template <typename T>
const unsigned int Inspector<T>::getPortSize() const
{
    return port_size;
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class Inspector)
