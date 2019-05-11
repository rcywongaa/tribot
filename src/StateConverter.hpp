#pragma once

#include <functional>

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/basic_vector.h>
#include <Eigen/Dense>

template <typename T>
using ConversionFuncImpl = std::function<void(const Eigen::VectorBlock<const drake::VectorX<T>>&, Eigen::VectorBlock<drake::VectorX<T>>&)>;

using ConversionFuncImpl_double = ConversionFuncImpl<double>;
using ConversionFuncImpl_autodiff = ConversionFuncImpl<drake::AutoDiffXd>;
using ConversionFuncImpl_symbolic = ConversionFuncImpl<drake::symbolic::Expression>;

struct ConversionFunc
{
    ConversionFunc(
            ConversionFuncImpl_double double_impl,
            ConversionFuncImpl_autodiff autodiff_impl,
            ConversionFuncImpl_symbolic symbolic_impl)
    {
        this->double_impl = double_impl;
        this->autodiff_impl = autodiff_impl;
        this->symbolic_impl = symbolic_impl;
    }
    ConversionFuncImpl_double double_impl;
    ConversionFuncImpl_autodiff autodiff_impl;
    ConversionFuncImpl_symbolic symbolic_impl;
};

template <typename T>
class StateConverter : public drake::systems::LeafSystem<T>
{
    public:
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StateConverter);

        // This is no longer a copy constructor, it cannot access protected / private members!
        // Copy constructors cannot be templates
        template <typename U>
        explicit StateConverter(const StateConverter<U>& other) : StateConverter<T>(other.getConvertFunc(), other.getInputSize(), other.getOutputSize()) {};

        StateConverter(ConversionFunc func, const unsigned int input_size, const unsigned int output_size);
        const drake::systems::InputPort<T>& get_input_port() const;
        const drake::systems::OutputPort<T>& get_output_port() const;

        const ConversionFunc getConvertFunc() const;
        const unsigned int getInputSize() const;
        const unsigned int getOutputSize() const;
    private:
        void convert(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const;
        int input_idx;
        int output_idx;
        ConversionFunc convert_func;
        unsigned int input_size;
        unsigned int output_size;
};
