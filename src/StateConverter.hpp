#pragma once

#include <functional>

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/basic_vector.h>
#include <Eigen/Dense>

template <typename T>
struct ConversionFuncImpl
{
    using type = std::function<void(const Eigen::VectorBlock<const drake::VectorX<T>>&, Eigen::VectorBlock<drake::VectorX<T>>& )>;
};

template <typename T>
using ConversionFunc = typename ConversionFuncImpl<T>::type;

template <typename T>
class StateConverter : public drake::systems::LeafSystem<T>
{
    public:
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StateConverter);

        // This is no longer a copy constructor, it cannot access protected / private members!
        // Copy constructors cannot be templates
        template <typename U>
        explicit StateConverter(const StateConverter<U>& other) : StateConverter<T>(other.getConvertFunc()) {};

        StateConverter(ConversionFunc<T> func);
        const drake::systems::InputPort<T>& get_input_port() const;
        const drake::systems::OutputPort<T>& get_output_port() const;

        const ConversionFunc<T> getConvertFunc() const;
    private:
        void convert(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const;
        int input_idx;
        int output_idx;
        ConversionFunc<T> convert_func;
};
