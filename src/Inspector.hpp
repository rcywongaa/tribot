#pragma once

#include <functional>

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/basic_vector.h>
#include <Eigen/Dense>

template <typename T>
using InspectionFuncImpl = std::function<void(const Eigen::VectorBlock<const drake::VectorX<T>>&)>;
using InspectionFuncImpl_double = InspectionFuncImpl<double>;
using InspectionFuncImpl_autodiff = InspectionFuncImpl<drake::AutoDiffXd>;
using InspectionFuncImpl_symbolic = InspectionFuncImpl<drake::symbolic::Expression>;

struct InspectionFunc
{
    InspectionFunc(
            InspectionFuncImpl_double double_impl,
            InspectionFuncImpl_autodiff autodiff_impl,
            InspectionFuncImpl_symbolic symbolic_impl)
    {
        this->double_impl = double_impl;
        this->autodiff_impl = autodiff_impl;
        this->symbolic_impl = symbolic_impl;
    }
    InspectionFuncImpl_double double_impl;
    InspectionFuncImpl_autodiff autodiff_impl;
    InspectionFuncImpl_symbolic symbolic_impl;
};

template <typename T>
class Inspector : public drake::systems::LeafSystem<T>
{
    public:
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Inspector);

        // This is no longer a copy constructor, it cannot access protected / private members!
        // Copy constructors cannot be templates
        template <typename U>
        explicit Inspector(const Inspector<U>& other) : Inspector<T>(other.getInspectFunc(), other.getPortSize()) {};

        Inspector(const InspectionFunc& func, const unsigned int port_size);
        const drake::systems::InputPort<T>& get_input_port() const;
        const drake::systems::OutputPort<T>& get_output_port() const;

        const InspectionFunc& getInspectFunc() const;
        const unsigned int getPortSize() const;
    private:
        void inspect(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const;
        int input_idx;
        int output_idx;
        const InspectionFunc& inspect_func;
        unsigned int port_size;
};

namespace drake {
namespace systems {
namespace scalar_conversion {
template <> struct Traits<Inspector> : public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake

