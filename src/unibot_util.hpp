#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/primitives/affine_system.h>

template <typename T>
class TorqueCombiner : public drake::systems::LeafSystem<T>
{
    public:
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TorqueCombiner);

        template <typename U>
        explicit TorqueCombiner(const TorqueCombiner<U>& other) : TorqueCombiner<T>() {}

        TorqueCombiner();
        const drake::systems::InputPort<T>& get_mip_input_port() const;
        const drake::systems::InputPort<T>& get_acrobot_input_port() const;
        const drake::systems::OutputPort<T>& get_torque_output_port() const;
    private:
        void convert(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const;
        int input_mip_idx;
        int input_acrobot_idx;
        int output_idx;
};
