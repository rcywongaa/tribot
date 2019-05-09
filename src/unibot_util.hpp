#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/primitives/affine_system.h>

template <typename T>
class UnibotToAcrobotStateConverter : public drake::systems::LeafSystem<T>
{
    public:
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnibotToAcrobotStateConverter);

        template <typename U>
        explicit UnibotToAcrobotStateConverter(const UnibotToAcrobotStateConverter<U>& other) : UnibotToAcrobotStateConverter<T>() {}

        UnibotToAcrobotStateConverter();
        const drake::systems::InputPort<T>& get_unibot_state_input() const;
        const drake::systems::OutputPort<T>& get_acrobot_state_output() const;
    private:
        void convert(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const;
        int input_idx;
        int output_idx;
};
