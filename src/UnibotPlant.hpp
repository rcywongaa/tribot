#include "drake/systems/framework/leaf_system.h"

namespace UnibotStateIndex
{
    enum UnibotStateIndex
    {
        X = 0,
        Y,
        Z,
        ROLL,
        PITCH,
        YAW,
        ALPHA, // link1 - link2
        BETA, // link1 - wheel
        X_D,
        Y_D,
        Z_D,
        ROLL_D,
        PITCH_D,
        YAW_D,
        ALPHA_D,
        BETA_D,
        NUM_STATES
    };
}

struct UnibotConfig
{
    double w_r; // wheel radius
    double w_m; // wheel mass
    double l1_m; // link1 mass
    double l1_l; // link1 length
    double l2_m; // link2 mass
    double l2_l; // link2 length
    double p_m; // point mass
};

template <typename T>
class UnibotPlant : public drake::systems::LeafSystem<T>
{
    public:
        /* Required to solve "does not support ToAutoDiffXd"
         * https://drake.mit.edu/doxygen_cxx/group__system__scalar__conversion.html
         */
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnibotPlant);

        /* Required to solve "does not support ToAutoDiffXd"
         * https://drake.mit.edu/doxygen_cxx/group__system__scalar__conversion.html
         */
        template <typename U>
        explicit UnibotPlant(const UnibotPlant<U>& other) : UnibotPlant<T>(other.getConfig()) {};

        UnibotPlant(UnibotConfig config);
        const drake::systems::OutputPort<T>& get_state_output_port() const;
        const drake::systems::InputPort<T>& get_actuation_input_port() const;
        void SetDefaultState(const drake::systems::Context<T>&, drake::systems::State<T>* state) const override;

        const UnibotConfig getConfig() const;
    private:
        void DoCalcTimeDerivatives(
                const drake::systems::Context<T>& context,
                drake::systems::ContinuousState<T>* derivatives) const override;
        void copyStateOut(const drake::systems::Context<T> &context, drake::systems::BasicVector<T> *output) const;
        int state_port_idx;
        int torque_port_idx;
        UnibotConfig cfg;
};
