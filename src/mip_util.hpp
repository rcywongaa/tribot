#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/basic_vector.h>


class MIPToCartPoleStateConverter : public drake::systems::LeafSystem<double>
{
    public:
        MIPToCartPoleStateConverter();
        const drake::systems::InputPort<double>& mip_state_input() const;
        const drake::systems::OutputPort<double>& cart_pole_state_output() const;
    private:
        void convert(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* output) const;
        int input_idx;
        int output_idx;
};

class CartPoleToMIPTorqueConverter : public drake::systems::LeafSystem<double>
{
    public:
        CartPoleToMIPTorqueConverter();
        const drake::systems::InputPort<double>& cart_pole_torque_input() const;
        const drake::systems::OutputPort<double>& mip_torque_output() const;
    private:
        void convert(const drake::systems::Context<double>& context, drake::systems::BasicVector<double>* output) const;
        int input_idx;
        int output_idx;
};
