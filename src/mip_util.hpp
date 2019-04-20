#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/primitives/affine_system.h>
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include <drake/systems/framework/state.h>

std::unique_ptr<drake::systems::AffineSystem<double>> MakeMIPLQRController();

class MobileInvertedPendulumPlant : public drake::systems::LeafSystem<double>
{
    public:
        MobileInvertedPendulumPlant();
        const drake::systems::OutputPort<double>& get_state_output() const;
        const drake::systems::InputPort<double>& get_torque_input() const;
        void SetDefaultState(const drake::systems::Context<double>&, drake::systems::State<double>* state) const override;

    private:
        void DoCalcTimeDerivatives(
                const drake::systems::Context<double>& context,
                drake::systems::ContinuousState<double>* derivatives) const override;
        void copyStateOut(const drake::systems::Context<double> &context, drake::systems::BasicVector<double> *output) const;
        int state_port_idx;
        int torque_port_idx;
};
