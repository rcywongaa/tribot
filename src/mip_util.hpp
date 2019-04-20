#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/primitives/affine_system.h>
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include <drake/systems/controllers/pid_controller.h>

std::unique_ptr<drake::systems::controllers::PidController<double>> MakeMIPLQRController();

class MobileInvertedPendulumPlant : public drake::systems::LeafSystem<double>
{
    public:
        MobileInvertedPendulumPlant();
        const drake::systems::InputPort<double>& get_state_output() const;
        const drake::systems::OutputPort<double>& get_torque_input() const;
    private:
        void DoCalcTimeDerivatives(
                const systems::Context<T>& context,
                systems::ContinuousState<T>* derivatives) const override;
        int state_port_idx;
        int torque_port_idx;
};
