#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/basic_vector.h>
#include <drake/systems/primitives/affine_system.h>
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include <drake/systems/framework/state.h>
#include <Eigen/Dense>

template <typename T>
class MIPController : public drake::systems::LeafSystem<T>
{
    public:
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MIPController);

        template <typename U>
        explicit MIPController(const MIPController<U>& other) : MIPController<T>() {}

        MIPController();
        const drake::systems::InputPort<T>& mip_state_input() const;
        const drake::systems::OutputPort<T>& torque_output() const;
    private:
        Eigen::Matrix4d S;
        Eigen::RowVector4d K;
        void getOutputTorque(const drake::systems::Context<T>& context, drake::systems::BasicVector<T>* output) const;
        int input_idx;
        int output_idx;
};
