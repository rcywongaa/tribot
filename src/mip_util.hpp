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

template <typename T>
class MobileInvertedPendulumPlant : public drake::systems::LeafSystem<T>
{
    public:
        /* Required to solve "does not support ToAutoDiffXd"
         * https://drake.mit.edu/doxygen_cxx/group__system__scalar__conversion.html
         */
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobileInvertedPendulumPlant);

        /* Required to solve "does not support ToAutoDiffXd"
         * https://drake.mit.edu/doxygen_cxx/group__system__scalar__conversion.html
         */
        template <typename U>
            explicit MobileInvertedPendulumPlant(const MobileInvertedPendulumPlant<U>& other) : MobileInvertedPendulumPlant<T>() {}

        MobileInvertedPendulumPlant();
        const drake::systems::OutputPort<T>& get_state_output() const;
        const drake::systems::InputPort<T>& get_torque_input() const;
        void SetDefaultState(const drake::systems::Context<T>&, drake::systems::State<T>* state) const override;

    private:
        void DoCalcTimeDerivatives(
                const drake::systems::Context<T>& context,
                drake::systems::ContinuousState<T>* derivatives) const override;
        void copyStateOut(const drake::systems::Context<T> &context, drake::systems::BasicVector<T> *output) const;
        int state_port_idx;
        int torque_port_idx;
};
