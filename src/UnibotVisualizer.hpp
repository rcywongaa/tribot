#pragma once

#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"

class UnibotVisualizer final : public drake::systems::LeafSystem<double> {
    public:
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnibotVisualizer);
        ~UnibotVisualizer() final;

        /// Creates, adds, and connects a UnibotVisualizer system into the given
        /// `builder`.
        static const UnibotVisualizer* AddToBuilder(
                drake::systems::DiagramBuilder<double>* builder,
                const drake::systems::OutputPort<double>& unibot_state_port);

    private:
        explicit UnibotVisualizer(drake::geometry::FrameId frame_id);
        void OutputMultibodyState(const drake::systems::Context<double>&,
                drake::geometry::FramePoseVector<double>*) const;

        // The id for the unibot body.
        drake::geometry::FrameId frame_id_{};
};
