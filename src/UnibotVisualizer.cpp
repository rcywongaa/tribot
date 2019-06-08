#include "UnibotPlant.hpp"
#include "UnibotVisualizer.hpp"
#include "meta.hpp"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/geometry_visualization.h"

using namespace drake;

namespace MultibodyStateIndex
{
    enum MultibodyStateIndex
    {
        Q_W = 0;
        Q_X = 1;
        Q_Y = 2;
        Q_Z = 3;
        X = 4;
        Y = 5;
        Z = 6;
        ALPHA = 7; // link1-link2 joint
        BETA = 8; // link1-wheel joint
        AA_D_X = 9; // X component of angular velocity in angle-axis form
        AA_D_Y = 10; // Y component of angular velocity in angle-axis form
        AA_D_Z = 11; // Z component of angular velocity in angle-axis form
        X_D = 12;
        Y_D = 13;
        Z_D = 14;
        ALPHA_D = 15;
        BETA_D = 16;
        NUM_STATES = 17;
    }
}

using namespace MultibodyStateIndex;

const UnibotVisualizer* UnibotVisualizer::AddToBuilder(
        systems::DiagramBuilder<double>* builder,
        const systems::OutputPort<double>& unibot_state_port)
{
    auto scene_graph = builder->AddSystem<geometry::SceneGraph>();
    multibody::MultibodyPlant<double> mbp;
    multibody::Parser parser(&mbp, scene_graph);
    auto model_id = parser.AddModelFromFile(getResDir() + "unibot.sdf");
    mbp.Finalize();

    auto source_id = *mbp.get_source_id();
    auto frame_id = mbp.GetBodyFrameIdOrThrow(mbp.GetBodyIndices(model_id)[0]);

    auto visualizer = builder->AddSystem(
            std::unique_ptr<UnibotVisualizer>(new UnibotVisualizer(frame_id)));
    builder->Connect(
            unibot_state_port,
            visualizer->get_input_port(0));
    builder->Connect(
            visualizer->get_output_port(0),
            scene_graph->get_source_pose_port(source_id));

    geometry::ConnectDrakeVisualizer(builder, *scene_graph);

    return visualizer;
}

UnibotVisualizer::UnibotVisualizer(geometry::FrameId frame_id)
{
    frame_id_ = frame_id;
    this->DeclareVectorInputPort("state", systems::BasicVector<double>(UnbotStateIndex::NUM_STATES));
    this->DeclareAbstractOutputPort(
            "geometry_pose", &UnibotVisualizer::OutputGeometryPose);
}

UnibotVisualizer::~UnibotVisualizer() = default;

void UnibotVisualizer::OutputGeometryPose(
        const systems::Context<double>& context,
        geometry::FramePoseVector<double>* poses) const
{
    DRAKE_DEMAND(frame_id_.is_valid());

    const auto& state = get_input_port(0).Eval(context);
    math::RigidTransformd pose(
            math::RollPitchYawd(state.segment<3>(3)),
            state.head<3>());

    *poses = {{frame_id_, pose.GetAsIsometry3()}};
}
