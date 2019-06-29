#include "UnibotPlant.hpp"

#include <cmath>

using namespace drake;

const double g = 9.81;

template <typename T>
UnibotPlant<T>::UnibotPlant(UnibotConfig config) :
    systems::LeafSystem<T>(systems::SystemTypeTag<UnibotPlant>{}),
    cfg(config),
    state_port_idx(this->DeclareVectorOutputPort("state_output", systems::BasicVector<T>(UnibotStateIndex::NUM_STATES), &UnibotPlant::copyStateOut, {this->all_state_ticket()}).get_index()),
    pose_port_idx(this->DeclareVectorOutputPort("pose_output", systems::BasicVector<T>(UnibotPoseIndex::NUM_POSES), &UnibotPlant::copyPoseOut, {this->all_state_ticket()}).get_index()),
    torque_port_idx(this->DeclareVectorInputPort("torque_input", systems::BasicVector<T>(2)).get_index())
{
    //x, y, z, roll, pitch, yaw, alpha, beta, + velocities)
    this->DeclareContinuousState(UnibotStateIndex::NUM_STATES);
}

template <typename T>
const drake::systems::OutputPort<T>& UnibotPlant<T>::get_state_output_port() const
{
    return drake::systems::System<T>::get_output_port(state_port_idx);
}

template <typename T>
const drake::systems::OutputPort<T>& UnibotPlant<T>::get_pose_output_port() const
{
    return drake::systems::System<T>::get_output_port(pose_port_idx);
}

template <typename T>
const drake::systems::InputPort<T>& UnibotPlant<T>::get_actuation_input_port() const
{
    return drake::systems::System<T>::get_input_port(torque_port_idx);
}

template <typename T>
const UnibotConfig UnibotPlant<T>::getConfig() const
{
    return cfg;
}

template <typename T>
drake::multibody::MultibodyPlant<T>& UnibotPlant<T>::get_visual_mbp()
{
    return mbp;
}

template <typename T>
void UnibotPlant<T>::DoCalcTimeDerivatives(
        const systems::Context<T> &context,
        systems::ContinuousState<T> *derivatives) const
{
    using namespace UnibotStateIndex;
    // Reference frame is at the center of the wheel
    const systems::VectorBase<T>& x = context.get_continuous_state_vector();
    const Vector2<T> u = this->EvalVectorInput(context, torque_port_idx)->get_value();
    Eigen::Matrix<T, UnibotStateIndex::NUM_STATES, 1> x_d;
    x_d[X] = x[X_D];
    x_d[Y] = x[Y_D];
    x_d[Z] = x[Z_D];
    x_d[ROLL] = x[ROLL_D];
    x_d[PITCH] = x[PITCH_D];
    x_d[YAW] = x[YAW_D];
    x_d[ALPHA] = x[ALPHA_D];
    x_d[BETA] = x[BETA_D];

    using std::cos;
    using std::sin;
    using std::pow;
    T w_b_i = 0.5*cfg.w_m*pow(cfg.w_r, 2); // wheel moment of inertia w.r.t beta joint
    T w_g_i = w_b_i + cfg.w_m*pow(cfg.w_r, 2); // wheel moment of inertia w.r.t. ground
    T l1_i = 1.0/3.0*cfg.l1_m*pow(cfg.l1_l, 2); // l1 moment of inertia
    T l2_b_i = 1.0/3.0*cfg.l2_m*pow(cfg.l1_l - cfg.l2_l*cos(x[ALPHA]), 2); // l2 instantaneous moment of inertia w.r.t beta joint
    T l2_a_i = 1.0/3.0*cfg.l2_m*pow(cfg.l2_l, 2); // l2 moment of inertia w.r.t. alpha joint
    T p_b_i = cfg.p_m*pow(cfg.l1_l - cfg.l2_l*cos(x[ALPHA]), 2); // point mass instantaneous moment of inertia w.r.t. beta joint
    T w_z_i = 1.0/4.0*cfg.w_m*pow(cfg.w_r, 2);

    T wheel_speed = x[BETA_D] + x[PITCH_D]; // wheel_d = beta_d + pitch_d
    T wheel_acceleration = u[1] / w_b_i;
    x_d[PITCH_D] = -u[1] / (l1_i + l2_b_i + p_b_i) + cfg.l1_l/2.0*sin(x[PITCH])*g/l1_i; // reactive torque + torque from gravity
    x_d[BETA_D] = wheel_acceleration - x_d[PITCH_D]; // beta_dd = wheel_dd - pitch_dd
    x_d[ALPHA_D] = u[0] / l2_a_i;
    T acceleration = cfg.w_r * wheel_acceleration;
    // Using derivative of the equation of precession: http://hyperphysics.phy-astr.gsu.edu/hbase/top.html
    x_d[YAW_D] = -cfg.p_m*g*(cfg.l2_l*sin(x[ALPHA])) / (sin(x[ROLL])*w_b_i*pow(wheel_speed, 2)) * wheel_acceleration;
    // Taken from https://arxiv.org/pdf/1007.5288.pdf
    x_d[ROLL_D] = 2.0*w_z_i*x[YAW_D]*x_d[YAW_D]/(cfg.w_m*g*cfg.w_r);
    T roll_acceleration = cfg.w_r*x_d[ROLL_D]; // linear acceleration caused by roll
    x_d[X_D] = acceleration * cos(x[YAW]) + roll_acceleration*cos(x[ROLL])*sin(x[YAW]);
    x_d[Y_D] = acceleration * sin(x[YAW]) + roll_acceleration*cos(x[ROLL])*cos(x[YAW]);
    x_d[Z_D] = roll_acceleration*sin(x[ROLL]);

    derivatives->SetFromVector(x_d);
}

template <typename T>
void UnibotPlant<T>::SetDefaultState(const systems::Context<T>&, systems::State<T>* state) const
{
    systems::VectorBase<T>& mutable_state = state->get_mutable_continuous_state().get_mutable_vector();
    mutable_state.SetFromVector(Eigen::Matrix<T, UnibotStateIndex::NUM_STATES, 1>::Zero());
}

template <typename T>
void UnibotPlant<T>::copyStateOut(const systems::Context<T> &context,
        systems::BasicVector<T> *output) const
{
    output->set_value(context.get_continuous_state_vector().CopyToVector());
}

template <typename T>
void UnibotPlant<T>::copyPoseOut(const systems::Context<T> &context,
        systems::BasicVector<T> *output) const
{
    using namespace UnibotPoseIndex;
    const VectorX<T> x = context.get_continuous_state_vector().CopyToVector();
    Eigen::Matrix<T, NUM_POSES, 1> pose;
    Eigen::Quaternion<T> q;
    q = Eigen::AngleAxis<T>(x[UnibotStateIndex::ROLL], Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxis<T>(x[UnibotStateIndex::PITCH], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxis<T>(x[UnibotStateIndex::YAW], Eigen::Vector3d::UnitZ());
    pose[Q_W] = q.w();
    pose[Q_X] = q.x();
    pose[Q_Y] = q.y();
    pose[Q_Z] = q.z();
    pose[X] = x[UnibotStateIndex::X];
    pose[Y] = x[UnibotStateIndex::Y];
    pose[Z] = x[UnibotStateIndex::Z];
    pose[ALPHA] = x[UnibotStateIndex::ALPHA];
    pose[BETA] = x[UnibotStateIndex::BETA];
    output->set_value(pose);
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class UnibotPlant)
