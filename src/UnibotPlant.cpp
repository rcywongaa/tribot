#include "UnibotPlant.hpp"

#include <cmath>
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/wrap_to.h"

using namespace drake;

const double g = 9.81;

/*
 * @brief Solves the simultaneous equation of the form
 * A*x + B*y + C = 0
 * D*x + E*y + F = 0
 */
template <typename T>
std::pair<T, T> solve_simultaneous_equation(T A, T B, T C, T D, T E, T F)
{
    T x = (-F+E*C/B) / (D - E*A/B);
    T y = (-C-A*x)/B;
    return std::make_pair(x, y);
};

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
    using std::abs;

    {
        // MIP Equation of Motion : http://renaissance.ucsd.edu/courses/mae143c/MIPdynamics.pdf
        T m_w = cfg.w_m;
        T m_r = cfg.l1_m + cfg.l2_m + cfg.p_m; // total mass of rod
        T I_w = 1.0/2.0*cfg.w_m*pow(cfg.w_r, 2); // inertia of wheel

        // Moment of inertia of link1
        T I_l1 = 1.0/3.0*cfg.l1_m*pow(cfg.l1_l,2);

        // l2 center of mass to l1 base distance
        T l2_cm_l1_x = 0.5*cfg.l2_l*sin(x[ALPHA]);
        T l2_cm_l1_y = cfg.l1_l - 0.5*cfg.l2_l*cos(x[ALPHA]);
        T l2_cm_l1 = pow(pow(l2_cm_l1_x,2) + pow(l2_cm_l1_y,2), 0.5);
        // Moment of inertia of link2
        T I_l2 = 1.0/12*cfg.l2_m*pow(cfg.l2_l,2) + cfg.l2_m*pow(l2_cm_l1,2);

        // point mass center of mass to l1 base distance
        T p_cm_l1_x = cfg.l2_l*sin(x[ALPHA]);
        T p_cm_l1_y = cfg.l1_l - cfg.l2_l*cos(x[ALPHA]);
        T p_cm_l1 = pow(pow(p_cm_l1_x,2) + pow(p_cm_l1_y,2), 0.5);
        // Moment of inertia of point mass
        T I_p = cfg.p_m*pow(p_cm_l1,2);
        T I_r = I_l1 + I_l2 + I_p; // inertia of rod

        T tau = u[1];
        T R = cfg.w_r;
        T L = (cfg.l1_m*0.5*cfg.l1_l + cfg.l2_m*l2_cm_l1_y + cfg.p_m*p_cm_l1_y) / (cfg.l1_m + cfg.l2_m + cfg.p_m); // length from wheel center to rod center of mass
        T L_T = cfg.l1_l; // length of rod
        /*
         * +x = right
         * +y = up
         * +z = out of paper
         */
        T theta = x[PITCH]; // rod angle from vertical, +z direction
        T phi = x[PITCH] + x[BETA]; // wheel angle from horizontal, +z direction
        T theta_d = x[PITCH_D];
        T phi_d = x[PITCH_D] + x[BETA_D];
        T theta_dd;
        T phi_dd;

        T A = m_r*R*L*cos(theta);
        T B = I_r + m_r*pow(L,2);
        T C = -(m_r*g*L*sin(theta) - tau);
        T D = I_w + (m_r + m_w)*pow(R, 2);
        T E = m_r*R*L*cos(theta);
        T F = -(m_r*R*L*pow(theta,2)*sin(theta) + tau);
        std::tie(phi_dd, theta_dd) = solve_simultaneous_equation(A, B, C, D, E, F);

        x_d[PITCH_D] = theta_dd;
        x_d[BETA_D] = phi_dd - theta_dd;
    }
    {
        // Acrobot Equation of Motion : http://underactuated.mit.edu/underactuated.html?chapter=acrobot
        // The above equation use x[ROLL] = 0 as pointing horizontally downward
        T q1 = math::wrap_to(x[ROLL] + M_PI, -M_PI, M_PI);
        T q2 = math::wrap_to(x[ALPHA] + M_PI, -M_PI, M_PI);
        T q1_d = x[ROLL_D];
        T q2_d = x[ALPHA_D];
        T tau = u[0];
        T m1 = cfg.l1_m + cfg.w_m;
        T m2 = cfg.l2_m + cfg.p_m;
        T l1 = cfg.l1_l + cfg.w_r;
        T lc1 = (cfg.l1_m*(cfg.w_r + cfg.l1_l/2.0) + cfg.w_m*cfg.w_r) / (cfg.l1_m + cfg.w_m);
        // Moment of inertia taken about pivot
        T I1 = 1.0/12.0*cfg.l1_m*pow(cfg.l1_l,2) + cfg.l1_m*pow(cfg.l1_l/2.0 + cfg.w_r, 2) // moment of inertia of link1 w.r.t pivot (ground)
            + 1.0/12.0*cfg.w_m*(3.0*pow(cfg.w_r, 2) + pow(cfg.w_t, 2)) + cfg.w_m*pow(cfg.w_r, 2); // moment of inertia of wheels w.r.t. pivot (ground)
        T l2 = cfg.l2_l;
        T lc2 = (cfg.l2_m*(cfg.l2_l/2.0) + cfg.p_m*cfg.l2_l) / (cfg.l2_m + cfg.p_m);
        // Moment of inertia taken about pivot
        T I2 = 1.0/3.0*cfg.l2_m*pow(cfg.l2_l,2) + cfg.p_m*pow(cfg.l2_l, 2);
        T c1 = cos(q1);
        T c2 = cos(q2);
        T c12 = cos(math::wrap_to(q1 + q2, -M_PI, M_PI));
        T s1 = sin(q1);
        T s2 = sin(q2);
        T s12 = sin(math::wrap_to(q1 + q2, -M_PI, M_PI));
        // Equation (5) : A * q1_dd + B * q2_dd + C = 0
        T A = I1 + I2 + m2*pow(l1,2) + 2*m2*l1*lc2*c2;
        T B = I2 + m2*l1*lc2*c2;
        T C = -2.0*m2*l1*lc2*s2*q1_d*q2_d - m2*l1*lc2*s2*pow(q2_d,2) + m1*g*lc1*s1 + m2*g*(l1*s1 + lc2*s12);
        // Equation (6) : D * q1_dd + E * q2_dd + F = 0
        T D = I2 + m2*l1*lc2*c2;
        T E = I2;
        T F = (m2*l1*lc2*s2*pow(q1_d,2) + m2*g*lc2*s12) - tau;

        std::tie(x_d[ROLL_D], x_d[ALPHA_D]) = solve_simultaneous_equation(A, B, C, D, E, F);
    }

    T wheel_acceleration = x_d[BETA_D] + x_d[PITCH_D];
    T wheel_speed = x[BETA_D] + x[PITCH_D]; // wheel_d = beta_d + pitch_d
    //printf("wheel_speed = %f\n", wheel_speed);
    T acceleration = cfg.w_r * wheel_acceleration;
    //printf("acceleration = %f\n", acceleration);

    if (wheel_speed > 0.01)
    {
        // Using derivative of the equation of precession: http://hyperphysics.phy-astr.gsu.edu/hbase/top.html
        // Assuming zero roll for now...
        //x_d[YAW_D] = cfg.p_m*g*cfg.l2_l*cos(x[ALPHA])*x[ALPHA_D]/(w_b_i * wheel_speed) +
            //cfg.p_m*g*cfg.l2_l*sin(x[ALPHA]) / (w_b_i*pow(wheel_speed, 2)) * wheel_acceleration;
        //x_d[ROLL_D] = 2.0*w_z_i*x[YAW_D]*x_d[YAW_D]/(cfg.w_m*g*cfg.w_r);
        ;
    }
    else
    {
        x_d[YAW_D] = 0.0;
    }
    // Taken from https://arxiv.org/pdf/1007.5288.pdf
    T roll_acceleration = cfg.w_r*x_d[ROLL_D]; // linear acceleration caused by roll
    //printf("roll_acceleration = %f\n", roll_acceleration);
    x_d[X_D] = acceleration * cos(x[YAW]) + roll_acceleration*cos(x[ROLL])*sin(x[YAW]);
    x_d[Y_D] = acceleration * sin(x[YAW]) - roll_acceleration*cos(x[ROLL])*cos(x[YAW]);
    x_d[Z_D] = -roll_acceleration*sin(x[ROLL]);

    printf("roll = %f\n", x[ROLL]);
    printf("pitch = %f\n", x[PITCH]);
    printf("yaw = %f\n", x[YAW]);
    printf("x = %f\n", x[X]);
    printf("y = %f\n", x[Y]);
    printf("z = %f\n", x[Z]);
    printf("alpha = %f\n", x[ALPHA]);
    printf("beta = %f\n", x[BETA]);
    printf("----------\n");
    printf("roll_d = %f\n", x[ROLL_D]);
    printf("pitch_d = %f\n", x[PITCH_D]);
    printf("yaw_d = %f\n", x[YAW_D]);
    printf("x_d = %f\n", x[X_D]);
    printf("y_d = %f\n", x[Y_D]);
    printf("z_d = %f\n", x[Z_D]);
    printf("alpha_d = %f\n", x[ALPHA_D]);
    printf("beta_d = %f\n", x[BETA_D]);
    printf("----------\n");
    printf("u_alpha = %f\n", u[0]);
    printf("u_beta = %f\n", u[1]);
    printf("----------\n");
    printf("roll_dd = %f\n", x_d[ROLL_D]);
    printf("pitch_dd = %f\n", x_d[PITCH_D]);
    printf("yaw_dd = %f\n", x_d[YAW_D]);
    printf("x_dd = %f\n", x_d[X_D]);
    printf("y_dd = %f\n", x_d[Y_D]);
    printf("z_dd = %f\n", x_d[Z_D]);
    printf("alpha_dd = %f\n", x_d[ALPHA_D]);
    printf("beta_dd = %f\n", x_d[BETA_D]);
    printf("==========\n");
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
    q = math::RollPitchYaw<T>(x[UnibotStateIndex::ROLL], x[UnibotStateIndex::PITCH], x[UnibotStateIndex::YAW]).ToQuaternion();
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
