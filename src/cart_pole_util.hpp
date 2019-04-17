#include <string>

#include "drake/systems/controllers/linear_quadratic_regulator.h"

/**
 * @brief Constructs an LQR controller for a cart pole using the given cart pole model file
 * @param cart_pole_filename Full path of the .sdf file representing the cart pole
 * @param slider_joint_name Name of the prismatic joint representing the cart slider in the .sdf file
 * @param pin_joint_name Name of the revolute joint representing the pole pin in the .sdf file
 * @param target_translation
 * @param target_translation_rate
 * @param target_angle The target angle where the LQR incurs zero cost.  This should be at a fixed point.
 * @param target_angular_rate
 *
 * Default parameters are based on the cart pole specified in
 * drake/examples/multibody/cart_pole/cart_pole.sdf
 */
std::unique_ptr<drake::systems::AffineSystem<double>> MakeCartPoleLQRController(
        const std::string cart_pole_filename,
        const std::string slider_joint_name = "CartSlider",
        const std::string pin_joint_name = "PolePin",
        const double target_translation = 0.0,
        const double target_translation_rate = 0.0,
        const double target_angle = M_PI,
        const double target_angular_rate = 0.0);
