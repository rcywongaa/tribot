#include "drake/systems/controllers/linear_quadratic_regulator.h"

std::unique_ptr<drake::systems::AffineSystem<double>> MakeCartPoleLQRController(const std::string full_model_filename);
