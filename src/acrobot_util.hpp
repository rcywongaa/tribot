#include <memory>

#include "drake/systems/primitives/affine_system.h"

std::unique_ptr<drake::systems::AffineSystem<double>> MakeAcrobotLQRController();
