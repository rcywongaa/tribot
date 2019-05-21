#pragma once

#include <memory>
#include <string>

#include "drake/systems/primitives/affine_system.h"

#include "meta.hpp"

std::unique_ptr<drake::systems::AffineSystem<double>> MakeAcrobotLQRController(std::string filename);
