//
// Created by Brian Jackson on 9/12/22.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#pragma once

#include <ArduinoEigenDense.h>
#include "typedefs.hpp"

namespace altro {

using Vector = Eigen::Vector<a_float, Eigen::Dynamic>;
using Matrix = Eigen::Matrix<a_float, Eigen::Dynamic, Eigen::Dynamic>;
using Eigen::VectorXi;

}  // namespace altro
