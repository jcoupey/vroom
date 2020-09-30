/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom {
namespace ls {

Gain Operator::gain() {
  if (!gain_computed) {
    this->compute_gain();
  }
  return stored_gain;
}

std::vector<Index> Operator::required_unassigned() const {
  return std::vector<Index>();
}

} // namespace ls
} // namespace vroom
