#ifndef VRPTW_H
#define VRPTW_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "problems/vrp.h"

namespace vroom {

class VRPTW : public VRP {
private:
  static const std::vector<HeuristicParameters> homogeneous_parameters;
  static const std::vector<HeuristicParameters> heterogeneous_parameters;

public:
  VRPTW(const Input& input);

  virtual Solution
  solve(unsigned exploration_level,
        unsigned nb_threads,
        const std::vector<HeuristicParameters>& h_param) const override;
};

} // namespace vroom

#endif
