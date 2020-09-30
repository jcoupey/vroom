#ifndef VRP_H
#define VRP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution/solution.h"

namespace vroom {

class Input;

class VRP {
  // Abstract class describing a VRP (vehicle routing problem).
protected:
  const Input& _input;

public:
  VRP(const Input& input);

  virtual ~VRP();

  virtual Solution
  solve(unsigned exploration_level,
        unsigned nb_threads,
        const std::vector<HeuristicParameters>& h_param) const = 0;
};

} // namespace vroom

#endif
