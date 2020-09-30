#ifndef SOLUTION_H
#define SOLUTION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

#include "structures/vroom/job.h"
#include "structures/vroom/solution/route.h"
#include "structures/vroom/solution/summary.h"

namespace vroom {

struct Solution {
  unsigned code;
  std::string error;
  Summary summary;
  std::vector<Route> routes;
  std::vector<Job> unassigned;

  Solution(unsigned code, std::string error);

  Solution(unsigned code,
           unsigned amount_size,
           std::vector<Route>&& routes,
           std::vector<Job>&& unassigned);
};

} // namespace vroom

#endif
