#ifndef COMPUTING_TIMES_H
#define COMPUTING_TIMES_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"

namespace vroom {

struct ComputingTimes {
  // Computing times in milliseconds.
  Duration loading;
  Duration solving;
  Duration routing;

  ComputingTimes();
};

} // namespace vroom

#endif
