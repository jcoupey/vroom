#ifndef TIME_WINDOW_H
#define TIME_WINDOW_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"

namespace vroom {

struct TimeWindow {
  static const Duration default_length;
  Duration start;
  Duration end;
  Duration length;

  // Default "no-constraint" time-window.
  TimeWindow();

  TimeWindow(Duration start, Duration end);

  bool contains(Duration time) const;

  bool is_default() const;

  friend bool operator<(const TimeWindow& lhs, const TimeWindow& rhs);
};

} // namespace vroom

#endif
