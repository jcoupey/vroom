/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

#include "structures/vroom/time_window.h"
#include "utils/exception.h"

namespace vroom {

constexpr Duration TimeWindow::default_length =
  std::numeric_limits<Duration>::max();

TimeWindow::TimeWindow()
  : start(0), end(std::numeric_limits<Duration>::max()), length(end - start) {
}

TimeWindow::TimeWindow(Duration start, Duration end)
  : start(start), end(end), length(end - start) {
  if (start > end) {
    throw Exception(ERROR::INPUT,
                    "Invalid time window: [" + std::to_string(start) + ", " +
                      std::to_string(end) + "]");
  }
}

bool TimeWindow::contains(Duration time) const {
  return (start <= time) and (time <= end);
}

bool TimeWindow::is_default() const {
  return end - start == default_length;
}

bool operator<(const TimeWindow& lhs, const TimeWindow& rhs) {
  return lhs.start < rhs.start or
         (lhs.start == rhs.start and lhs.end < rhs.end);
}

} // namespace vroom
