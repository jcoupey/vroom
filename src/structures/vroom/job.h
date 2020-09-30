#ifndef JOB_H
#define JOB_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>
#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/amount.h"
#include "structures/vroom/location.h"
#include "structures/vroom/time_window.h"

namespace vroom {

struct Job {
  Location location;
  const Id id;
  const JOB_TYPE type;
  const Duration service;
  const Amount delivery;
  const Amount pickup;
  const Skills skills;
  Priorities priorities;
  const Priority max_priority;
  const std::vector<TimeWindow> tws;
  const std::string description;
  const Duration tw_length;

  // Constructor for regular one-stop job (JOB_TYPE::SINGLE).
  Job(Id id,
      const Location& location,
      Duration service = 0,
      const Amount& delivery = Amount(0),
      const Amount& pickup = Amount(0),
      const Skills& skills = Skills(),
      Priorities priorities =  Priorities(),
      const Priority max_priority = Priority(0),
      const std::vector<TimeWindow>& tws =
        std::vector<TimeWindow>(1, TimeWindow()),
      const std::string& description = "");

  // Constructor for pickup and delivery jobs (JOB_TYPE::PICKUP or
  // JOB_TYPE::DELIVERY).
  Job(Id id,
      JOB_TYPE type,
      const Location& location,
      Duration service = 0,
      const Amount& amount = Amount(0),
      const Skills& skills = Skills(),
      Priorities priorities =  Priorities(),
      const Priority max_priority = Priority(0),
      const std::vector<TimeWindow>& tws =
        std::vector<TimeWindow>(1, TimeWindow()),
      const std::string& description = "");

  Index index() const {
    return location.index();
  }

  bool is_valid_start(Duration time) const;
};

} // namespace vroom

#endif
