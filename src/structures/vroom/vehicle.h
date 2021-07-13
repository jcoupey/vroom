#ifndef VEHICLE_H
#define VEHICLE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>
#include <unordered_map>

#include "structures/typedefs.h"
#include "structures/vroom/amount.h"
#include "structures/vroom/break.h"
#include "structures/vroom/cost_wrapper.h"
#include "structures/vroom/input/vehicle_step.h"
#include "structures/vroom/location.h"
#include "structures/vroom/time_window.h"

namespace vroom {

struct Vehicle {
  const Id id;
  std::optional<Location> start;
  std::optional<Location> end;
  const std::string profile;
  const Amount capacity;
  const Skills skills;
  const TimeWindow tw;
  const std::vector<Break> breaks;
  const std::string description;
  CostWrapper cost_wrapper;
  const size_t max_number_of_tasks;
  std::vector<VehicleStep> steps;
  std::unordered_map<Id, Index> break_id_to_rank;

  Vehicle(
    Id id,
    const std::optional<Location>& start,
    const std::optional<Location>& end,
    const std::string& profile = DEFAULT_PROFILE,
    const Amount& capacity = Amount(0),
    const Skills& skills = Skills(),
    const TimeWindow& tw = TimeWindow(),
    const std::vector<Break>& breaks = std::vector<Break>(),
    const std::string& description = "",
    double speed_factor = 1.,
    const size_t max_number_of_tasks = std::numeric_limits<size_t>::max(),
    const std::vector<VehicleStep>& input_steps = std::vector<VehicleStep>());

  bool has_start() const;

  bool has_end() const;

  bool has_same_locations(const Vehicle& other) const;

  bool has_same_profile(const Vehicle& other) const;

  Cost duration(Index i, Index j) const {
    return cost_wrapper.cost(i, j);
  }

  Cost cost(Index i, Index j) const {
    return cost_wrapper.cost(i, j);
  }
};

} // namespace vroom

#endif
