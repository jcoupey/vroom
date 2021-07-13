/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/vehicle.h"
#include "utils/exception.h"

namespace vroom {

Vehicle::Vehicle(Id id,
                 const std::optional<Location>& start,
                 const std::optional<Location>& end,
                 const std::string& profile,
                 const Amount& capacity,
                 const Skills& skills,
                 const TimeWindow& tw,
                 const std::vector<Break>& breaks,
                 const std::string& description,
                 double speed_factor,
                 const size_t max_number_of_tasks,
                 const std::vector<VehicleStep>& input_steps)
  : id(id),
    start(start),
    end(end),
    profile(profile),
    capacity(capacity),
    skills(skills),
    tw(tw),
    breaks(breaks),
    description(description),
    cost_wrapper(speed_factor),
    max_number_of_tasks(max_number_of_tasks) {
  if (!static_cast<bool>(start) and !static_cast<bool>(end)) {
    throw Exception(ERROR::INPUT,
                    "No start or end specified for vehicle " +
                      std::to_string(id) + '.');
  }

  for (unsigned i = 0; i < breaks.size(); ++i) {
    const auto& b = breaks[i];
    if (break_id_to_rank.find(b.id) != break_id_to_rank.end()) {
      throw Exception(ERROR::INPUT,
                      "Duplicate break id: " + std::to_string(b.id) + ".");
    }
    break_id_to_rank[b.id] = i;
  }

  if (!input_steps.empty()) {
    // Populating steps. We rely on always having start and end steps
    // in input, so just add them if they're missing.
    unsigned rank_after_start = 0;
    if (input_steps.front().type == STEP_TYPE::START) {
      steps.push_back(input_steps.front());
      rank_after_start = 1;
    } else {
      steps.emplace_back(STEP_TYPE::START);
    }

    for (unsigned i = rank_after_start; i < input_steps.size(); ++i) {
      if (input_steps[i].type == STEP_TYPE::START) {
        throw Exception(ERROR::INPUT,
                        "Unexpected start in input steps for vehicle " +
                          std::to_string(id) + ".");
      }
      if (input_steps[i].type == STEP_TYPE::END and
          (i != input_steps.size() - 1)) {
        throw Exception(ERROR::INPUT,
                        "Unexpected end in input steps for vehicle " +
                          std::to_string(id) + ".");
      }

      steps.push_back(input_steps[i]);
    }

    if (steps.back().type != STEP_TYPE::END) {
      steps.emplace_back(STEP_TYPE::END);
    }
  }
}

bool Vehicle::has_start() const {
  return static_cast<bool>(start);
}

bool Vehicle::has_end() const {
  return static_cast<bool>(end);
}

bool Vehicle::has_same_locations(const Vehicle& other) const {
  bool same = (this->has_start() == other.has_start()) and
              (this->has_end() == other.has_end());

  if (same and this->has_start()) {
    same = this->start.value() == other.start.value();
  }

  if (same and this->has_end()) {
    same = this->end.value() == other.end.value();
  }

  return same;
}

bool Vehicle::has_same_profile(const Vehicle& other) const {
  return (this->profile == other.profile) and
         (this->cost_wrapper.discrete_factor ==
          other.cost_wrapper.discrete_factor);
}

} // namespace vroom
