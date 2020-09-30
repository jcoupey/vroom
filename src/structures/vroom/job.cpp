/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <numeric>

#include "structures/vroom/job.h"
#include "utils/helpers.h"

namespace vroom {

inline Duration get_tw_length(const std::vector<TimeWindow>& tws) {
  return std::accumulate(std::next(tws.begin()),
                         tws.end(),
                         tws[0].length,
                         [](auto sum, auto tw) { return sum + tw.length; });
}

Job::Job(Id id,
         const Location& location,
         Duration service,
         const Amount& delivery,
         const Amount& pickup,
         const Skills& skills,
         Priorities priorities,
         const Priority max_priority,
         const std::vector<TimeWindow>& tws,
         const std::string& description)
  : location(location),
    id(id),
    type(JOB_TYPE::SINGLE),
    service(service),
    delivery(delivery),
    pickup(pickup),
    skills(skills),
    priorities(priorities),
    max_priority(max_priority),
    tws(tws),
    description(description),
    tw_length(get_tw_length(tws)) {
  utils::check_tws(tws);
}

Job::Job(Id id,
         JOB_TYPE type,
         const Location& location,
         Duration service,
         const Amount& amount,
         const Skills& skills,
         Priorities priorities,
         const Priority max_priority,
         const std::vector<TimeWindow>& tws,
         const std::string& description)
  : location(location),
    id(id),
    type(type),
    service(service),
    delivery((type == JOB_TYPE::DELIVERY) ? amount : Amount(amount.size())),
    pickup((type == JOB_TYPE::PICKUP) ? amount : Amount(amount.size())),
    skills(skills),
    priorities(priorities),
    max_priority(max_priority),
    tws(tws),
    description(description),
    tw_length(get_tw_length(tws)) {
  assert(type == JOB_TYPE::PICKUP or type == JOB_TYPE::DELIVERY);
  utils::check_tws(tws);
}

bool Job::is_valid_start(Duration time) const {
  bool valid = false;

  for (const auto& tw : tws) {
    if (tw.contains(time)) {
      valid = true;
      break;
    }
  }

  return valid;
}

} // namespace vroom
