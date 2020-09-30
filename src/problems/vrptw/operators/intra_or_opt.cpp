/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_or_opt.h"

namespace vroom {
namespace vrptw {

IntraOrOpt::IntraOrOpt(const Input& input,
                       const utils::SolutionState& sol_state,
                       TWRoute& tw_s_route,
                       Index s_vehicle,
                       Index s_rank,
                       Index t_rank,
                       bool check_reverse)
  : cvrp::IntraOrOpt(input,
                     sol_state,
                     static_cast<RawRoute&>(tw_s_route),
                     s_vehicle,
                     s_rank,
                     t_rank,
                     check_reverse),
    _tw_s_route(tw_s_route) {
}

bool IntraOrOpt::is_valid() {
  bool valid = cvrp::IntraOrOpt::is_valid();

  if (valid) {
    is_normal_valid = is_normal_valid &&
                      _tw_s_route.is_valid_addition_for_tw(_input,
                                                           _moved_jobs.begin(),
                                                           _moved_jobs.end(),
                                                           _first_rank,
                                                           _last_rank);

    if (check_reverse) {
      std::swap(_moved_jobs[_s_edge_first], _moved_jobs[_s_edge_last]);

      is_reverse_valid =
        is_reverse_valid &&
        _tw_s_route.is_valid_addition_for_tw(_input,
                                             _moved_jobs.begin(),
                                             _moved_jobs.end(),
                                             _first_rank,
                                             _last_rank);

      // Reset to initial situation before potential application.
      std::swap(_moved_jobs[_s_edge_first], _moved_jobs[_s_edge_last]);
    }

    valid = (is_normal_valid or is_reverse_valid);
  }

  return valid;
}

void IntraOrOpt::apply() {
  assert(!reverse_s_edge or
         (_input.jobs[s_route[s_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[s_route[s_rank + 1]].type == JOB_TYPE::SINGLE));

  if (reverse_s_edge) {
    std::swap(_moved_jobs[_s_edge_first], _moved_jobs[_s_edge_last]);
  }

  _tw_s_route.replace(_input,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);
}

std::vector<Index> IntraOrOpt::addition_candidates() const {
  return {s_vehicle};
}

} // namespace vrptw
} // namespace vroom
