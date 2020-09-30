/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/intra_cross_exchange.h"

namespace vroom {
namespace vrptw {

IntraCrossExchange::IntraCrossExchange(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       TWRoute& tw_s_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank,
                                       bool check_s_reverse,
                                       bool check_t_reverse)
  : cvrp::IntraCrossExchange(input,
                             sol_state,
                             static_cast<RawRoute&>(tw_s_route),
                             s_vehicle,
                             s_rank,
                             t_rank,
                             check_s_reverse,
                             check_t_reverse),
    _tw_s_route(tw_s_route) {
}

bool IntraCrossExchange::is_valid() {
  bool valid = cvrp::IntraCrossExchange::is_valid();

  if (valid) {
    s_normal_t_normal_is_valid =
      s_normal_t_normal_is_valid &&
      _tw_s_route.is_valid_addition_for_tw(_input,
                                           _moved_jobs.begin(),
                                           _moved_jobs.end(),
                                           _first_rank,
                                           _last_rank);

    std::swap(_moved_jobs[0], _moved_jobs[1]);

    if (check_t_reverse) {
      s_normal_t_reverse_is_valid =
        s_normal_t_reverse_is_valid &&
        _tw_s_route.is_valid_addition_for_tw(_input,
                                             _moved_jobs.begin(),
                                             _moved_jobs.end(),
                                             _first_rank,
                                             _last_rank);
    }

    std::swap(_moved_jobs[_moved_jobs.size() - 2],
              _moved_jobs[_moved_jobs.size() - 1]);

    if (check_s_reverse and check_t_reverse) {
      s_reverse_t_reverse_is_valid =
        s_reverse_t_reverse_is_valid &&
        _tw_s_route.is_valid_addition_for_tw(_input,
                                             _moved_jobs.begin(),
                                             _moved_jobs.end(),
                                             _first_rank,
                                             _last_rank);
    }

    std::swap(_moved_jobs[0], _moved_jobs[1]);

    if (check_s_reverse) {
      s_reverse_t_normal_is_valid =
        s_reverse_t_normal_is_valid &&
        _tw_s_route.is_valid_addition_for_tw(_input,
                                             _moved_jobs.begin(),
                                             _moved_jobs.end(),
                                             _first_rank,
                                             _last_rank);
    }

    // Reset to initial situation before potential application.
    std::swap(_moved_jobs[_moved_jobs.size() - 2],
              _moved_jobs[_moved_jobs.size() - 1]);

    valid = s_normal_t_normal_is_valid or s_normal_t_reverse_is_valid or
            s_reverse_t_reverse_is_valid or s_reverse_t_normal_is_valid;
  }

  return valid;
}

void IntraCrossExchange::apply() {
  assert(!reverse_s_edge or
         (_input.jobs[s_route[s_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[s_route[s_rank + 1]].type == JOB_TYPE::SINGLE));
  assert(!reverse_t_edge or
         (_input.jobs[t_route[t_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[t_route[t_rank + 1]].type == JOB_TYPE::SINGLE));

  if (reverse_t_edge) {
    std::swap(_moved_jobs[0], _moved_jobs[1]);
  }
  if (reverse_s_edge) {
    std::swap(_moved_jobs[_moved_jobs.size() - 2],
              _moved_jobs[_moved_jobs.size() - 1]);
  }

  _tw_s_route.replace(_input,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);
}

std::vector<Index> IntraCrossExchange::addition_candidates() const {
  return {s_vehicle};
}

} // namespace vrptw
} // namespace vroom
