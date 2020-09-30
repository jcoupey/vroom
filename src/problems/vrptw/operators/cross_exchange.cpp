/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/cross_exchange.h"

namespace vroom {
namespace vrptw {

CrossExchange::CrossExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             TWRoute& tw_t_route,
                             Index t_vehicle,
                             Index t_rank,
                             bool check_s_reverse,
                             bool check_t_reverse)
  : cvrp::CrossExchange(input,
                        sol_state,
                        static_cast<RawRoute&>(tw_s_route),
                        s_vehicle,
                        s_rank,
                        static_cast<RawRoute&>(tw_t_route),
                        t_vehicle,
                        t_rank,
                        check_s_reverse,
                        check_t_reverse),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

bool CrossExchange::is_valid() {
  bool valid = cvrp::CrossExchange::is_valid();

  if (valid) {
    // Keep target edge direction when inserting in source route.
    auto t_start = t_route.begin() + t_rank;
    s_is_normal_valid =
      s_is_normal_valid && _tw_s_route.is_valid_addition_for_tw(_input,
                                                                t_start,
                                                                t_start + 2,
                                                                s_rank,
                                                                s_rank + 2);

    if (check_t_reverse) {
      // Reverse target edge direction when inserting in source route.
      auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
      s_is_reverse_valid =
        s_is_reverse_valid &&
        _tw_s_route.is_valid_addition_for_tw(_input,
                                             t_reverse_start,
                                             t_reverse_start + 2,
                                             s_rank,
                                             s_rank + 2);
    }

    valid = s_is_normal_valid or s_is_reverse_valid;
  }

  if (valid) {
    // Keep source edge direction when inserting in target route.
    auto s_start = s_route.begin() + s_rank;
    t_is_normal_valid =
      t_is_normal_valid && _tw_t_route.is_valid_addition_for_tw(_input,
                                                                s_start,
                                                                s_start + 2,
                                                                t_rank,
                                                                t_rank + 2);

    if (check_s_reverse) {
      // Reverse source edge direction when inserting in target route.
      auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
      t_is_reverse_valid =
        t_is_reverse_valid &&
        _tw_t_route.is_valid_addition_for_tw(_input,
                                             s_reverse_start,
                                             s_reverse_start + 2,
                                             t_rank,
                                             t_rank + 2);
    }

    valid = t_is_normal_valid or t_is_reverse_valid;
  }

  return valid;
}

void CrossExchange::apply() {
  assert(!reverse_s_edge or
         (_input.jobs[s_route[s_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[s_route[s_rank + 1]].type == JOB_TYPE::SINGLE));
  assert(!reverse_t_edge or
         (_input.jobs[t_route[t_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[t_route[t_rank + 1]].type == JOB_TYPE::SINGLE));

  std::vector<Index> t_job_ranks;
  if (!reverse_t_edge) {
    auto t_start = t_route.begin() + t_rank;
    t_job_ranks.insert(t_job_ranks.begin(), t_start, t_start + 2);
  } else {
    auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
    t_job_ranks.insert(t_job_ranks.begin(),
                       t_reverse_start,
                       t_reverse_start + 2);
  }

  if (!reverse_s_edge) {
    _tw_t_route.replace(_input,
                        s_route.begin() + s_rank,
                        s_route.begin() + s_rank + 2,
                        t_rank,
                        t_rank + 2);
  } else {
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _tw_t_route.replace(_input,
                        s_reverse_start,
                        s_reverse_start + 2,
                        t_rank,
                        t_rank + 2);
  }

  _tw_s_route.replace(_input,
                      t_job_ranks.begin(),
                      t_job_ranks.end(),
                      s_rank,
                      s_rank + 2);
}

} // namespace vrptw
} // namespace vroom
