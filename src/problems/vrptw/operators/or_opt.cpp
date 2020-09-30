/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/or_opt.h"

namespace vroom {
namespace vrptw {

OrOpt::OrOpt(const Input& input,
             const utils::SolutionState& sol_state,
             TWRoute& tw_s_route,
             Index s_vehicle,
             Index s_rank,
             TWRoute& tw_t_route,
             Index t_vehicle,
             Index t_rank)
  : cvrp::OrOpt(input,
                sol_state,
                static_cast<RawRoute&>(tw_s_route),
                s_vehicle,
                s_rank,
                static_cast<RawRoute&>(tw_t_route),
                t_vehicle,
                t_rank),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

bool OrOpt::is_valid() {
  bool valid =
    cvrp::OrOpt::is_valid() and _tw_s_route.is_valid_removal(_input, s_rank, 2);

  if (valid) {
    // Keep edge direction.
    auto s_start = s_route.begin() + s_rank;
    is_normal_valid =
      is_normal_valid && _tw_t_route.is_valid_addition_for_tw(_input,
                                                              s_start,
                                                              s_start + 2,
                                                              t_rank,
                                                              t_rank);
    // Reverse edge direction.
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    is_reverse_valid = is_reverse_valid &&
                       _tw_t_route.is_valid_addition_for_tw(_input,
                                                            s_reverse_start,
                                                            s_reverse_start + 2,
                                                            t_rank,
                                                            t_rank);

    valid = is_normal_valid or is_reverse_valid;
  }

  return valid;
}

void OrOpt::apply() {
  if (reverse_s_edge) {
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    _tw_t_route.replace(_input,
                        s_reverse_start,
                        s_reverse_start + 2,
                        t_rank,
                        t_rank);
    _tw_s_route.remove(_input, s_rank, 2);
  } else {
    auto s_start = s_route.begin() + s_rank;
    _tw_t_route.replace(_input, s_start, s_start + 2, t_rank, t_rank);
    _tw_s_route.remove(_input, s_rank, 2);
  }
}

} // namespace vrptw
} // namespace vroom
