/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/exchange.h"

namespace vroom {
namespace vrptw {

Exchange::Exchange(const Input& input,
                   const utils::SolutionState& sol_state,
                   TWRoute& tw_s_route,
                   Index s_vehicle,
                   Index s_rank,
                   TWRoute& tw_t_route,
                   Index t_vehicle,
                   Index t_rank)
  : cvrp::Exchange(input,
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

bool Exchange::is_valid() {
  bool valid = cvrp::Exchange::is_valid();
  valid =
    valid && _tw_t_route.is_valid_addition_for_tw(_input,
                                                  s_route.begin() + s_rank,
                                                  s_route.begin() + s_rank + 1,
                                                  t_rank,
                                                  t_rank + 1);
  valid =
    valid && _tw_s_route.is_valid_addition_for_tw(_input,
                                                  t_route.begin() + t_rank,
                                                  t_route.begin() + t_rank + 1,
                                                  s_rank,
                                                  s_rank + 1);
  return valid;
}

void Exchange::apply() {
  std::vector<Index> t_job_ranks(1, t_route[t_rank]);

  _tw_t_route.replace(_input,
                      s_route.begin() + s_rank,
                      s_route.begin() + s_rank + 1,
                      t_rank,
                      t_rank + 1);
  _tw_s_route.replace(_input,
                      t_job_ranks.begin(),
                      t_job_ranks.end(),
                      s_rank,
                      s_rank + 1);
}

} // namespace vrptw
} // namespace vroom
