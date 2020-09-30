/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "problems/vrptw/operators/reverse_two_opt.h"

namespace vroom {
namespace vrptw {

ReverseTwoOpt::ReverseTwoOpt(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             Index s_rank,
                             TWRoute& tw_t_route,
                             Index t_vehicle,
                             Index t_rank)
  : cvrp::ReverseTwoOpt(input,
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

bool ReverseTwoOpt::is_valid() {
  return cvrp::ReverseTwoOpt::is_valid() and
         _tw_t_route.is_valid_addition_for_tw(_input,
                                              s_route.rbegin(),
                                              s_route.rbegin() +
                                                s_route.size() - 1 - s_rank,
                                              0,
                                              t_rank + 1) and
         _tw_s_route.is_valid_addition_for_tw(_input,
                                              t_route.rbegin() +
                                                t_route.size() - 1 - t_rank,
                                              t_route.rend(),
                                              s_rank + 1,
                                              s_route.size());
}

void ReverseTwoOpt::apply() {
  std::vector<Index> t_job_ranks;
  t_job_ranks.insert(t_job_ranks.begin(),
                     t_route.rbegin() + t_route.size() - 1 - t_rank,
                     t_route.rend());

  _tw_t_route.replace(_input,
                      s_route.rbegin(),
                      s_route.rbegin() + s_route.size() - 1 - s_rank,
                      0,
                      t_rank + 1);

  _tw_s_route.replace(_input,
                      t_job_ranks.begin(),
                      t_job_ranks.end(),
                      s_rank + 1,
                      s_route.size());
}

} // namespace vrptw
} // namespace vroom
