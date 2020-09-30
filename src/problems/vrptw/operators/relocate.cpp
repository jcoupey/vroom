/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/relocate.h"

namespace vroom {
namespace vrptw {

Relocate::Relocate(const Input& input,
                   const utils::SolutionState& sol_state,
                   TWRoute& tw_s_route,
                   Index s_vehicle,
                   Index s_rank,
                   TWRoute& tw_t_route,
                   Index t_vehicle,
                   Index t_rank)
  : cvrp::Relocate(input,
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

bool Relocate::is_valid() {
  return cvrp::Relocate::is_valid() and
         _tw_t_route.is_valid_addition_for_tw(_input,
                                              s_route[s_rank],
                                              t_rank) and
         _tw_s_route.is_valid_removal(_input, s_rank, 1);
}

void Relocate::apply() {
  auto relocate_job_rank = s_route[s_rank];

  _tw_s_route.remove(_input, s_rank, 1);
  _tw_t_route.add(_input, relocate_job_rank, t_rank);
}

} // namespace vrptw
} // namespace vroom
