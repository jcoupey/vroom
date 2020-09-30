/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/unassigned_exchange.h"

namespace vroom {
namespace vrptw {

UnassignedExchange::UnassignedExchange(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       std::unordered_set<Index>& unassigned,
                                       TWRoute& tw_s_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank,
                                       Index u)
  : cvrp::UnassignedExchange(input,
                             sol_state,
                             unassigned,
                             static_cast<RawRoute&>(tw_s_route),
                             s_vehicle,
                             s_rank,
                             t_rank,
                             u),
    _tw_s_route(tw_s_route) {
}

bool UnassignedExchange::is_valid() {
  return cvrp::UnassignedExchange::is_valid() and
         _tw_s_route.is_valid_addition_for_tw(_input,
                                              _moved_jobs.begin(),
                                              _moved_jobs.end(),
                                              _first_rank,
                                              _last_rank);
}

void UnassignedExchange::apply() {
  _tw_s_route.replace(_input,
                      _moved_jobs.begin(),
                      _moved_jobs.end(),
                      _first_rank,
                      _last_rank);

  assert(_unassigned.find(_u) != _unassigned.end());
  _unassigned.erase(_u);
  assert(_unassigned.find(_removed) == _unassigned.end());
  _unassigned.insert(_removed);
}

} // namespace vrptw
} // namespace vroom
