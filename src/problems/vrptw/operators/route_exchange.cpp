/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/operators/route_exchange.h"

namespace vroom {
namespace vrptw {

RouteExchange::RouteExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             TWRoute& tw_s_route,
                             Index s_vehicle,
                             TWRoute& tw_t_route,
                             Index t_vehicle)
  : cvrp::RouteExchange(input,
                        sol_state,
                        static_cast<RawRoute&>(tw_s_route),
                        s_vehicle,
                        static_cast<RawRoute&>(tw_t_route),
                        t_vehicle),
    _tw_s_route(tw_s_route),
    _tw_t_route(tw_t_route) {
}

bool RouteExchange::is_valid() {
  bool valid = cvrp::RouteExchange::is_valid();
  valid = valid && _tw_t_route.is_valid_addition_for_tw(_input,
                                                        s_route.begin(),
                                                        s_route.end(),
                                                        0,
                                                        t_route.size());
  valid = valid && _tw_s_route.is_valid_addition_for_tw(_input,
                                                        t_route.begin(),
                                                        t_route.end(),
                                                        0,
                                                        s_route.size());
  return valid;
}

void RouteExchange::apply() {
  std::vector<Index> t_job_ranks(t_route);

  if (s_route.empty()) {
    _tw_t_route.remove(_input, 0, t_route.size());
  } else {
    _tw_t_route.replace(_input,
                        s_route.begin(),
                        s_route.end(),
                        0,
                        t_route.size());
  }

  if (t_job_ranks.empty()) {
    _tw_s_route.remove(_input, 0, s_route.size());
  } else {
    _tw_s_route.replace(_input,
                        t_job_ranks.begin(),
                        t_job_ranks.end(),
                        0,
                        s_route.size());
  }
}

} // namespace vrptw
} // namespace vroom
