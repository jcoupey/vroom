/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/two_opt.h"

namespace vroom {
namespace cvrp {

TwoOpt::TwoOpt(const Input& input,
               const utils::SolutionState& sol_state,
               RawRoute& s_route,
               Index s_vehicle,
               Index s_rank,
               RawRoute& t_route,
               Index t_vehicle,
               Index t_rank)
  : Operator(input,
             sol_state,
             s_route,
             s_vehicle,
             s_rank,
             t_route,
             t_vehicle,
             t_rank) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 1);
  assert(t_route.size() >= 1);
  assert(s_rank < s_route.size());
  assert(t_rank < t_route.size());

  assert(_sol_state.bwd_skill_rank[s_vehicle][t_vehicle] <= s_rank + 1);
  assert(_sol_state.bwd_skill_rank[t_vehicle][s_vehicle] <= t_rank + 1);
}

void TwoOpt::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v_source = _input.vehicles[s_vehicle];
  const auto& v_target = _input.vehicles[t_vehicle];

  Index s_index = _input.jobs[s_route[s_rank]].index();
  Index t_index = _input.jobs[t_route[t_rank]].index();
  Index last_s = _input.jobs[s_route.back()].index();
  Index last_t = _input.jobs[t_route.back()].index();
  stored_gain = 0;
  Index new_last_s = last_t;
  Index new_last_t = last_s;

  // Cost of swapping route for vehicle s_vehicle after step
  // s_rank with route for vehicle t_vehicle after step
  // t_rank.

  // Basic costs in case we really swap jobs and not only the end of
  // the route. Otherwise remember that last job does not change.
  if (s_rank < s_route.size() - 1) {
    Index next_index = _input.jobs[s_route[s_rank + 1]].index();
    stored_gain += m[s_index][next_index];
    stored_gain -= m[t_index][next_index];
  } else {
    new_last_t = t_index;
  }
  if (t_rank < t_route.size() - 1) {
    Index next_index = _input.jobs[t_route[t_rank + 1]].index();
    stored_gain += m[t_index][next_index];
    stored_gain -= m[s_index][next_index];
  } else {
    new_last_s = s_index;
  }

  // Handling end route cost change because vehicle ends can be
  // different or none.
  if (v_source.has_end()) {
    auto end_s = v_source.end.value().index();
    stored_gain += m[last_s][end_s];
    stored_gain -= m[new_last_s][end_s];
  }
  if (v_target.has_end()) {
    auto end_t = v_target.end.value().index();
    stored_gain += m[last_t][end_t];
    stored_gain -= m[new_last_t][end_t];
  }

  gain_computed = true;
}

bool TwoOpt::is_valid() {
  auto t_delivery = target.delivery_in_range(t_rank + 1, t_route.size());
  auto t_pickup = target.pickup_in_range(t_rank + 1, t_route.size());

  bool valid = source.is_valid_addition_for_capacity_margins(_input,
                                                             t_pickup,
                                                             t_delivery,
                                                             s_rank + 1,
                                                             s_route.size());

  auto s_delivery = source.delivery_in_range(s_rank + 1, s_route.size());
  auto s_pickup = source.pickup_in_range(s_rank + 1, s_route.size());

  valid =
    valid && target.is_valid_addition_for_capacity_margins(_input,
                                                           s_pickup,
                                                           s_delivery,
                                                           t_rank + 1,
                                                           t_route.size());

  valid =
    valid && source.is_valid_addition_for_capacity_inclusion(_input,
                                                             t_delivery,
                                                             t_route.begin() +
                                                               t_rank + 1,
                                                             t_route.end(),
                                                             s_rank + 1,
                                                             s_route.size());

  valid =
    valid && target.is_valid_addition_for_capacity_inclusion(_input,
                                                             s_delivery,
                                                             s_route.begin() +
                                                               s_rank + 1,
                                                             s_route.end(),
                                                             t_rank + 1,
                                                             t_route.size());

  return valid;
}

void TwoOpt::apply() {
  auto nb_source = s_route.size() - 1 - s_rank;

  t_route.insert(t_route.begin() + t_rank + 1,
                 s_route.begin() + s_rank + 1,
                 s_route.end());
  s_route.erase(s_route.begin() + s_rank + 1, s_route.end());
  s_route.insert(s_route.end(),
                 t_route.begin() + t_rank + 1 + nb_source,
                 t_route.end());
  t_route.erase(t_route.begin() + t_rank + 1 + nb_source, t_route.end());

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> TwoOpt::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<Index> TwoOpt::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace cvrp
} // namespace vroom
