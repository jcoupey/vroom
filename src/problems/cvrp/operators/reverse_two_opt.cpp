/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/reverse_two_opt.h"

namespace vroom {
namespace cvrp {

ReverseTwoOpt::ReverseTwoOpt(const Input& input,
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
  assert(t_rank < _sol_state.fwd_skill_rank[t_vehicle][s_vehicle]);
}

void ReverseTwoOpt::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v_source = _input.vehicles[s_vehicle];
  const auto& v_target = _input.vehicles[t_vehicle];

  Index s_index = _input.jobs[s_route[s_rank]].index();
  Index t_index = _input.jobs[t_route[t_rank]].index();
  Index last_s = _input.jobs[s_route.back()].index();
  Index first_t = _input.jobs[t_route.front()].index();
  stored_gain = 0;
  bool last_in_source = (s_rank == s_route.size() - 1);
  bool last_in_target = (t_rank == t_route.size() - 1);

  // Cost of swapping route for vehicle s_vehicle after step
  // s_rank with route for vehicle t_vehicle up to step
  // t_rank, but reversed.

  // Add new source -> target edge.
  stored_gain -= m[s_index][t_index];

  // Cost of reversing target route portion.
  stored_gain += _sol_state.fwd_costs[t_vehicle][t_rank];
  stored_gain -= _sol_state.bwd_costs[t_vehicle][t_rank];

  if (!last_in_target) {
    // Spare next edge in target route.
    Index next_index = _input.jobs[t_route[t_rank + 1]].index();
    stored_gain += m[t_index][next_index];
  }

  if (!last_in_source) {
    // Spare next edge in source route.
    Index next_index = _input.jobs[s_route[s_rank + 1]].index();
    stored_gain += m[s_index][next_index];

    // Part of source route is moved to target route.
    Index next_s_index = _input.jobs[s_route[s_rank + 1]].index();

    // Cost or reverting source route portion.
    stored_gain += _sol_state.fwd_costs[s_vehicle].back();
    stored_gain -= _sol_state.fwd_costs[s_vehicle][s_rank + 1];
    stored_gain -= _sol_state.bwd_costs[s_vehicle].back();
    stored_gain += _sol_state.bwd_costs[s_vehicle][s_rank + 1];

    if (last_in_target) {
      if (v_target.has_end()) {
        // Handle target route new end.
        auto end_t = v_target.end.value().index();
        stored_gain += m[t_index][end_t];
        stored_gain -= m[next_s_index][end_t];
      }
    } else {
      // Add new target -> source edge.
      Index next_t_index = _input.jobs[t_route[t_rank + 1]].index();
      stored_gain -= m[next_s_index][next_t_index];
    }
  }

  if (v_source.has_end()) {
    // Update cost to source end because last job changed.
    auto end_s = v_source.end.value().index();
    stored_gain += m[last_s][end_s];
    stored_gain -= m[first_t][end_s];
  }

  if (v_target.has_start()) {
    // Spare cost from target start because first job changed.
    auto start_t = v_target.start.value().index();
    stored_gain += m[start_t][first_t];
    if (!last_in_source) {
      stored_gain -= m[start_t][last_s];
    } else {
      // No job from source route actually swapped to target route.
      if (!last_in_target) {
        // Going straight from start to next job in target route.
        Index next_index = _input.jobs[t_route[t_rank + 1]].index();
        stored_gain -= m[start_t][next_index];
      } else {
        // Emptying the whole target route here, so also gaining cost
        // to end if it exists.
        if (v_target.has_end()) {
          auto end_t = v_target.end.value().index();
          stored_gain += m[t_index][end_t];
        }
      }
    }
  }

  gain_computed = true;
}

bool ReverseTwoOpt::is_valid() {
  auto t_delivery = target.delivery_in_range(0, t_rank + 1);
  auto t_pickup = target.pickup_in_range(0, t_rank + 1);

  bool valid = source.is_valid_addition_for_capacity_margins(_input,
                                                             t_pickup,
                                                             t_delivery,
                                                             s_rank + 1,
                                                             s_route.size());

  auto s_delivery = source.delivery_in_range(s_rank + 1, s_route.size());
  auto s_pickup = source.pickup_in_range(s_rank + 1, s_route.size());

  valid = valid && target.is_valid_addition_for_capacity_margins(_input,
                                                                 s_pickup,
                                                                 s_delivery,
                                                                 0,
                                                                 t_rank + 1);

  valid =
    valid && source.is_valid_addition_for_capacity_inclusion(_input,
                                                             t_delivery,
                                                             t_route.rbegin() +
                                                               t_route.size() -
                                                               1 - t_rank,
                                                             t_route.rend(),
                                                             s_rank + 1,
                                                             s_route.size());

  valid =
    valid && target.is_valid_addition_for_capacity_inclusion(_input,
                                                             s_delivery,
                                                             s_route.rbegin(),
                                                             s_route.rbegin() +
                                                               s_route.size() -
                                                               1 - s_rank,
                                                             0,
                                                             t_rank + 1);

  return valid;
}

void ReverseTwoOpt::apply() {
  auto nb_source = s_route.size() - 1 - s_rank;

  t_route.insert(t_route.begin(),
                 s_route.rbegin(),
                 s_route.rbegin() + nb_source);
  s_route.erase(s_route.begin() + s_rank + 1, s_route.end());
  s_route.insert(s_route.end(),
                 t_route.rend() - t_rank - nb_source - 1,
                 t_route.rend() - nb_source);
  t_route.erase(t_route.begin() + nb_source,
                t_route.begin() + nb_source + t_rank + 1);

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> ReverseTwoOpt::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<Index> ReverseTwoOpt::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace cvrp
} // namespace vroom
