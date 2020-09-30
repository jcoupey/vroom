/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/exchange.h"

namespace vroom {
namespace cvrp {

Exchange::Exchange(const Input& input,
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
  assert(_input.vehicle_ok_with_job(t_vehicle, this->s_route[s_rank]));
  assert(_input.vehicle_ok_with_job(s_vehicle, this->t_route[t_rank]));
}

void Exchange::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v_source = _input.vehicles[s_vehicle];
  const auto& v_target = _input.vehicles[t_vehicle];

  // For source vehicle, we consider the cost of replacing job at rank
  // s_rank with target job. Part of that cost (for adjacent
  // edges) is stored in _sol_state.edge_costs_around_node.
  Index s_index = _input.jobs[s_route[s_rank]].index();
  Index t_index = _input.jobs[t_route[t_rank]].index();

  // Determine costs added with target job.
  Gain new_previous_cost = 0;
  Gain new_next_cost = 0;

  if (s_rank == 0) {
    if (v_source.has_start()) {
      auto p_index = v_source.start.value().index();
      new_previous_cost = m[p_index][t_index];
    }
  } else {
    auto p_index = _input.jobs[s_route[s_rank - 1]].index();
    new_previous_cost = m[p_index][t_index];
  }

  if (s_rank == s_route.size() - 1) {
    if (v_source.has_end()) {
      auto n_index = v_source.end.value().index();
      new_next_cost = m[t_index][n_index];
    }
  } else {
    auto n_index = _input.jobs[s_route[s_rank + 1]].index();
    new_next_cost = m[t_index][n_index];
  }

  Gain s_gain = _sol_state.edge_costs_around_node[s_vehicle][s_rank] -
                new_previous_cost - new_next_cost;

  // For target vehicle, we consider the cost of replacing job at rank
  // t_rank with source job. Part of that cost (for adjacent
  // edges) is stored in _sol_state.edge_costs_around_node.

  // Determine costs added with source job.
  new_previous_cost = 0;
  new_next_cost = 0;

  if (t_rank == 0) {
    if (v_target.has_start()) {
      auto p_index = v_target.start.value().index();
      new_previous_cost = m[p_index][s_index];
    }
  } else {
    auto p_index = _input.jobs[t_route[t_rank - 1]].index();
    new_previous_cost = m[p_index][s_index];
  }

  if (t_rank == t_route.size() - 1) {
    if (v_target.has_end()) {
      auto n_index = v_target.end.value().index();
      new_next_cost = m[s_index][n_index];
    }
  } else {
    auto n_index = _input.jobs[t_route[t_rank + 1]].index();
    new_next_cost = m[s_index][n_index];
  }

  Gain t_gain = _sol_state.edge_costs_around_node[t_vehicle][t_rank] -
                new_previous_cost - new_next_cost;

  stored_gain = s_gain + t_gain;
  gain_computed = true;
}

bool Exchange::is_valid() {
  bool valid =
    target.is_valid_addition_for_capacity_margins(_input,
                                                  _input.jobs[s_route[s_rank]]
                                                    .pickup,
                                                  _input.jobs[s_route[s_rank]]
                                                    .delivery,
                                                  t_rank,
                                                  t_rank + 1);

  valid =
    valid &&
    source.is_valid_addition_for_capacity_margins(_input,
                                                  _input.jobs[t_route[t_rank]]
                                                    .pickup,
                                                  _input.jobs[t_route[t_rank]]
                                                    .delivery,
                                                  s_rank,
                                                  s_rank + 1);

  return valid;
}

void Exchange::apply() {
  std::swap(s_route[s_rank], t_route[t_rank]);

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> Exchange::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<Index> Exchange::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace cvrp
} // namespace vroom
