/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/mixed_exchange.h"

namespace vroom {
namespace cvrp {

MixedExchange::MixedExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             RawRoute& s_route,
                             Index s_vehicle,
                             Index s_rank,
                             RawRoute& t_route,
                             Index t_vehicle,
                             Index t_rank,
                             bool check_t_reverse)
  : Operator(input,
             sol_state,
             s_route,
             s_vehicle,
             s_rank,
             t_route,
             t_vehicle,
             t_rank),
    _gain_upper_bound_computed(false),
    // Required for consistency in compute_gain if check_t_reverse is
    // false.
    _reversed_s_gain(std::numeric_limits<Gain>::min()),
    reverse_t_edge(false),
    check_t_reverse(check_t_reverse),
    s_is_normal_valid(false),
    s_is_reverse_valid(false) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 1);
  assert(t_route.size() >= 2);
  assert(s_rank < s_route.size());
  assert(t_rank < t_route.size() - 1);

  assert(_input.vehicle_ok_with_job(t_vehicle, this->s_route[s_rank]));
  assert(_input.vehicle_ok_with_job(s_vehicle, this->t_route[t_rank]));
  assert(_input.vehicle_ok_with_job(s_vehicle, this->t_route[t_rank + 1]));

  // Either moving edge with single jobs or a whole shipment.
  assert((_input.jobs[this->t_route[t_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[this->t_route[t_rank + 1]].type == JOB_TYPE::SINGLE and
          check_t_reverse) or
         (_input.jobs[this->t_route[t_rank]].type == JOB_TYPE::PICKUP and
          _input.jobs[this->t_route[t_rank + 1]].type == JOB_TYPE::DELIVERY and
          !check_t_reverse and
          _sol_state.matching_delivery_rank[t_vehicle][t_rank] == t_rank + 1));
}

Gain MixedExchange::gain_upper_bound() {
  const auto& m = _input.get_matrix();
  const auto& v_source = _input.vehicles[s_vehicle];
  const auto& v_target = _input.vehicles[t_vehicle];

  // For source vehicle, we consider the cost of replacing job at rank
  // s_rank with target edge. Part of that cost (for adjacent edges)
  // is stored in _sol_state.edge_costs_around_node. reverse_t_edge
  // checks whether we should change the target edge order.
  Index s_index = _input.jobs[s_route[s_rank]].index();
  Index t_index = _input.jobs[t_route[t_rank]].index();
  Index t_after_index = _input.jobs[t_route[t_rank + 1]].index();

  // Determine costs added with target edge.
  Gain previous_cost = 0;
  Gain next_cost = 0;
  Gain reverse_previous_cost = 0;
  Gain reverse_next_cost = 0;

  if (s_rank == 0) {
    if (v_source.has_start()) {
      auto p_index = v_source.start.value().index();
      previous_cost = m[p_index][t_index];
      reverse_previous_cost = m[p_index][t_after_index];
    }
  } else {
    auto p_index = _input.jobs[s_route[s_rank - 1]].index();
    previous_cost = m[p_index][t_index];
    reverse_previous_cost = m[p_index][t_after_index];
  }

  if (s_rank == s_route.size() - 1) {
    if (v_source.has_end()) {
      auto n_index = v_source.end.value().index();
      next_cost = m[t_after_index][n_index];
      reverse_next_cost = m[t_index][n_index];
    }
  } else {
    auto n_index = _input.jobs[s_route[s_rank + 1]].index();
    next_cost = m[t_after_index][n_index];
    reverse_next_cost = m[t_index][n_index];
  }

  _normal_s_gain = _sol_state.edge_costs_around_node[s_vehicle][s_rank] -
                   previous_cost - next_cost;

  auto s_gain_upper_bound = _normal_s_gain;

  if (check_t_reverse) {
    Gain reverse_edge_cost = static_cast<Gain>(m[t_index][t_after_index]) -
                             static_cast<Gain>(m[t_after_index][t_index]);
    _reversed_s_gain = _sol_state.edge_costs_around_node[s_vehicle][s_rank] +
                       reverse_edge_cost - reverse_previous_cost -
                       reverse_next_cost;

    s_gain_upper_bound = std::max(_normal_s_gain, _reversed_s_gain);
  }

  // For target vehicle, we consider the cost of replacing edge at
  // rank t_rank with source job. Part of that cost (for adjacent
  // edges) is stored in _sol_state.edge_costs_around_edges.

  // Determine costs added with source job.
  previous_cost = 0;
  next_cost = 0;

  if (t_rank == 0) {
    if (v_target.has_start()) {
      auto p_index = v_target.start.value().index();
      previous_cost = m[p_index][s_index];
    }
  } else {
    auto p_index = _input.jobs[t_route[t_rank - 1]].index();
    previous_cost = m[p_index][s_index];
  }

  if (t_rank == t_route.size() - 2) {
    if (v_target.has_end()) {
      auto n_index = v_target.end.value().index();
      next_cost = m[s_index][n_index];
    }
  } else {
    auto n_index = _input.jobs[t_route[t_rank + 2]].index();
    next_cost = m[s_index][n_index];
  }

  _t_gain = _sol_state.edge_costs_around_edge[t_vehicle][t_rank] -
            previous_cost - next_cost;

  _gain_upper_bound_computed = true;

  return s_gain_upper_bound + _t_gain;
}

void MixedExchange::compute_gain() {
  assert(_gain_upper_bound_computed);
  assert(s_is_normal_valid or s_is_reverse_valid);
  if (_reversed_s_gain > _normal_s_gain) {
    // Biggest potential gain is obtained when reversing edge.
    if (s_is_reverse_valid) {
      stored_gain += _reversed_s_gain;
      reverse_t_edge = true;
    } else {
      stored_gain += _normal_s_gain;
    }
  } else {
    // Biggest potential gain is obtained when not reversing edge.
    if (s_is_normal_valid) {
      stored_gain += _normal_s_gain;
    } else {
      stored_gain += _reversed_s_gain;
      reverse_t_edge = true;
    }
  }

  stored_gain += _t_gain;

  gain_computed = true;
}

bool MixedExchange::is_valid() {
  bool valid =
    target.is_valid_addition_for_capacity_margins(_input,
                                                  _input.jobs[s_route[s_rank]]
                                                    .pickup,
                                                  _input.jobs[s_route[s_rank]]
                                                    .delivery,
                                                  t_rank,
                                                  t_rank + 2);

  auto target_pickup = _input.jobs[t_route[t_rank]].pickup +
                       _input.jobs[t_route[t_rank + 1]].pickup;
  auto target_delivery = _input.jobs[t_route[t_rank]].delivery +
                         _input.jobs[t_route[t_rank + 1]].delivery;

  valid =
    valid && source.is_valid_addition_for_capacity_margins(_input,
                                                           target_pickup,
                                                           target_delivery,
                                                           s_rank,
                                                           s_rank + 1);

  if (valid) {
    // Keep target edge direction when inserting in source route.
    auto t_start = t_route.begin() + t_rank;

    s_is_normal_valid =
      source.is_valid_addition_for_capacity_inclusion(_input,
                                                      target_delivery,
                                                      t_start,
                                                      t_start + 2,
                                                      s_rank,
                                                      s_rank + 1);
    if (check_t_reverse) {
      // Reverse target edge direction when inserting in source route.
      auto t_reverse_start = t_route.rbegin() + t_route.size() - 2 - t_rank;
      s_is_reverse_valid =
        source.is_valid_addition_for_capacity_inclusion(_input,
                                                        target_delivery,
                                                        t_reverse_start,
                                                        t_reverse_start + 2,
                                                        s_rank,
                                                        s_rank + 1);
    }

    valid = s_is_normal_valid or s_is_reverse_valid;
  }

  return valid;
}

void MixedExchange::apply() {
  assert(!reverse_t_edge or
         (_input.jobs[t_route[t_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[t_route[t_rank + 1]].type == JOB_TYPE::SINGLE));

  std::swap(s_route[s_rank], t_route[t_rank]);
  s_route.insert(s_route.begin() + s_rank + 1,
                 t_route.begin() + t_rank + 1,
                 t_route.begin() + t_rank + 2);
  t_route.erase(t_route.begin() + t_rank + 1);

  if (reverse_t_edge) {
    std::swap(s_route[s_rank], s_route[s_rank + 1]);
  }

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> MixedExchange::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<Index> MixedExchange::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace cvrp
} // namespace vroom
