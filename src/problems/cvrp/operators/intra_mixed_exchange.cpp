/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/intra_mixed_exchange.h"
#include "utils/helpers.h"

namespace vroom {
namespace cvrp {

IntraMixedExchange::IntraMixedExchange(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       RawRoute& s_raw_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank,
                                       bool check_t_reverse)
  : Operator(input,
             sol_state,
             s_raw_route,
             s_vehicle,
             s_rank,
             s_raw_route,
             s_vehicle,
             t_rank),
    _gain_upper_bound_computed(false),
    // Required for consistency in compute_gain if check_t_reverse is
    // false.
    _reversed_s_gain(std::numeric_limits<Gain>::min()),
    reverse_t_edge(false),
    check_t_reverse(check_t_reverse),
    s_is_normal_valid(false),
    s_is_reverse_valid(false),
    _moved_jobs((s_rank < t_rank) ? t_rank - s_rank + 2 : s_rank - t_rank + 1),
    _first_rank(std::min(s_rank, t_rank)),
    _last_rank((t_rank < s_rank) ? s_rank + 1 : t_rank + 2) {
  // If node at s_rank is right before/after edge at t_rank, then the
  // move is a relocate.
  assert(s_rank + 1 < t_rank or t_rank + 2 < s_rank);
  assert(s_route.size() >= 4);
  assert(s_rank < s_route.size());
  assert(t_rank < s_route.size() - 1);

  // Either moving edge with single jobs or a whole shipment.
  assert((_input.jobs[this->t_route[t_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[this->t_route[t_rank + 1]].type == JOB_TYPE::SINGLE and
          check_t_reverse) or
         (_input.jobs[this->t_route[t_rank]].type == JOB_TYPE::PICKUP and
          _input.jobs[this->t_route[t_rank + 1]].type == JOB_TYPE::DELIVERY and
          !check_t_reverse and
          _sol_state.matching_delivery_rank[t_vehicle][t_rank] == t_rank + 1));

  Index s_node;
  if (t_rank < s_rank) {
    s_node = 0;
    _t_edge_first = _moved_jobs.size() - 2;
    _t_edge_last = _moved_jobs.size() - 1;

    std::copy(s_route.begin() + t_rank + 2,
              s_route.begin() + s_rank,
              _moved_jobs.begin() + 1);
  } else {
    _t_edge_first = 0;
    _t_edge_last = 1;
    s_node = _moved_jobs.size() - 1;

    std::copy(s_route.begin() + s_rank + 1,
              s_route.begin() + t_rank,
              _moved_jobs.begin() + 2);
  }

  _moved_jobs[s_node] = s_route[s_rank];
  _moved_jobs[_t_edge_first] = s_route[t_rank];
  _moved_jobs[_t_edge_last] = s_route[t_rank + 1];
}

Gain IntraMixedExchange::gain_upper_bound() {
  const auto& m = _input.get_matrix();
  const auto& v = _input.vehicles[s_vehicle];

  // Consider the cost of replacing node at rank s_rank with target
  // edge. Part of that cost (for adjacent edges) is stored in
  // _sol_state.edge_costs_around_node.  reverse_* checks whether we
  // should change the target edge order.
  Index s_index = _input.jobs[s_route[s_rank]].index();
  Index t_index = _input.jobs[s_route[t_rank]].index();
  Index t_after_index = _input.jobs[s_route[t_rank + 1]].index();

  // Determine costs added with target edge.
  Gain previous_cost = 0;
  Gain next_cost = 0;
  Gain reverse_previous_cost = 0;
  Gain reverse_next_cost = 0;

  if (s_rank == 0) {
    if (v.has_start()) {
      auto p_index = v.start.value().index();
      previous_cost = m[p_index][t_index];
      reverse_previous_cost = m[p_index][t_after_index];
    }
  } else {
    auto p_index = _input.jobs[s_route[s_rank - 1]].index();
    previous_cost = m[p_index][t_index];
    reverse_previous_cost = m[p_index][t_after_index];
  }

  if (s_rank == s_route.size() - 1) {
    if (v.has_end()) {
      auto n_index = v.end.value().index();
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

  // Consider the cost of replacing edge starting at rank t_rank with
  // source node. Part of that cost (for adjacent edges) is stored in
  // _sol_state.edge_costs_around_edge.  reverse_* checks whether we
  // should change the source edge order.
  previous_cost = 0;
  next_cost = 0;

  if (t_rank == 0) {
    if (v.has_start()) {
      auto p_index = v.start.value().index();
      previous_cost = m[p_index][s_index];
    }
  } else {
    auto p_index = _input.jobs[s_route[t_rank - 1]].index();
    previous_cost = m[p_index][s_index];
  }

  if (t_rank == s_route.size() - 2) {
    if (v.has_end()) {
      auto n_index = v.end.value().index();
      next_cost = m[s_index][n_index];
      reverse_next_cost = m[s_index][n_index];
    }
  } else {
    auto n_index = _input.jobs[s_route[t_rank + 2]].index();
    next_cost = m[s_index][n_index];
  }

  _t_gain = _sol_state.edge_costs_around_edge[t_vehicle][t_rank] -
            previous_cost - next_cost;

  _gain_upper_bound_computed = true;

  return s_gain_upper_bound + _t_gain;
}

void IntraMixedExchange::compute_gain() {
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

bool IntraMixedExchange::is_valid() {
  auto delivery = source.delivery_in_range(_first_rank, _last_rank);

  s_is_normal_valid =
    source.is_valid_addition_for_capacity_inclusion(_input,
                                                    delivery,
                                                    _moved_jobs.begin(),
                                                    _moved_jobs.end(),
                                                    _first_rank,
                                                    _last_rank);

  if (check_t_reverse) {
    std::swap(_moved_jobs[_t_edge_first], _moved_jobs[_t_edge_last]);

    s_is_reverse_valid =
      source.is_valid_addition_for_capacity_inclusion(_input,
                                                      delivery,
                                                      _moved_jobs.begin(),
                                                      _moved_jobs.end(),
                                                      _first_rank,
                                                      _last_rank);

    // Reset to initial situation before potential application or TW
    // checks.
    std::swap(_moved_jobs[_t_edge_first], _moved_jobs[_t_edge_last]);
  }

  return s_is_normal_valid or s_is_reverse_valid;
}

void IntraMixedExchange::apply() {
  assert(!reverse_t_edge or
         (_input.jobs[t_route[t_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[t_route[t_rank + 1]].type == JOB_TYPE::SINGLE));

  if (reverse_t_edge) {
    std::swap(s_route[t_rank], s_route[t_rank + 1]);
  }

  std::swap(s_route[s_rank], s_route[t_rank]);

  auto t_after = s_route[t_rank + 1];
  s_route.erase(s_route.begin() + t_rank + 1);

  auto end_t_rank = s_rank + 1;
  if (t_rank < s_rank) {
    --end_t_rank;
  }

  s_route.insert(s_route.begin() + end_t_rank, t_after);

  source.update_amounts(_input);
}

std::vector<Index> IntraMixedExchange::addition_candidates() const {
  return {};
}

std::vector<Index> IntraMixedExchange::update_candidates() const {
  return {s_vehicle};
}

} // namespace cvrp
} // namespace vroom
