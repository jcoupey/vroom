/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/intra_cross_exchange.h"
#include "utils/helpers.h"

namespace vroom {
namespace cvrp {

IntraCrossExchange::IntraCrossExchange(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       RawRoute& s_raw_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank,
                                       bool check_s_reverse,
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
    // Required for consistency in compute_gain if check_s_reverse or
    // check_t_reverse are false.
    _reversed_s_gain(std::numeric_limits<Gain>::min()),
    _reversed_t_gain(std::numeric_limits<Gain>::min()),
    reverse_s_edge(false),
    reverse_t_edge(false),
    check_s_reverse(check_s_reverse),
    check_t_reverse(check_t_reverse),
    s_normal_t_normal_is_valid(false),
    s_normal_t_reverse_is_valid(false),
    s_reverse_t_reverse_is_valid(false),
    s_reverse_t_normal_is_valid(false),
    _moved_jobs(t_rank - s_rank + 2),
    _first_rank(s_rank),
    _last_rank(t_rank + 2) {
  // Use s_rank as smallest rank for symmetry reasons.
  assert(s_rank + 2 < t_rank); // Avoid common edge.
  assert(s_route.size() >= 5);
  assert(t_rank < s_route.size() - 1);

  // Either moving edges of single jobs or whole shipments.
  assert((_input.jobs[this->s_route[s_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[this->s_route[s_rank + 1]].type == JOB_TYPE::SINGLE and
          check_s_reverse) or
         (_input.jobs[this->s_route[s_rank]].type == JOB_TYPE::PICKUP and
          _input.jobs[this->s_route[s_rank + 1]].type == JOB_TYPE::DELIVERY and
          !check_s_reverse and
          _sol_state.matching_delivery_rank[s_vehicle][s_rank] == s_rank + 1));
  assert((_input.jobs[this->t_route[t_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[this->t_route[t_rank + 1]].type == JOB_TYPE::SINGLE and
          check_t_reverse) or
         (_input.jobs[this->t_route[t_rank]].type == JOB_TYPE::PICKUP and
          _input.jobs[this->t_route[t_rank + 1]].type == JOB_TYPE::DELIVERY and
          !check_t_reverse and
          _sol_state.matching_delivery_rank[t_vehicle][t_rank] == t_rank + 1));

  _moved_jobs[0] = s_route[t_rank];
  _moved_jobs[1] = s_route[t_rank + 1];
  std::copy(s_route.begin() + s_rank + 2,
            s_route.begin() + t_rank,
            _moved_jobs.begin() + 2);
  _moved_jobs[_moved_jobs.size() - 2] = s_route[s_rank];
  _moved_jobs[_moved_jobs.size() - 1] = s_route[s_rank + 1];
}

Gain IntraCrossExchange::gain_upper_bound() {
  const auto& m = _input.get_matrix();
  const auto& v = _input.vehicles[s_vehicle];

  // Consider the cost of replacing edge starting at rank s_rank with
  // target edge. Part of that cost (for adjacent edges) is stored in
  // _sol_state.edge_costs_around_edge.  reverse_* checks whether we
  // should change the target edge order.
  Index s_index = _input.jobs[s_route[s_rank]].index();
  Index s_after_index = _input.jobs[s_route[s_rank + 1]].index();
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

  auto n_index = _input.jobs[s_route[s_rank + 2]].index();
  next_cost = m[t_after_index][n_index];
  reverse_next_cost = m[t_index][n_index];

  _normal_s_gain = _sol_state.edge_costs_around_edge[s_vehicle][s_rank] -
                   previous_cost - next_cost;

  auto s_gain_upper_bound = _normal_s_gain;

  if (check_t_reverse) {
    Gain reverse_edge_cost = static_cast<Gain>(m[t_index][t_after_index]) -
                             static_cast<Gain>(m[t_after_index][t_index]);
    _reversed_s_gain = _sol_state.edge_costs_around_edge[s_vehicle][s_rank] +
                       reverse_edge_cost - reverse_previous_cost -
                       reverse_next_cost;

    s_gain_upper_bound = std::max(_normal_s_gain, _reversed_s_gain);
  }

  // Consider the cost of replacing edge starting at rank t_rank with
  // source edge. Part of that cost (for adjacent edges) is stored in
  // _sol_state.edge_costs_around_edge.  reverse_* checks whether we
  // should change the source edge order.
  next_cost = 0;
  reverse_previous_cost = 0;
  reverse_next_cost = 0;

  auto p_index = _input.jobs[s_route[t_rank - 1]].index();
  previous_cost = m[p_index][s_index];
  reverse_previous_cost = m[p_index][s_after_index];

  if (t_rank == s_route.size() - 2) {
    if (v.has_end()) {
      auto n_index = v.end.value().index();
      next_cost = m[s_after_index][n_index];
      reverse_next_cost = m[s_index][n_index];
    }
  } else {
    auto n_index = _input.jobs[s_route[t_rank + 2]].index();
    next_cost = m[s_after_index][n_index];
    reverse_next_cost = m[s_index][n_index];
  }

  _normal_t_gain = _sol_state.edge_costs_around_edge[t_vehicle][t_rank] -
                   previous_cost - next_cost;

  auto t_gain_upper_bound = _normal_t_gain;

  if (check_s_reverse) {
    Gain reverse_edge_cost = static_cast<Gain>(m[s_index][s_after_index]) -
                             static_cast<Gain>(m[s_after_index][s_index]);
    _reversed_t_gain = _sol_state.edge_costs_around_edge[t_vehicle][t_rank] +
                       reverse_edge_cost - reverse_previous_cost -
                       reverse_next_cost;

    t_gain_upper_bound = std::max(_normal_t_gain, _reversed_t_gain);
  }

  _gain_upper_bound_computed = true;

  return s_gain_upper_bound + t_gain_upper_bound;
}

void IntraCrossExchange::compute_gain() {
  assert(_gain_upper_bound_computed);
  assert(s_normal_t_normal_is_valid or s_normal_t_reverse_is_valid or
         s_reverse_t_reverse_is_valid or s_reverse_t_normal_is_valid);

  stored_gain = std::numeric_limits<Gain>::min();

  if (s_normal_t_normal_is_valid) {
    Gain current_gain = _normal_s_gain + _normal_t_gain;
    if (current_gain > stored_gain) {
      stored_gain = current_gain;
      reverse_s_edge = false;
      reverse_t_edge = false;
    }
  }

  if (s_normal_t_reverse_is_valid) {
    Gain current_gain = _reversed_s_gain + _normal_t_gain;
    if (current_gain > stored_gain) {
      stored_gain = current_gain;
      reverse_s_edge = false;
      reverse_t_edge = true;
    }
  }

  if (s_reverse_t_reverse_is_valid) {
    Gain current_gain = _reversed_s_gain + _reversed_t_gain;
    if (current_gain > stored_gain) {
      stored_gain = current_gain;
      reverse_s_edge = true;
      reverse_t_edge = true;
    }
  }

  if (s_reverse_t_normal_is_valid) {
    Gain current_gain = _normal_s_gain + _reversed_t_gain;
    if (current_gain > stored_gain) {
      stored_gain = current_gain;
      reverse_s_edge = true;
      reverse_t_edge = false;
    }
  }

  gain_computed = true;
}

bool IntraCrossExchange::is_valid() {
  auto delivery = source.delivery_in_range(_first_rank, _last_rank);

  s_normal_t_normal_is_valid =
    source.is_valid_addition_for_capacity_inclusion(_input,
                                                    delivery,
                                                    _moved_jobs.begin(),
                                                    _moved_jobs.end(),
                                                    _first_rank,
                                                    _last_rank);

  std::swap(_moved_jobs[0], _moved_jobs[1]);

  if (check_t_reverse) {
    s_normal_t_reverse_is_valid =
      source.is_valid_addition_for_capacity_inclusion(_input,
                                                      delivery,
                                                      _moved_jobs.begin(),
                                                      _moved_jobs.end(),
                                                      _first_rank,
                                                      _last_rank);
  }

  std::swap(_moved_jobs[_moved_jobs.size() - 2],
            _moved_jobs[_moved_jobs.size() - 1]);

  if (check_s_reverse and check_t_reverse) {
    s_reverse_t_reverse_is_valid =
      source.is_valid_addition_for_capacity_inclusion(_input,
                                                      delivery,
                                                      _moved_jobs.begin(),
                                                      _moved_jobs.end(),
                                                      _first_rank,
                                                      _last_rank);
  }

  std::swap(_moved_jobs[0], _moved_jobs[1]);

  if (check_s_reverse) {
    s_reverse_t_normal_is_valid =
      source.is_valid_addition_for_capacity_inclusion(_input,
                                                      delivery,
                                                      _moved_jobs.begin(),
                                                      _moved_jobs.end(),
                                                      _first_rank,
                                                      _last_rank);
  }

  // Reset to initial situation before potential application and TW
  // checks.
  std::swap(_moved_jobs[_moved_jobs.size() - 2],
            _moved_jobs[_moved_jobs.size() - 1]);

  return s_normal_t_normal_is_valid or s_normal_t_reverse_is_valid or
         s_reverse_t_reverse_is_valid or s_reverse_t_normal_is_valid;
}

void IntraCrossExchange::apply() {
  assert(!reverse_s_edge or
         (_input.jobs[s_route[s_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[s_route[s_rank + 1]].type == JOB_TYPE::SINGLE));
  assert(!reverse_t_edge or
         (_input.jobs[t_route[t_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[t_route[t_rank + 1]].type == JOB_TYPE::SINGLE));

  std::swap(s_route[s_rank], s_route[t_rank]);
  std::swap(s_route[s_rank + 1], s_route[t_rank + 1]);

  if (reverse_s_edge) {
    std::swap(s_route[t_rank], s_route[t_rank + 1]);
  }
  if (reverse_t_edge) {
    std::swap(s_route[s_rank], s_route[s_rank + 1]);
  }

  source.update_amounts(_input);
}

std::vector<Index> IntraCrossExchange::addition_candidates() const {
  return {};
}

std::vector<Index> IntraCrossExchange::update_candidates() const {
  return {s_vehicle};
}

} // namespace cvrp
} // namespace vroom
