/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/intra_or_opt.h"
#include "utils/helpers.h"

namespace vroom {
namespace cvrp {

IntraOrOpt::IntraOrOpt(const Input& input,
                       const utils::SolutionState& sol_state,
                       RawRoute& s_raw_route,
                       Index s_vehicle,
                       Index s_rank,
                       Index t_rank,
                       bool check_reverse)
  : Operator(input,
             sol_state,
             s_raw_route,
             s_vehicle,
             s_rank,
             s_raw_route,
             s_vehicle,
             t_rank),
    _gain_upper_bound_computed(false),
    // Required for consistency in compute_gain if check_reverse is
    // false.
    _reversed_t_gain(std::numeric_limits<Gain>::min()),
    reverse_s_edge(false),
    is_normal_valid(false),
    is_reverse_valid(false),
    check_reverse(check_reverse),
    _moved_jobs((s_rank < t_rank) ? t_rank - s_rank + 2 : s_rank - t_rank + 2),
    _first_rank(std::min(s_rank, t_rank)),
    _last_rank(std::max(s_rank, t_rank) + 2) {
  assert(s_route.size() >= 4);
  assert(s_rank < s_route.size() - 1);
  assert(t_rank <= s_route.size() - 2);
  assert(s_rank != t_rank);
  // Either moving an edge of single jobs or a whole shipment.
  assert((_input.jobs[s_route[s_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[s_route[s_rank + 1]].type == JOB_TYPE::SINGLE and
          check_reverse) or
         (_input.jobs[s_route[s_rank]].type == JOB_TYPE::PICKUP and
          _input.jobs[s_route[s_rank + 1]].type == JOB_TYPE::DELIVERY and
          !check_reverse and
          _sol_state.matching_delivery_rank[s_vehicle][s_rank] == s_rank + 1));

  if (t_rank < s_rank) {
    _s_edge_first = 0;
    _s_edge_last = 1;

    std::copy(s_route.begin() + t_rank,
              s_route.begin() + s_rank,
              _moved_jobs.begin() + 2);
  } else {
    _s_edge_first = _moved_jobs.size() - 2;
    _s_edge_last = _moved_jobs.size() - 1;

    std::copy(s_route.begin() + s_rank + 2,
              s_route.begin() + t_rank + 2,
              _moved_jobs.begin());
  }

  _moved_jobs[_s_edge_first] = s_route[s_rank];
  _moved_jobs[_s_edge_last] = s_route[s_rank + 1];
}

Gain IntraOrOpt::gain_upper_bound() {
  const auto& m = _input.get_matrix();
  const auto& v = _input.vehicles[s_vehicle];

  // The cost of removing edge starting at rank s_rank is already
  // stored in _sol_state.edge_gains[s_vehicle][s_rank].

  // For addition, consider the cost of adding source edge at new rank
  // *after* removal.
  auto new_rank = t_rank;
  if (s_rank < t_rank) {
    new_rank += 2;
  }

  Index s_index = _input.jobs[s_route[s_rank]].index();
  Index after_s_index = _input.jobs[s_route[s_rank + 1]].index();

  Gain previous_cost = 0;
  Gain next_cost = 0;
  Gain reverse_previous_cost = 0;
  Gain reverse_next_cost = 0;
  Gain old_edge_cost = 0;

  if (new_rank == s_route.size()) {
    // Adding edge past the end after a real job that was unmoved.
    auto p_index = _input.jobs[s_route[new_rank - 1]].index();
    previous_cost = m[p_index][s_index];
    reverse_previous_cost = m[p_index][after_s_index];
    if (v.has_end()) {
      auto n_index = v.end.value().index();
      old_edge_cost = m[p_index][n_index];
      next_cost = m[after_s_index][n_index];
      reverse_next_cost = m[s_index][n_index];
    }
  } else {
    // Adding before one of the jobs.
    auto n_index = _input.jobs[s_route[new_rank]].index();
    next_cost = m[after_s_index][n_index];
    reverse_next_cost = m[s_index][n_index];

    if (new_rank == 0) {
      if (v.has_start()) {
        auto p_index = v.start.value().index();
        previous_cost = m[p_index][s_index];
        reverse_previous_cost = m[p_index][after_s_index];
        old_edge_cost = m[p_index][n_index];
      }
    } else {
      auto p_index = _input.jobs[s_route[new_rank - 1]].index();
      previous_cost = m[p_index][s_index];
      reverse_previous_cost = m[p_index][after_s_index];
      old_edge_cost = m[p_index][n_index];
    }
  }

  // Gain for source vehicle.
  _s_gain = _sol_state.edge_gains[s_vehicle][s_rank];

  // Gain for target vehicle.
  _normal_t_gain = old_edge_cost - previous_cost - next_cost;

  auto t_gain_upper_bound = _normal_t_gain;

  if (check_reverse) {
    Gain reverse_edge_cost = static_cast<Gain>(m[s_index][after_s_index]) -
                             static_cast<Gain>(m[after_s_index][s_index]);
    _reversed_t_gain = old_edge_cost + reverse_edge_cost -
                       reverse_previous_cost - reverse_next_cost;

    t_gain_upper_bound = std::max(_normal_t_gain, _reversed_t_gain);
  }

  _gain_upper_bound_computed = true;

  return _s_gain + t_gain_upper_bound;
}

void IntraOrOpt::compute_gain() {
  assert(_gain_upper_bound_computed);
  assert(is_normal_valid or is_reverse_valid);

  stored_gain = _s_gain;

  if (_reversed_t_gain > _normal_t_gain) {
    // Biggest potential gain is obtained when reversing edge.
    if (is_reverse_valid) {
      reverse_s_edge = true;
      stored_gain += _reversed_t_gain;
    } else {
      stored_gain += _normal_t_gain;
    }
  } else {
    // Biggest potential gain is obtained when not reversing edge.
    if (is_normal_valid) {
      stored_gain += _normal_t_gain;
    } else {
      reverse_s_edge = true;
      stored_gain += _reversed_t_gain;
    }
  }

  gain_computed = true;
}

bool IntraOrOpt::is_valid() {
  is_normal_valid = source.is_valid_addition_for_capacity_inclusion(
    _input,
    source.delivery_in_range(_first_rank, _last_rank),
    _moved_jobs.begin(),
    _moved_jobs.end(),
    _first_rank,
    _last_rank);

  if (check_reverse) {
    std::swap(_moved_jobs[_s_edge_first], _moved_jobs[_s_edge_last]);

    is_reverse_valid = source.is_valid_addition_for_capacity_inclusion(
      _input,
      source.delivery_in_range(_first_rank, _last_rank),
      _moved_jobs.begin(),
      _moved_jobs.end(),
      _first_rank,
      _last_rank);

    // Reset to initial situation before potential application or TW
    // checks.
    std::swap(_moved_jobs[_s_edge_first], _moved_jobs[_s_edge_last]);
  }

  return is_normal_valid or is_reverse_valid;
}

void IntraOrOpt::apply() {
  assert(!reverse_s_edge or
         (_input.jobs[s_route[s_rank]].type == JOB_TYPE::SINGLE and
          _input.jobs[s_route[s_rank + 1]].type == JOB_TYPE::SINGLE));

  auto first_job_rank = s_route[s_rank];
  auto second_job_rank = s_route[s_rank + 1];
  s_route.erase(s_route.begin() + s_rank, s_route.begin() + s_rank + 2);
  s_route.insert(s_route.begin() + t_rank, {first_job_rank, second_job_rank});
  if (reverse_s_edge) {
    std::swap(t_route[t_rank], t_route[t_rank + 1]);
  }

  source.update_amounts(_input);
}

std::vector<Index> IntraOrOpt::addition_candidates() const {
  return {};
}

std::vector<Index> IntraOrOpt::update_candidates() const {
  return {s_vehicle};
}

} // namespace cvrp
} // namespace vroom
