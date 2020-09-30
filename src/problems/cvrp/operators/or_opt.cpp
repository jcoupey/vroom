/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/or_opt.h"

namespace vroom {
namespace cvrp {

OrOpt::OrOpt(const Input& input,
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
             t_rank),
    _gain_upper_bound_computed(false),
    reverse_s_edge(false),
    is_normal_valid(false),
    is_reverse_valid(false) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 2);
  assert(s_rank < s_route.size() - 1);
  assert(t_rank <= t_route.size());

  assert(_input.vehicle_ok_with_job(t_vehicle, this->s_route[s_rank]));
  assert(_input.vehicle_ok_with_job(t_vehicle, this->s_route[s_rank + 1]));
}

Gain OrOpt::gain_upper_bound() {
  const auto& m = _input.get_matrix();
  const auto& v_target = _input.vehicles[t_vehicle];

  // For source vehicle, we consider the cost of removing edge
  // starting at rank s_rank, already stored in
  // _sol_state.edge_gains[s_vehicle][s_rank].

  // For target vehicle, we consider the cost of adding source edge at
  // rank t_rank. reverse_* checks whether we should change the
  // source edge order.
  Index s_index = _input.jobs[s_route[s_rank]].index();
  Index after_s_index = _input.jobs[s_route[s_rank + 1]].index();

  Gain previous_cost = 0;
  Gain next_cost = 0;
  Gain reverse_previous_cost = 0;
  Gain reverse_next_cost = 0;
  Gain old_edge_cost = 0;

  if (t_rank == t_route.size()) {
    if (t_route.size() == 0) {
      // Adding edge to an empty route.
      if (v_target.has_start()) {
        previous_cost = m[v_target.start.value().index()][s_index];
        reverse_previous_cost =
          m[v_target.start.value().index()][after_s_index];
      }
      if (v_target.has_end()) {
        next_cost = m[after_s_index][v_target.end.value().index()];
        reverse_next_cost = m[s_index][v_target.end.value().index()];
      }
    } else {
      // Adding edge past the end after a real job.
      auto p_index = _input.jobs[t_route[t_rank - 1]].index();
      previous_cost = m[p_index][s_index];
      reverse_previous_cost = m[p_index][after_s_index];
      if (v_target.has_end()) {
        auto n_index = v_target.end.value().index();
        old_edge_cost = m[p_index][n_index];
        next_cost = m[after_s_index][n_index];
        reverse_next_cost = m[s_index][n_index];
      }
    }
  } else {
    // Adding before one of the jobs.
    auto n_index = _input.jobs[t_route[t_rank]].index();
    next_cost = m[after_s_index][n_index];
    reverse_next_cost = m[s_index][n_index];

    if (t_rank == 0) {
      if (v_target.has_start()) {
        auto p_index = v_target.start.value().index();
        previous_cost = m[p_index][s_index];
        reverse_previous_cost = m[p_index][after_s_index];
        old_edge_cost = m[p_index][n_index];
      }
    } else {
      auto p_index = _input.jobs[t_route[t_rank - 1]].index();
      previous_cost = m[p_index][s_index];
      reverse_previous_cost = m[p_index][after_s_index];
      old_edge_cost = m[p_index][n_index];
    }
  }

  // Gain for source vehicle.
  _s_gain = _sol_state.edge_gains[s_vehicle][s_rank];

  // Gain for target vehicle.
  _normal_t_gain = old_edge_cost - previous_cost - next_cost;

  Gain reverse_edge_cost = static_cast<Gain>(m[s_index][after_s_index]) -
                           static_cast<Gain>(m[after_s_index][s_index]);
  _reversed_t_gain = old_edge_cost + reverse_edge_cost - reverse_previous_cost -
                     reverse_next_cost;

  _gain_upper_bound_computed = true;

  return _s_gain + std::max(_normal_t_gain, _reversed_t_gain);
}

void OrOpt::compute_gain() {
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

bool OrOpt::is_valid() {
  auto edge_pickup = _input.jobs[s_route[s_rank]].pickup +
                     _input.jobs[s_route[s_rank + 1]].pickup;
  auto edge_delivery = _input.jobs[s_route[s_rank]].delivery +
                       _input.jobs[s_route[s_rank + 1]].delivery;

  bool valid = target.is_valid_addition_for_capacity(_input,
                                                     edge_pickup,
                                                     edge_delivery,
                                                     t_rank);

  if (valid) {
    // Keep edge direction.
    auto s_start = s_route.begin() + s_rank;

    is_normal_valid =
      target.is_valid_addition_for_capacity_inclusion(_input,
                                                      edge_delivery,
                                                      s_start,
                                                      s_start + 2,
                                                      t_rank,
                                                      t_rank);

    // Reverse edge direction.
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    is_reverse_valid =
      target.is_valid_addition_for_capacity_inclusion(_input,
                                                      edge_delivery,
                                                      s_reverse_start,
                                                      s_reverse_start + 2,
                                                      t_rank,
                                                      t_rank);

    valid = (is_normal_valid or is_reverse_valid);
  }

  return valid;
}

void OrOpt::apply() {
  t_route.insert(t_route.begin() + t_rank,
                 s_route.begin() + s_rank,
                 s_route.begin() + s_rank + 2);
  if (reverse_s_edge) {
    std::swap(t_route[t_rank], t_route[t_rank + 1]);
  }

  s_route.erase(s_route.begin() + s_rank, s_route.begin() + s_rank + 2);

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> OrOpt::addition_candidates() const {
  return {s_vehicle};
}

std::vector<Index> OrOpt::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace cvrp
} // namespace vroom
