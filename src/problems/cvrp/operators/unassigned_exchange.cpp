/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/unassigned_exchange.h"
#include "utils/helpers.h"

namespace vroom {
namespace cvrp {

UnassignedExchange::UnassignedExchange(const Input& input,
                                       const utils::SolutionState& sol_state,
                                       std::unordered_set<Index>& unassigned,
                                       RawRoute& s_raw_route,
                                       Index s_vehicle,
                                       Index s_rank,
                                       Index t_rank,
                                       Index u)
  : Operator(input,
             sol_state,
             s_raw_route,
             s_vehicle,
             s_rank,
             s_raw_route,
             s_vehicle,
             t_rank),
    _u(u),
    _unassigned(unassigned),
    _first_rank(std::min(s_rank, t_rank)),
    _last_rank((s_rank < t_rank) ? t_rank : s_rank + 1),
    _moved_jobs(_last_rank - _first_rank),
    _removed(s_route[s_rank]) {
  assert(t_rank != s_rank + 1);
  assert(!s_route.empty());
  assert(s_rank < s_route.size());
  assert(t_rank <= s_route.size());

  if (s_rank < t_rank) {
    std::copy(s_route.begin() + s_rank + 1,
              s_route.begin() + t_rank,
              _moved_jobs.begin());
    _moved_jobs.back() = u;
  } else {
    std::copy(s_route.begin() + t_rank,
              s_route.begin() + s_rank,
              _moved_jobs.begin() + 1);
    _moved_jobs.front() = u;
  }
}

void UnassignedExchange::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v = _input.vehicles[s_vehicle];

  const Index u_index = _input.jobs[_u].index();

  Gain s_gain;
  Gain t_gain;

  if (t_rank == s_rank) {
    // Removed job is replaced by the unassigned one so there is no
    // new edge in place of removal.
    s_gain = _sol_state.edge_costs_around_node[s_vehicle][s_rank];

    // No old edge to remove when adding unassigned job in place of
    // removed job.
    Gain previous_cost = 0;
    Gain next_cost = 0;

    if (t_rank == 0) {
      if (v.has_start()) {
        previous_cost = m[v.start.value().index()][u_index];
      }
    } else {
      previous_cost = m[_input.jobs[s_route[t_rank - 1]].index()][u_index];
    }

    if (t_rank == s_route.size() - 1) {
      if (v.has_end()) {
        next_cost = m[u_index][v.end.value().index()];
      }
    } else {
      next_cost = m[u_index][_input.jobs[s_route[s_rank + 1]].index()];
    }

    t_gain = -previous_cost - next_cost;
  } else {
    // No common edge so both gains can be computed independently.
    s_gain = _sol_state.node_gains[s_vehicle][s_rank];

    t_gain = -utils::addition_cost(_input, m, _u, v, s_route, t_rank);
  }

  stored_gain = s_gain + t_gain;
  gain_computed = true;
}

bool UnassignedExchange::is_valid() {
  auto pickup = source.pickup_in_range(_first_rank, _last_rank);
  assert(_input.jobs[_removed].pickup <= pickup);
  pickup -= _input.jobs[_removed].pickup;
  pickup += _input.jobs[_u].pickup;

  auto delivery = source.delivery_in_range(_first_rank, _last_rank);
  assert(_input.jobs[_removed].delivery <= delivery);
  delivery -= _input.jobs[_removed].delivery;
  delivery += _input.jobs[_u].delivery;

  bool valid = source.is_valid_addition_for_capacity_margins(_input,
                                                             pickup,
                                                             delivery,
                                                             _first_rank,
                                                             _last_rank);

  valid = valid &&
          source.is_valid_addition_for_capacity_inclusion(_input,
                                                          delivery,
                                                          _moved_jobs.begin(),
                                                          _moved_jobs.end(),
                                                          _first_rank,
                                                          _last_rank);

  return valid;
}

void UnassignedExchange::apply() {
  std::copy(_moved_jobs.begin(),
            _moved_jobs.end(),
            s_route.begin() + _first_rank);

  assert(_unassigned.find(_u) != _unassigned.end());
  _unassigned.erase(_u);
  assert(_unassigned.find(_removed) == _unassigned.end());
  _unassigned.insert(_removed);

  source.update_amounts(_input);
}

std::vector<Index> UnassignedExchange::addition_candidates() const {
  return {s_vehicle};
}

std::vector<Index> UnassignedExchange::update_candidates() const {
  return {s_vehicle};
}

std::vector<Index> UnassignedExchange::required_unassigned() const {
  return {_u};
}

} // namespace cvrp
} // namespace vroom
