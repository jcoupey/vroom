/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <numeric>
#include <unordered_map>

#include "structures/vroom/solution_state.h"
#include "utils/helpers.h"

namespace vroom {
namespace utils {

SolutionState::SolutionState(const Input& input)
  : _input(input),
    _m(_input.get_matrix()),
    _nb_vehicles(_input.vehicles.size()),
    fwd_costs(_nb_vehicles),
    bwd_costs(_nb_vehicles),
    fwd_skill_rank(_nb_vehicles, std::vector<Index>(_nb_vehicles)),
    bwd_skill_rank(_nb_vehicles, std::vector<Index>(_nb_vehicles)),
    edge_costs_around_node(_nb_vehicles),
    node_gains(_nb_vehicles),
    node_candidates(_nb_vehicles),
    edge_costs_around_edge(_nb_vehicles),
    edge_gains(_nb_vehicles),
    edge_candidates(_nb_vehicles),
    pd_gains(_nb_vehicles),
    matching_delivery_rank(_nb_vehicles),
    matching_pickup_rank(_nb_vehicles),
    nearest_job_rank_in_routes_from(_nb_vehicles,
                                    std::vector<std::vector<Index>>(
                                      _nb_vehicles)),
    nearest_job_rank_in_routes_to(_nb_vehicles,
                                  std::vector<std::vector<Index>>(
                                    _nb_vehicles)),
    route_costs(_nb_vehicles) {
}

void SolutionState::setup(const std::vector<Index>& r, Index v) {
  update_costs(r, v);
  update_skills(r, v);
  set_node_gains(r, v);
  set_edge_gains(r, v);
  set_pd_matching_ranks(r, v);
  set_pd_gains(r, v);
#ifndef NDEBUG
  update_route_cost(r, v);
#endif
}

void SolutionState::setup(const RawSolution& sol) {
  for (std::size_t v = 0; v < _nb_vehicles; ++v) {
    setup(sol[v].route, v);
  }

  // Initialize unassigned jobs.
  Index x = 0;
  std::generate_n(std::inserter(unassigned, unassigned.end()),
                  _input.jobs.size(),
                  [&] { return x++; });

  for (const auto& s : sol) {
    for (const auto i : s.route) {
      unassigned.erase(i);
    }
  }
}

void SolutionState::setup(const TWSolution& tw_sol) {
  for (std::size_t v = 0; v < _nb_vehicles; ++v) {
    setup(tw_sol[v].route, v);
  }

  // Initialize unassigned jobs.
  Index x = 0;
  std::generate_n(std::inserter(unassigned, unassigned.end()),
                  _input.jobs.size(),
                  [&] { return x++; });

  for (const auto& tw_r : tw_sol) {
    for (const auto i : tw_r.route) {
      unassigned.erase(i);
    }
  }
}

void SolutionState::update_costs(const std::vector<Index>& route, Index v) {
  fwd_costs[v] = std::vector<Cost>(route.size());
  bwd_costs[v] = std::vector<Cost>(route.size());

  Cost current_fwd = 0;
  Cost current_bwd = 0;

  Index previous_index = 0; // dummy init
  if (!route.empty()) {
    previous_index = _input.jobs[route[0]].index();
    fwd_costs[v][0] = current_fwd;
    bwd_costs[v][0] = current_bwd;
  }

  for (std::size_t i = 1; i < route.size(); ++i) {
    auto current_index = _input.jobs[route[i]].index();
    current_fwd += _m[previous_index][current_index];
    current_bwd += _m[current_index][previous_index];
    fwd_costs[v][i] = current_fwd;
    bwd_costs[v][i] = current_bwd;
    previous_index = current_index;
  }
}

void SolutionState::update_skills(const std::vector<Index>& route, Index v1) {
  for (std::size_t v2 = 0; v2 < _nb_vehicles; ++v2) {
    if (v1 == v2) {
      continue;
    }

    auto fwd = std::find_if_not(route.begin(), route.end(), [&](auto j_rank) {
      return _input.vehicle_ok_with_job(v2, j_rank);
    });
    fwd_skill_rank[v1][v2] = std::distance(route.begin(), fwd);

    auto bwd = std::find_if_not(route.rbegin(), route.rend(), [&](auto j_rank) {
      return _input.vehicle_ok_with_job(v2, j_rank);
    });
    bwd_skill_rank[v1][v2] = route.size() - std::distance(route.rbegin(), bwd);
  }
}

void SolutionState::set_node_gains(const std::vector<Index>& route, Index v) {
  node_gains[v] = std::vector<Gain>(route.size());
  edge_costs_around_node[v] = std::vector<Gain>(route.size());

  if (route.size() == 0) {
    return;
  }

  // Handling first job is special due to potential open tours.
  Index p_index;
  Index c_index = _input.jobs[route[0]].index();
  Index n_index;

  Gain previous_cost = 0;
  Gain next_cost = 0;
  Gain new_edge_cost = 0;

  if (_input.vehicles[v].has_start()) {
    // There is a previous step before job at rank 0.
    p_index = _input.vehicles[v].start.value().index();
    previous_cost = _m[p_index][c_index];

    // Update next_cost with next job or end.
    if (route.size() > 1) {
      n_index = _input.jobs[route[1]].index();
      next_cost = _m[c_index][n_index];
      new_edge_cost = _m[p_index][n_index];
    } else {
      // route.size() is 1 and first job is also the last.
      if (_input.vehicles[v].has_end()) {
        next_cost = _m[c_index][_input.vehicles[v].end.value().index()];
      }
    }
  } else {
    // There is a next cost either to next job or to end of route, but
    // no new edge.
    if (route.size() > 1) {
      n_index = _input.jobs[route[1]].index();
    } else {
      assert(_input.vehicles[v].has_end());
      n_index = _input.vehicles[v].end.value().index();
    }
    next_cost = _m[c_index][n_index];
  }

  Gain edges_costs_around = previous_cost + next_cost;
  edge_costs_around_node[v][0] = edges_costs_around;

  Gain current_gain = edges_costs_around - new_edge_cost;
  node_gains[v][0] = current_gain;
  Gain best_gain = current_gain;
  node_candidates[v] = 0;

  if (route.size() == 1) {
    // No more jobs.
    return;
  }

  // Handle jobs that always have a previous and next job.
  for (std::size_t i = 1; i < route.size() - 1; ++i) {
    // Compute potential gain to relocate current job.
    p_index = _input.jobs[route[i - 1]].index();
    c_index = _input.jobs[route[i]].index();
    n_index = _input.jobs[route[i + 1]].index();

    edges_costs_around = _m[p_index][c_index] + _m[c_index][n_index];
    edge_costs_around_node[v][i] = edges_costs_around;

    current_gain = edges_costs_around - _m[p_index][n_index];
    node_gains[v][i] = current_gain;

    if (current_gain > best_gain) {
      best_gain = current_gain;
      node_candidates[v] = i;
    }
  }

  // Handling last job is special due to potential open tours.
  auto last_rank = route.size() - 1;
  c_index = _input.jobs[route[last_rank]].index();

  previous_cost = 0;
  next_cost = 0;
  new_edge_cost = 0;

  if (_input.vehicles[v].has_end()) {
    // There is a next step after last job.
    n_index = _input.vehicles[v].end.value().index();
    next_cost = _m[c_index][n_index];

    if (route.size() > 1) {
      p_index = _input.jobs[route[last_rank - 1]].index();
      previous_cost = _m[p_index][c_index];
      new_edge_cost = _m[p_index][n_index];
    }
  } else {
    // There is a previous cost either from previous job or from start
    // of route, but no new edge.
    if (route.size() > 1) {
      p_index = _input.jobs[route[last_rank - 1]].index();
    } else {
      assert(_input.vehicles[v].has_start());
      p_index = _input.vehicles[v].start.value().index();
    }
    previous_cost = _m[p_index][c_index];
  }

  edges_costs_around = previous_cost + next_cost;
  edge_costs_around_node[v][last_rank] = edges_costs_around;

  current_gain = edges_costs_around - new_edge_cost;
  node_gains[v][last_rank] = current_gain;

  if (current_gain > best_gain) {
    node_candidates[v] = last_rank;
  }
}

void SolutionState::set_edge_gains(const std::vector<Index>& route, Index v) {
  std::size_t nb_edges = (route.size() < 2) ? 0 : route.size() - 1;

  edge_gains[v] = std::vector<Gain>(nb_edges);
  edge_costs_around_edge[v] = std::vector<Gain>(nb_edges);

  if (route.size() < 2) {
    return;
  }

  // Handling first edge is special due to potential open tours.
  Index p_index;
  Index c_index = _input.jobs[route[0]].index();
  Index after_c_index = _input.jobs[route[1]].index();
  Index n_index;

  Gain previous_cost = 0;
  Gain next_cost = 0;
  Gain new_edge_cost = 0;

  if (_input.vehicles[v].has_start()) {
    // There is a previous step before job at rank 0.
    p_index = _input.vehicles[v].start.value().index();
    previous_cost = _m[p_index][c_index];

    // Update next_cost with next job or end.
    if (route.size() > 2) {
      n_index = _input.jobs[route[2]].index();
      next_cost = _m[after_c_index][n_index];
      new_edge_cost = _m[p_index][n_index];
    } else {
      // route.size() is 2 and first edge is also the last.
      if (_input.vehicles[v].has_end()) {
        next_cost = _m[after_c_index][_input.vehicles[v].end.value().index()];
      }
    }
  } else {
    // There is a next cost either to next job or to end of route, but
    // no new edge.
    if (route.size() > 2) {
      n_index = _input.jobs[route[2]].index();
    } else {
      assert(_input.vehicles[v].has_end());
      n_index = _input.vehicles[v].end.value().index();
    }
    next_cost = _m[after_c_index][n_index];
  }

  Gain edges_costs_around = previous_cost + next_cost;
  edge_costs_around_edge[v][0] = edges_costs_around;

  Gain current_gain = edges_costs_around - new_edge_cost;
  edge_gains[v][0] = current_gain;
  Gain best_gain = current_gain;
  edge_candidates[v] = 0;

  if (route.size() == 2) {
    // No more edges.
    return;
  }

  // Handle edges that always have a previous and next job.
  for (std::size_t i = 1; i < nb_edges - 1; ++i) {
    // Compute potential gain to relocate edge from current to next
    // job.
    p_index = _input.jobs[route[i - 1]].index();
    c_index = _input.jobs[route[i]].index();
    after_c_index = _input.jobs[route[i + 1]].index();
    n_index = _input.jobs[route[i + 2]].index();

    edges_costs_around = _m[p_index][c_index] + _m[after_c_index][n_index];
    edge_costs_around_edge[v][i] = edges_costs_around;

    current_gain = edges_costs_around - _m[p_index][n_index];
    edge_gains[v][i] = current_gain;

    if (current_gain > best_gain) {
      best_gain = current_gain;
      edge_candidates[v] = i;
    }
  }

  // Handling last edge is special due to potential open tours.
  auto last_edge_rank = nb_edges - 1;
  c_index = _input.jobs[route[last_edge_rank]].index();
  after_c_index = _input.jobs[route[last_edge_rank + 1]].index();

  previous_cost = 0;
  next_cost = 0;
  new_edge_cost = 0;

  if (_input.vehicles[v].has_end()) {
    // There is a next step after last job.
    n_index = _input.vehicles[v].end.value().index();
    next_cost = _m[after_c_index][n_index];

    if (route.size() > 2) {
      p_index = _input.jobs[route[last_edge_rank - 1]].index();
      previous_cost = _m[p_index][c_index];
      new_edge_cost = _m[p_index][n_index];
    }
  } else {
    // There is a previous cost either from previous job or from start
    // of route, but no new edge.
    if (route.size() > 2) {
      p_index = _input.jobs[route[last_edge_rank - 1]].index();
    } else {
      assert(_input.vehicles[v].has_start());
      p_index = _input.vehicles[v].start.value().index();
    }
    previous_cost = _m[p_index][c_index];
  }

  edges_costs_around = previous_cost + next_cost;
  edge_costs_around_edge[v][last_edge_rank] = edges_costs_around;

  current_gain = edges_costs_around - new_edge_cost;
  edge_gains[v][last_edge_rank] = current_gain;

  if (current_gain > best_gain) {
    edge_candidates[v] = last_edge_rank;
  }
}

void SolutionState::set_pd_gains(const std::vector<Index>& route, Index v) {
  // Expects to have valid values in node_gains, so should be run
  // after set_node_gains. Expects to have valid values in
  // matching_delivery_rank, so should be run after
  // set_pd_matching_ranks.
  pd_gains[v] = std::vector<Gain>(route.size());

  for (std::size_t pickup_rank = 0; pickup_rank < route.size(); ++pickup_rank) {
    if (_input.jobs[route[pickup_rank]].type != JOB_TYPE::PICKUP) {
      continue;
    }
    Index pickup_index = _input.jobs[route[pickup_rank]].index();
    unsigned delivery_rank = matching_delivery_rank[v][pickup_rank];
    Index delivery_index = _input.jobs[route[delivery_rank]].index();

    if (pickup_rank + 1 == delivery_rank) {
      // Pickup and delivery in a row.
      Gain previous_cost = 0;
      Gain next_cost = 0;
      Gain new_edge_cost = 0;
      Index p_index;
      Index n_index;

      // Compute cost for step before pickup.
      bool has_previous_step = false;
      if (pickup_rank > 0) {
        has_previous_step = true;
        p_index = _input.jobs[route[pickup_rank - 1]].index();
        previous_cost = _m[p_index][pickup_index];
      } else {
        if (_input.vehicles[v].has_start()) {
          has_previous_step = true;
          p_index = _input.vehicles[v].start.value().index();
          previous_cost = _m[p_index][pickup_index];
        }
      }

      // Compute cost for step after delivery.
      bool has_next_step = false;
      if (delivery_rank < route.size() - 1) {
        has_next_step = true;
        n_index = _input.jobs[route[delivery_rank + 1]].index();
        next_cost = _m[delivery_index][n_index];
      } else {
        if (_input.vehicles[v].has_end()) {
          has_next_step = true;
          n_index = _input.vehicles[v].end.value().index();
          next_cost = _m[delivery_index][n_index];
        }
      }

      if (has_previous_step and has_next_step and (route.size() > 2)) {
        // No new edge with an open trip or if removing P&D creates an
        // empty route.
        new_edge_cost = _m[p_index][n_index];
      }

      pd_gains[v][pickup_rank] = previous_cost +
                                 _m[pickup_index][delivery_index] + next_cost -
                                 new_edge_cost;
    } else {
      // Simply add both gains as neighbouring edges are disjoint.
      pd_gains[v][pickup_rank] =
        node_gains[v][pickup_rank] + node_gains[v][delivery_rank];
    }
  }
}

void SolutionState::set_pd_matching_ranks(const std::vector<Index>& route,
                                          Index v) {
  matching_delivery_rank[v] = std::vector<Index>(route.size());
  matching_pickup_rank[v] = std::vector<Index>(route.size());

  std::unordered_map<Index, Index> pickup_route_rank_to_input_rank;
  std::unordered_map<Index, Index> delivery_input_rank_to_route_rank;

  for (std::size_t i = 0; i < route.size(); ++i) {
    switch (_input.jobs[route[i]].type) {
    case JOB_TYPE::SINGLE:
      break;
    case JOB_TYPE::PICKUP:
      pickup_route_rank_to_input_rank.insert({i, route[i]});
      break;
    case JOB_TYPE::DELIVERY:
      delivery_input_rank_to_route_rank.insert({route[i], i});
      break;
    }
  }

  assert(pickup_route_rank_to_input_rank.size() ==
         delivery_input_rank_to_route_rank.size());
  for (const auto& pair : pickup_route_rank_to_input_rank) {
    // Relies of the fact that associated pickup and delivery are
    // stored sequentially in input jobs vector.
    auto pickup_route_rank = pair.first;
    auto delivery_input_rank = pair.second + 1;
    auto search = delivery_input_rank_to_route_rank.find(delivery_input_rank);
    assert(search != delivery_input_rank_to_route_rank.end());
    auto delivery_route_rank = search->second;

    matching_delivery_rank[v][pickup_route_rank] = delivery_route_rank;
    matching_pickup_rank[v][delivery_route_rank] = pickup_route_rank;
  }
}

void SolutionState::update_nearest_job_rank_in_routes(
  const std::vector<Index>& route_1,
  const std::vector<Index>& route_2,
  Index v1,
  Index v2) {
  nearest_job_rank_in_routes_from[v1][v2].assign(route_1.size(), 0);
  nearest_job_rank_in_routes_to[v1][v2].assign(route_1.size(), 0);

  for (std::size_t r1 = 0; r1 < route_1.size(); ++r1) {
    Index index_r1 = _input.jobs[route_1[r1]].index();

    auto min_from = std::numeric_limits<Cost>::max();
    auto min_to = std::numeric_limits<Cost>::max();
    Index best_from_rank = 0;
    Index best_to_rank = 0;

    for (std::size_t r2 = 0; r2 < route_2.size(); ++r2) {
      Index index_r2 = _input.jobs[route_2[r2]].index();
      if (_m[index_r1][index_r2] < min_from) {
        min_from = _m[index_r1][index_r2];
        best_from_rank = r2;
      }
      if (_m[index_r2][index_r1] < min_to) {
        min_to = _m[index_r2][index_r1];
        best_to_rank = r2;
      }
    }

    nearest_job_rank_in_routes_from[v1][v2][r1] = best_from_rank;
    nearest_job_rank_in_routes_to[v1][v2][r1] = best_to_rank;
  }
}

void SolutionState::update_route_cost(const std::vector<Index>& route,
                                      Index v) {
  route_costs[v] = route_cost_for_vehicle(_input, v, route);
}

} // namespace utils
} // namespace vroom
