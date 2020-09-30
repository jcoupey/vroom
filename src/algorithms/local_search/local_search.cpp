/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <numeric>

#include "algorithms/local_search/local_search.h"
#include "algorithms/local_search/operator.h"
#include "problems/cvrp/operators/cross_exchange.h"
#include "problems/cvrp/operators/exchange.h"
#include "problems/cvrp/operators/intra_cross_exchange.h"
#include "problems/cvrp/operators/intra_exchange.h"
#include "problems/cvrp/operators/intra_mixed_exchange.h"
#include "problems/cvrp/operators/intra_or_opt.h"
#include "problems/cvrp/operators/intra_relocate.h"
#include "problems/cvrp/operators/mixed_exchange.h"
#include "problems/cvrp/operators/or_opt.h"
#include "problems/cvrp/operators/pd_shift.h"
#include "problems/cvrp/operators/relocate.h"
#include "problems/cvrp/operators/reverse_two_opt.h"
#include "problems/cvrp/operators/route_exchange.h"
#include "problems/cvrp/operators/two_opt.h"
#include "problems/cvrp/operators/unassigned_exchange.h"
#include "problems/vrptw/operators/cross_exchange.h"
#include "problems/vrptw/operators/exchange.h"
#include "problems/vrptw/operators/intra_cross_exchange.h"
#include "problems/vrptw/operators/intra_exchange.h"
#include "problems/vrptw/operators/intra_mixed_exchange.h"
#include "problems/vrptw/operators/intra_or_opt.h"
#include "problems/vrptw/operators/intra_relocate.h"
#include "problems/vrptw/operators/mixed_exchange.h"
#include "problems/vrptw/operators/or_opt.h"
#include "problems/vrptw/operators/pd_shift.h"
#include "problems/vrptw/operators/relocate.h"
#include "problems/vrptw/operators/reverse_two_opt.h"
#include "problems/vrptw/operators/route_exchange.h"
#include "problems/vrptw/operators/two_opt.h"
#include "problems/vrptw/operators/unassigned_exchange.h"
#include "utils/helpers.h"

#include <string>
#include <iostream>

namespace vroom {
namespace ls {

template <class Route,
          class UnassignedExchange,
          class Exchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class PDShift,
          class RouteExchange>
LocalSearch<Route,
            UnassignedExchange,
            Exchange,
            CrossExchange,
            MixedExchange,
            TwoOpt,
            ReverseTwoOpt,
            Relocate,
            OrOpt,
            IntraExchange,
            IntraCrossExchange,
            IntraMixedExchange,
            IntraRelocate,
            IntraOrOpt,
            PDShift,
            RouteExchange>::LocalSearch(const Input& input,
                                        std::vector<Route>& sol,
                                        unsigned max_nb_jobs_removal)
  : _input(input),
    _matrix(_input.get_matrix()),
    _nb_vehicles(_input.vehicles.size()),
    _max_nb_jobs_removal(max_nb_jobs_removal),
    _all_routes(_nb_vehicles),
    _sol_state(input),
    _sol(sol),
    _best_sol(sol) {
  // Initialize all route indices.
  std::iota(_all_routes.begin(), _all_routes.end(), 0);

  // Setup solution state.
  _sol_state.setup(_sol);

  Index v_rank = 0;
  _best_sol_indicators.priority_sum =
    std::accumulate(_sol.begin(), _sol.end(), 0, [&](auto sum, const auto& r) {
      return sum + utils::priority_sum_for_route(_input, r.route, v_rank++);
    });

  _best_sol_indicators.unassigned = _sol_state.unassigned.size();

  v_rank = 0;
  _best_sol_indicators.cost =
    std::accumulate(_sol.begin(), _sol.end(), 0, [&](auto sum, const auto& r) {
      return sum + utils::route_cost_for_vehicle(_input, v_rank++, r.route);
    });

  _best_sol_indicators.used_vehicles =
    std::count_if(_sol.begin(), _sol.end(), [](const auto& r) {
      return !r.empty();
    });
}

template <class Route,
          class UnassignedExchange,
          class Exchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class PDShift,
          class RouteExchange>
void LocalSearch<Route,
                 UnassignedExchange,
                 Exchange,
                 CrossExchange,
                 MixedExchange,
                 TwoOpt,
                 ReverseTwoOpt,
                 Relocate,
                 OrOpt,
                 IntraExchange,
                 IntraCrossExchange,
                 IntraMixedExchange,
                 IntraRelocate,
                 IntraOrOpt,
                 PDShift,
                 RouteExchange>::try_job_additions(const std::vector<Index>&
                                                     routes,
                                                   double regret_coeff) {
  bool job_added;
  std::vector<Gain> best_costs;
  std::vector<Index> best_ranks;
  std::vector<Index> best_pickup_ranks;
  std::vector<Index> best_delivery_ranks;

  do {
    Priority best_priority = 0;
    double best_cost = std::numeric_limits<double>::max();
    Index best_job_rank = 0;
    Index best_route = 0;
    Index best_rank = 0;
    Index best_pickup_r = 0;
    Index best_delivery_r = 0;

    for (const auto j : _sol_state.unassigned) {
      const auto& current_job = _input.jobs[j];
      if (current_job.type == JOB_TYPE::DELIVERY) {
        continue;
      }

      const auto job_priorities = current_job.priorities;
      const auto job_max_priority = current_job.max_priority;
      if (job_max_priority < best_priority) {
        // Insert higher priority jobs first.
        continue;
      }

      best_costs.assign(routes.size(), std::numeric_limits<Gain>::max());
      best_ranks.assign(routes.size(), 0);
      best_pickup_ranks.assign(routes.size(), 0);
      best_delivery_ranks.assign(routes.size(), 0);

      if (current_job.type == JOB_TYPE::SINGLE) {
        for (std::size_t i = 0; i < routes.size(); ++i) {
          const auto v = routes[i];
          const auto& v_target = _input.vehicles[v];

          if (!_input.vehicle_ok_with_job(v, j)) {
            continue;
          }

          for (std::size_t r = 0; r <= _sol[v].size(); ++r) {
            Gain current_cost = utils::addition_cost(_input,
                                                     _matrix,
                                                     j,
                                                     v_target,
                                                     _sol[v].route,
                                                     r);
            if (current_cost < best_costs[i] and
                _sol[v].is_valid_addition_for_capacity(_input,
                                                       current_job.pickup,
                                                       current_job.delivery,
                                                       r) and
                _sol[v].is_valid_addition_for_tw(_input, j, r)) {
              best_costs[i] = current_cost;
              best_ranks[i] = r;
            }
          }
        }
      }

      if (current_job.type == JOB_TYPE::PICKUP) {
        for (std::size_t i = 0; i < routes.size(); ++i) {
          const auto v = routes[i];
          const auto& v_target = _input.vehicles[v];

          if (!_input.vehicle_ok_with_job(v, j)) {
            continue;
          }

          // Pre-compute cost of addition for matching delivery.
          std::vector<Gain> d_adds(_sol[v].route.size() + 1);
          std::vector<unsigned char> valid_delivery_insertions(
            _sol[v].route.size() + 1);

          for (unsigned d_rank = 0; d_rank <= _sol[v].route.size(); ++d_rank) {
            d_adds[d_rank] = utils::addition_cost(_input,
                                                  _matrix,
                                                  j + 1,
                                                  v_target,
                                                  _sol[v].route,
                                                  d_rank);
            valid_delivery_insertions[d_rank] =
              _sol[v].is_valid_addition_for_tw(_input, j + 1, d_rank);
          }

          for (Index pickup_r = 0; pickup_r <= _sol[v].size(); ++pickup_r) {
            Gain p_add = utils::addition_cost(_input,
                                              _matrix,
                                              j,
                                              v_target,
                                              _sol[v].route,
                                              pickup_r);

            if (!_sol[v].is_valid_addition_for_load(_input,
                                                    current_job.pickup,
                                                    pickup_r) or
                !_sol[v].is_valid_addition_for_tw(_input, j, pickup_r)) {
              continue;
            }

            // Build replacement sequence for current insertion.
            std::vector<Index> modified_with_pd({j});
            Amount modified_delivery = _input.zero_amount();

            for (Index delivery_r = pickup_r; delivery_r <= _sol[v].size();
                 ++delivery_r) {
              // Update state variables along the way before potential
              // early abort.
              if (pickup_r < delivery_r) {
                modified_with_pd.push_back(_sol[v].route[delivery_r - 1]);
                const auto& new_modified_job =
                  _input.jobs[_sol[v].route[delivery_r - 1]];
                if (new_modified_job.type == JOB_TYPE::SINGLE) {
                  modified_delivery += new_modified_job.delivery;
                }
              }

              if (!(bool)valid_delivery_insertions[delivery_r]) {
                continue;
              }

              Gain pd_cost;
              if (pickup_r == delivery_r) {
                pd_cost = utils::addition_cost(_input,
                                               _matrix,
                                               j,
                                               v_target,
                                               _sol[v].route,
                                               pickup_r,
                                               pickup_r + 1);
              } else {
                pd_cost = p_add + d_adds[delivery_r];
              }

              // Normalize cost per job for consistency with single jobs.
              Gain current_cost =
                static_cast<Gain>(static_cast<double>(pd_cost) / 2);

              if (current_cost < best_costs[i]) {
                modified_with_pd.push_back(j + 1);

                // Update best cost depending on validity.
                bool is_valid =
                  _sol[v]
                    .is_valid_addition_for_capacity_inclusion(_input,
                                                              modified_delivery,
                                                              modified_with_pd
                                                                .begin(),
                                                              modified_with_pd
                                                                .end(),
                                                              pickup_r,
                                                              delivery_r);

                is_valid =
                  is_valid &&
                  _sol[v].is_valid_addition_for_tw(_input,
                                                   modified_with_pd.begin(),
                                                   modified_with_pd.end(),
                                                   pickup_r,
                                                   delivery_r);

                modified_with_pd.pop_back();

                if (is_valid) {
                  best_costs[i] = current_cost;
                  best_pickup_ranks[i] = pickup_r;
                  best_delivery_ranks[i] = delivery_r;
                }
              }
            }
          }
        }
      }

      auto smallest = std::numeric_limits<Gain>::max();
      auto second_smallest = std::numeric_limits<Gain>::max();
      std::size_t smallest_idx = std::numeric_limits<std::size_t>::max();

      for (std::size_t i = 0; i < routes.size(); ++i) {
        if (best_costs[i] < smallest) {
          smallest_idx = i;
          second_smallest = smallest;
          smallest = best_costs[i];
        } else if (best_costs[i] < second_smallest) {
          second_smallest = best_costs[i];
        }
      }

      // Find best route for current job based on cost of addition and
      // regret cost of not adding.
      for (std::size_t i = 0; i < routes.size(); ++i) {
        const auto addition_cost = best_costs[i];
        if (addition_cost == std::numeric_limits<Gain>::max()) {
          continue;
        }
        const Gain regret_cost =
          (i == smallest_idx) ? second_smallest : smallest;

        const double eval = static_cast<double>(addition_cost) -
                            regret_coeff * static_cast<double>(regret_cost);

        if ((job_max_priority > best_priority) or
            (job_max_priority == best_priority and eval < best_cost)) {
          best_priority = job_max_priority;
          best_cost = eval;
          best_job_rank = j;
          best_route = routes[i];
          if (current_job.type == JOB_TYPE::SINGLE) {
            best_rank = best_ranks[i];
          }
          if (current_job.type == JOB_TYPE::PICKUP) {
            best_pickup_r = best_pickup_ranks[i];
            best_delivery_r = best_delivery_ranks[i];
          }
        }
      }
    }

    job_added = (best_cost < std::numeric_limits<double>::max());

    if (job_added) {
      _sol_state.unassigned.erase(best_job_rank);
      const auto& best_job = _input.jobs[best_job_rank];

      if (best_job.type == JOB_TYPE::SINGLE) {
        _sol[best_route].add(_input, best_job_rank, best_rank);
      } else {
        assert(best_job.type == JOB_TYPE::PICKUP);

        std::vector<Index> modified_with_pd({best_job_rank});
        std::copy(_sol[best_route].route.begin() + best_pickup_r,
                  _sol[best_route].route.begin() + best_delivery_r,
                  std::back_inserter(modified_with_pd));
        modified_with_pd.push_back(best_job_rank + 1);

        _sol[best_route].replace(_input,
                                 modified_with_pd.begin(),
                                 modified_with_pd.end(),
                                 best_pickup_r,
                                 best_delivery_r);

        assert(_sol_state.unassigned.find(best_job_rank + 1) !=
               _sol_state.unassigned.end());
        _sol_state.unassigned.erase(best_job_rank + 1);
      }

#ifndef NDEBUG
      // Update cost after addition.
      _sol_state.update_route_cost(_sol[best_route].route, best_route);
#endif
    }
  } while (job_added);
}

template <class Route,
          class UnassignedExchange,
          class Exchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class PDShift,
          class RouteExchange>
void LocalSearch<Route,
                 UnassignedExchange,
                 Exchange,
                 CrossExchange,
                 MixedExchange,
                 TwoOpt,
                 ReverseTwoOpt,
                 Relocate,
                 OrOpt,
                 IntraExchange,
                 IntraCrossExchange,
                 IntraMixedExchange,
                 IntraRelocate,
                 IntraOrOpt,
                 PDShift,
                 RouteExchange>::run_ls_step() {
  // Store best move involving a pair of routes.
  std::vector<std::vector<std::unique_ptr<Operator>>> best_ops(_nb_vehicles);
  for (std::size_t v = 0; v < _nb_vehicles; ++v) {
    best_ops[v] = std::vector<std::unique_ptr<Operator>>(_nb_vehicles);
  }

  // List of source/target pairs we need to test (all related vehicles
  // at first).
  std::vector<std::pair<Index, Index>> s_t_pairs;
  for (unsigned s_v = 0; s_v < _nb_vehicles; ++s_v) {
    for (unsigned t_v = 0; t_v < _nb_vehicles; ++t_v) {
      if (_input.vehicle_ok_with_vehicle(s_v, t_v)) {
        s_t_pairs.emplace_back(s_v, t_v);
      }
    }
  }

  // Store best gain for matching move.
  std::vector<std::vector<Gain>>
    best_gains(_nb_vehicles,
               std::vector<Gain>(_nb_vehicles,
                                 std::numeric_limits<Gain>::min()));

  // Store best priority increase for matching move. Only operators
  // involving a single route and unassigned jobs can change overall
  // priority (currently only UnassignedExchange).
  std::vector<Priority> best_priorities(_nb_vehicles, 0);

  Gain best_gain = 1;
  Priority best_priority = 0;

  while (best_gain > 0 or best_priority > 0) {
    // Operators applied to a pair of (different) routes.

    if (_input.has_jobs()) {
      // Move(s) that don't make sense for shipment-only instances.

      // Unassigned-exchange stuff
      for (const Index u : _sol_state.unassigned) {
        if (_input.jobs[u].type != JOB_TYPE::SINGLE) {
          continue;
        }

        Priorities u_priorities = _input.jobs[u].priorities;
        for (const auto& s_t : s_t_pairs) {
          if (s_t.first != s_t.second or
              !_input.vehicle_ok_with_job(s_t.first, u) or
              _sol[s_t.first].empty()) {
            continue;
          }

          for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
            const auto& current_job =
              _input.jobs[_sol[s_t.first].route[s_rank]];
            if (current_job.type != JOB_TYPE::SINGLE or
                u_priorities.at(s_t.first) < current_job.priorities.at(s_t.first)) {
              continue;
            }

            const Priority priority_gain = u_priorities.at(s_t.first) - current_job.priorities.at(s_t.first);

            if (best_priorities[s_t.first] <= priority_gain) {
              for (unsigned t_rank = 0; t_rank <= _sol[s_t.first].size();
                   ++t_rank) {
                if (t_rank == s_rank + 1) {
                  // Same move as with t_rank == s_rank.
                  continue;
                }
                UnassignedExchange r(_input,
                                     _sol_state,
                                     _sol_state.unassigned,
                                     _sol[s_t.first],
                                     s_t.first,
                                     s_rank,
                                     t_rank,
                                     u);

                bool better_if_valid =
                  (best_priorities[s_t.first] < priority_gain) or
                  (best_priorities[s_t.first] == priority_gain and
                   r.gain() > best_gains[s_t.first][s_t.first]);

                if (better_if_valid and r.is_valid()) {
                  best_priorities[s_t.first] = priority_gain;
                  best_gains[s_t.first][s_t.first] = r.gain();
                  best_ops[s_t.first][s_t.first] =
                    std::make_unique<UnassignedExchange>(r);
                }
              }
            }
          }
        }
      }

      // Exchange stuff
      for (const auto& s_t : s_t_pairs) {
        if (s_t.second <= s_t.first or // This operator is symmetric.
            best_priorities[s_t.first] > 0 or best_priorities[s_t.second] > 0 or
            _sol[s_t.first].size() == 0 or _sol[s_t.second].size() == 0) {
          continue;
        }

        for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
          const auto& s_job_rank = _sol[s_t.first].route[s_rank];
          if (_input.jobs[s_job_rank].type != JOB_TYPE::SINGLE or
              !_input.vehicle_ok_with_job(s_t.second, s_job_rank)) {
            // Don't try moving (part of) a shipment or an
            // incompatible job.
            continue;
          }

          for (unsigned t_rank = 0; t_rank < _sol[s_t.second].size();
               ++t_rank) {
            const auto& t_job_rank = _sol[s_t.second].route[t_rank];
            if (_input.jobs[t_job_rank].type != JOB_TYPE::SINGLE or
                !_input.vehicle_ok_with_job(s_t.first, t_job_rank)) {
              // Don't try moving (part of) a shipment or an
              // incompatible job.
              continue;
            }

            Exchange r(_input,
                       _sol_state,
                       _sol[s_t.first],
                       s_t.first,
                       s_rank,
                       _sol[s_t.second],
                       s_t.second,
                       t_rank);

            if (r.gain() > best_gains[s_t.first][s_t.second] and r.is_valid()) {
              best_gains[s_t.first][s_t.second] = r.gain();
              best_ops[s_t.first][s_t.second] = std::make_unique<Exchange>(r);
            }
          }
        }
      }
    }

    // CROSS-exchange stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.second <= s_t.first or // This operator is symmetric.
          best_priorities[s_t.first] > 0 or best_priorities[s_t.second] > 0 or
          _sol[s_t.first].size() < 2 or _sol[s_t.second].size() < 2) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 1; ++s_rank) {
        if (!_input.vehicle_ok_with_job(s_t.second,
                                        _sol[s_t.first].route[s_rank]) or
            !_input.vehicle_ok_with_job(s_t.second,
                                        _sol[s_t.first].route[s_rank + 1])) {
          continue;
        }

        const auto& job_s_type =
          _input.jobs[_sol[s_t.first].route[s_rank]].type;

        bool both_s_single =
          (job_s_type == JOB_TYPE::SINGLE) and
          (_input.jobs[_sol[s_t.first].route[s_rank + 1]].type ==
           JOB_TYPE::SINGLE);

        bool is_s_pickup =
          (job_s_type == JOB_TYPE::PICKUP) and
          (_sol_state.matching_delivery_rank[s_t.first][s_rank] == s_rank + 1);

        if (!both_s_single and !is_s_pickup) {
          continue;
        }

        for (unsigned t_rank = 0; t_rank < _sol[s_t.second].size() - 1;
             ++t_rank) {
          if (!_input.vehicle_ok_with_job(s_t.first,
                                          _sol[s_t.second].route[t_rank]) or
              !_input.vehicle_ok_with_job(s_t.first,
                                          _sol[s_t.second].route[t_rank + 1])) {
            continue;
          }

          const auto& job_t_type =
            _input.jobs[_sol[s_t.second].route[t_rank]].type;

          bool both_t_single =
            (job_t_type == JOB_TYPE::SINGLE) and
            (_input.jobs[_sol[s_t.second].route[t_rank + 1]].type ==
             JOB_TYPE::SINGLE);

          bool is_t_pickup =
            (job_t_type == JOB_TYPE::PICKUP) and
            (_sol_state.matching_delivery_rank[s_t.second][t_rank] ==
             t_rank + 1);

          if (!both_t_single and !is_t_pickup) {
            continue;
          }

          CrossExchange r(_input,
                          _sol_state,
                          _sol[s_t.first],
                          s_t.first,
                          s_rank,
                          _sol[s_t.second],
                          s_t.second,
                          t_rank,
                          !is_s_pickup,
                          !is_t_pickup);

          auto& current_best = best_gains[s_t.first][s_t.second];
          if (r.gain_upper_bound() > current_best and r.is_valid() and
              r.gain() > current_best) {
            current_best = r.gain();
            best_ops[s_t.first][s_t.second] =
              std::make_unique<CrossExchange>(r);
          }
        }
      }
    }

    if (_input.has_jobs()) {
      // Mixed-exchange stuff
      for (const auto& s_t : s_t_pairs) {
        if (s_t.first == s_t.second or best_priorities[s_t.first] > 0 or
            best_priorities[s_t.second] > 0 or _sol[s_t.first].size() == 0 or
            _sol[s_t.second].size() < 2) {
          continue;
        }

        for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
          const auto& s_job_rank = _sol[s_t.first].route[s_rank];
          if (_input.jobs[s_job_rank].type != JOB_TYPE::SINGLE or
              !_input.vehicle_ok_with_job(s_t.second, s_job_rank)) {
            // Don't try moving part of a shipment or an incompatible
            // job.
            continue;
          }

          for (unsigned t_rank = 0; t_rank < _sol[s_t.second].size() - 1;
               ++t_rank) {
            if (!_input.vehicle_ok_with_job(s_t.first,
                                            _sol[s_t.second].route[t_rank]) or
                !_input
                   .vehicle_ok_with_job(s_t.first,
                                        _sol[s_t.second].route[t_rank + 1])) {
              continue;
            }

            const auto& job_t_type =
              _input.jobs[_sol[s_t.second].route[t_rank]].type;

            bool both_t_single =
              (job_t_type == JOB_TYPE::SINGLE) and
              (_input.jobs[_sol[s_t.second].route[t_rank + 1]].type ==
               JOB_TYPE::SINGLE);

            bool is_t_pickup =
              (job_t_type == JOB_TYPE::PICKUP) and
              (_sol_state.matching_delivery_rank[s_t.second][t_rank] ==
               t_rank + 1);

            if (!both_t_single and !is_t_pickup) {
              continue;
            }

            MixedExchange r(_input,
                            _sol_state,
                            _sol[s_t.first],
                            s_t.first,
                            s_rank,
                            _sol[s_t.second],
                            s_t.second,
                            t_rank,
                            !is_t_pickup);

            auto& current_best = best_gains[s_t.first][s_t.second];
            if (r.gain_upper_bound() > current_best and r.is_valid() and
                r.gain() > current_best) {
              current_best = r.gain();
              best_ops[s_t.first][s_t.second] =
                std::make_unique<MixedExchange>(r);
            }
          }
        }
      }
    }

    // 2-opt* stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.second <= s_t.first or // This operator is symmetric.
          best_priorities[s_t.first] > 0 or best_priorities[s_t.second] > 0) {
        continue;
      }

      // Determine first ranks for inner loops based on vehicles/jobs
      // compatibility along the routes.
      unsigned first_s_rank = 0;
      const auto first_s_candidate =
        _sol_state.bwd_skill_rank[s_t.first][s_t.second];
      if (first_s_candidate > 0) {
        first_s_rank = first_s_candidate - 1;
      }

      int first_t_rank = 0;
      const auto first_t_candidate =
        _sol_state.bwd_skill_rank[s_t.second][s_t.first];
      if (first_t_candidate > 0) {
        first_t_rank = first_t_candidate - 1;
      }

      for (unsigned s_rank = first_s_rank; s_rank < _sol[s_t.first].size();
           ++s_rank) {
        if (_sol[s_t.first].has_pending_delivery_after_rank(s_rank)) {
          continue;
        }

        for (int t_rank = _sol[s_t.second].size() - 1; t_rank >= first_t_rank;
             --t_rank) {
          if (_sol[s_t.second].has_pending_delivery_after_rank(t_rank)) {
            continue;
          }

          TwoOpt r(_input,
                   _sol_state,
                   _sol[s_t.first],
                   s_t.first,
                   s_rank,
                   _sol[s_t.second],
                   s_t.second,
                   t_rank);

          if (r.gain() > best_gains[s_t.first][s_t.second] and r.is_valid()) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] = std::make_unique<TwoOpt>(r);
          }
        }
      }
    }

    // Reverse 2-opt* stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.first == s_t.second or best_priorities[s_t.first] > 0 or
          best_priorities[s_t.second] > 0) {
        continue;
      }

      // Determine first rank for inner loop based on vehicles/jobs
      // compatibility along the routes.
      unsigned first_s_rank = 0;
      const auto first_s_candidate =
        _sol_state.bwd_skill_rank[s_t.first][s_t.second];
      if (first_s_candidate > 0) {
        first_s_rank = first_s_candidate - 1;
      }

      for (unsigned s_rank = first_s_rank; s_rank < _sol[s_t.first].size();
           ++s_rank) {
        if (_sol[s_t.first].has_delivery_after_rank(s_rank)) {
          continue;
        }

        for (unsigned t_rank = 0;
             t_rank < _sol_state.fwd_skill_rank[s_t.second][s_t.first];
             ++t_rank) {
          if (_sol[s_t.second].has_pickup_up_to_rank(t_rank)) {
            continue;
          }

          ReverseTwoOpt r(_input,
                          _sol_state,
                          _sol[s_t.first],
                          s_t.first,
                          s_rank,
                          _sol[s_t.second],
                          s_t.second,
                          t_rank);

          if (r.gain() > best_gains[s_t.first][s_t.second] and r.is_valid()) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] =
              std::make_unique<ReverseTwoOpt>(r);
          }
        }
      }
    }

    if (_input.has_jobs()) {
      // Move(s) that don't make sense for shipment-only instances.

      // Relocate stuff
      for (const auto& s_t : s_t_pairs) {
        if (s_t.first == s_t.second or best_priorities[s_t.first] > 0 or
            best_priorities[s_t.second] > 0 or _sol[s_t.first].size() == 0) {
          continue;
        }

        for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
          if (_sol_state.node_gains[s_t.first][s_rank] <=
              best_gains[s_t.first][s_t.second]) {
            // Except if addition cost in route s_t.second is negative
            // (!!), overall gain can't exceed current known best gain.
            continue;
          }

          const auto& s_job_rank = _sol[s_t.first].route[s_rank];
          if (_input.jobs[s_job_rank].type != JOB_TYPE::SINGLE or
              !_input.vehicle_ok_with_job(s_t.second, s_job_rank)) {
            // Don't try moving (part of) a shipment or an
            // incompatible job.
            continue;
          }

          for (unsigned t_rank = 0; t_rank <= _sol[s_t.second].size();
               ++t_rank) {
            Relocate r(_input,
                       _sol_state,
                       _sol[s_t.first],
                       s_t.first,
                       s_rank,
                       _sol[s_t.second],
                       s_t.second,
                       t_rank);

            if (r.gain() > best_gains[s_t.first][s_t.second] and r.is_valid()) {
              best_gains[s_t.first][s_t.second] = r.gain();
              best_ops[s_t.first][s_t.second] = std::make_unique<Relocate>(r);
            }
          }
        }
      }

      // Or-opt stuff
      for (const auto& s_t : s_t_pairs) {
        if (s_t.first == s_t.second or best_priorities[s_t.first] > 0 or
            best_priorities[s_t.second] > 0 or _sol[s_t.first].size() < 2) {
          continue;
        }

        for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 1;
             ++s_rank) {
          if (_sol_state.edge_gains[s_t.first][s_rank] <=
              best_gains[s_t.first][s_t.second]) {
            // Except if addition cost in route s_t.second is negative
            // (!!), overall gain can't exceed current known best gain.
            continue;
          }

          if (!_input.vehicle_ok_with_job(s_t.second,
                                          _sol[s_t.first].route[s_rank]) or
              !_input.vehicle_ok_with_job(s_t.second,
                                          _sol[s_t.first].route[s_rank + 1])) {
            continue;
          }

          if (_input.jobs[_sol[s_t.first].route[s_rank]].type !=
                JOB_TYPE::SINGLE or
              _input.jobs[_sol[s_t.first].route[s_rank + 1]].type !=
                JOB_TYPE::SINGLE) {
            // Don't try moving part of a shipment. Moving a full
            // shipment as an edge is not tested because it's a
            // special case of PDShift.
            continue;
          }

          for (unsigned t_rank = 0; t_rank <= _sol[s_t.second].size();
               ++t_rank) {
            OrOpt r(_input,
                    _sol_state,
                    _sol[s_t.first],
                    s_t.first,
                    s_rank,
                    _sol[s_t.second],
                    s_t.second,
                    t_rank);

            auto& current_best = best_gains[s_t.first][s_t.second];
            if (r.gain_upper_bound() > current_best and r.is_valid() and
                r.gain() > current_best) {
              current_best = r.gain();
              best_ops[s_t.first][s_t.second] = std::make_unique<OrOpt>(r);
            }
          }
        }
      }
    }

    // Operators applied to a single route.

    // Intra exchange stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.first != s_t.second or best_priorities[s_t.first] > 0 or
          _sol[s_t.first].size() < 3) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 2; ++s_rank) {
        unsigned max_t_rank = _sol[s_t.first].size() - 1;
        if (_input.jobs[_sol[s_t.first].route[s_rank]].type ==
            JOB_TYPE::PICKUP) {
          // Don't move a pickup past its matching delivery.
          max_t_rank = _sol_state.matching_delivery_rank[s_t.first][s_rank] - 1;
        }

        for (unsigned t_rank = s_rank + 2; t_rank <= max_t_rank; ++t_rank) {
          if (_input.jobs[_sol[s_t.first].route[t_rank]].type ==
                JOB_TYPE::DELIVERY and
              s_rank <= _sol_state.matching_pickup_rank[s_t.first][t_rank]) {
            // Don't move a delivery before its matching pickup.
            continue;
          }

          IntraExchange r(_input,
                          _sol_state,
                          _sol[s_t.first],
                          s_t.first,
                          s_rank,
                          t_rank);

          if (r.gain() > best_gains[s_t.first][s_t.first] and r.is_valid()) {
            best_gains[s_t.first][s_t.first] = r.gain();
            best_ops[s_t.first][s_t.first] = std::make_unique<IntraExchange>(r);
          }
        }
      }
    }

    // Intra CROSS-exchange stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.first != s_t.second or best_priorities[s_t.first] > 0 or
          _sol[s_t.first].size() < 5) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank <= _sol[s_t.first].size() - 4;
           ++s_rank) {
        const auto& job_s_type =
          _input.jobs[_sol[s_t.first].route[s_rank]].type;

        bool both_s_single =
          (job_s_type == JOB_TYPE::SINGLE) and
          (_input.jobs[_sol[s_t.first].route[s_rank + 1]].type ==
           JOB_TYPE::SINGLE);

        bool is_s_pickup =
          (job_s_type == JOB_TYPE::PICKUP) and
          (_sol_state.matching_delivery_rank[s_t.first][s_rank] == s_rank + 1);

        if (!both_s_single and !is_s_pickup) {
          continue;
        }

        for (unsigned t_rank = s_rank + 3; t_rank < _sol[s_t.first].size() - 1;
             ++t_rank) {
          const auto& job_t_type =
            _input.jobs[_sol[s_t.second].route[t_rank]].type;

          bool both_t_single =
            (job_t_type == JOB_TYPE::SINGLE) and
            (_input.jobs[_sol[s_t.second].route[t_rank + 1]].type ==
             JOB_TYPE::SINGLE);

          bool is_t_pickup =
            (job_t_type == JOB_TYPE::PICKUP) and
            (_sol_state.matching_delivery_rank[s_t.second][t_rank] ==
             t_rank + 1);

          if (!both_t_single and !is_t_pickup) {
            continue;
          }

          IntraCrossExchange r(_input,
                               _sol_state,
                               _sol[s_t.first],
                               s_t.first,
                               s_rank,
                               t_rank,
                               !is_s_pickup,
                               !is_t_pickup);

          auto& current_best = best_gains[s_t.first][s_t.second];
          if (r.gain_upper_bound() > current_best and r.is_valid() and
              r.gain() > current_best) {
            current_best = r.gain();
            best_ops[s_t.first][s_t.first] =
              std::make_unique<IntraCrossExchange>(r);
          }
        }
      }
    }

    // Intra mixed-exchange stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.first != s_t.second or best_priorities[s_t.first] > 0 or
          _sol[s_t.first].size() < 4) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
        if (_input.jobs[_sol[s_t.first].route[s_rank]].type !=
            JOB_TYPE::SINGLE) {
          // Don't try moving part of a shipment.
          continue;
        }

        for (unsigned t_rank = 0; t_rank < _sol[s_t.first].size() - 1;
             ++t_rank) {
          if (t_rank <= s_rank + 1 and s_rank <= t_rank + 2) {
            continue;
          }
          const auto& job_t_type =
            _input.jobs[_sol[s_t.second].route[t_rank]].type;

          bool both_t_single =
            (job_t_type == JOB_TYPE::SINGLE) and
            (_input.jobs[_sol[s_t.first].route[t_rank + 1]].type ==
             JOB_TYPE::SINGLE);

          bool is_t_pickup =
            (job_t_type == JOB_TYPE::PICKUP) and
            (_sol_state.matching_delivery_rank[s_t.second][t_rank] ==
             t_rank + 1);

          if (!both_t_single and !is_t_pickup) {
            continue;
          }

          IntraMixedExchange r(_input,
                               _sol_state,
                               _sol[s_t.first],
                               s_t.first,
                               s_rank,
                               t_rank,
                               !is_t_pickup);
          auto& current_best = best_gains[s_t.first][s_t.second];
          if (r.gain_upper_bound() > current_best and r.is_valid() and
              r.gain() > current_best) {
            current_best = r.gain();
            best_ops[s_t.first][s_t.first] =
              std::make_unique<IntraMixedExchange>(r);
          }
        }
      }
    }

    // Intra relocate stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.first != s_t.second or best_priorities[s_t.first] > 0 or
          _sol[s_t.first].size() < 2) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
        if (_sol_state.node_gains[s_t.first][s_rank] <=
            best_gains[s_t.first][s_t.first]) {
          // Except if addition cost in route is negative (!!),
          // overall gain can't exceed current known best gain.
          continue;
        }

        unsigned min_t_rank = 0;
        if (_input.jobs[_sol[s_t.first].route[s_rank]].type ==
            JOB_TYPE::DELIVERY) {
          // Don't move a delivery before its matching pickup.
          min_t_rank = _sol_state.matching_pickup_rank[s_t.first][s_rank] + 1;
        }

        unsigned max_t_rank = _sol[s_t.first].size() - 1;
        if (_input.jobs[_sol[s_t.first].route[s_rank]].type ==
            JOB_TYPE::PICKUP) {
          // Don't move a pickup past its matching delivery.
          max_t_rank = _sol_state.matching_delivery_rank[s_t.first][s_rank] - 1;
        }

        for (unsigned t_rank = min_t_rank; t_rank <= max_t_rank; ++t_rank) {
          if (t_rank == s_rank) {
            continue;
          }

          IntraRelocate r(_input,
                          _sol_state,
                          _sol[s_t.first],
                          s_t.first,
                          s_rank,
                          t_rank);

          if (r.gain() > best_gains[s_t.first][s_t.first] and r.is_valid()) {
            best_gains[s_t.first][s_t.first] = r.gain();
            best_ops[s_t.first][s_t.first] = std::make_unique<IntraRelocate>(r);
          }
        }
      }
    }

    // Intra Or-opt stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.first != s_t.second or best_priorities[s_t.first] > 0 or
          _sol[s_t.first].size() < 4) {
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 1; ++s_rank) {
        const auto& job_type = _input.jobs[_sol[s_t.first].route[s_rank]].type;

        bool both_single =
          (job_type == JOB_TYPE::SINGLE) and
          (_input.jobs[_sol[s_t.first].route[s_rank + 1]].type ==
           JOB_TYPE::SINGLE);

        bool is_pickup =
          (job_type == JOB_TYPE::PICKUP) and
          (_sol_state.matching_delivery_rank[s_t.first][s_rank] == s_rank + 1);

        if (!both_single and !is_pickup) {
          continue;
        }

        if (is_pickup) {
          if (_sol_state.pd_gains[s_t.first][s_rank] <=
              best_gains[s_t.first][s_t.first]) {
            // Except if addition cost in route is negative (!!),
            // overall gain can't exceed current known best gain.
            continue;
          }
        } else {
          // Regular single job.
          if (_sol_state.edge_gains[s_t.first][s_rank] <=
              best_gains[s_t.first][s_t.first]) {
            // Except if addition cost in route is negative (!!),
            // overall gain can't exceed current known best gain.
            continue;
          }
        }

        for (unsigned t_rank = 0; t_rank <= _sol[s_t.first].size() - 2;
             ++t_rank) {
          if (t_rank == s_rank) {
            continue;
          }
          IntraOrOpt r(_input,
                       _sol_state,
                       _sol[s_t.first],
                       s_t.first,
                       s_rank,
                       t_rank,
                       !is_pickup);
          auto& current_best = best_gains[s_t.first][s_t.second];
          if (r.gain_upper_bound() > current_best and r.is_valid() and
              r.gain() > current_best) {
            current_best = r.gain();
            best_ops[s_t.first][s_t.first] = std::make_unique<IntraOrOpt>(r);
          }
        }
      }
    }

    if (_input.has_shipments()) {
      // Move(s) that don't make sense for job-only instances.

      // P&D relocate stuff
      for (const auto& s_t : s_t_pairs) {
        if (s_t.first == s_t.second or best_priorities[s_t.first] > 0 or
            best_priorities[s_t.second] > 0 or _sol[s_t.first].size() == 0) {
          // Don't try to put things from an empty vehicle.
          continue;
        }

        for (unsigned s_p_rank = 0; s_p_rank < _sol[s_t.first].size();
             ++s_p_rank) {
          if (_input.jobs[_sol[s_t.first].route[s_p_rank]].type !=
              JOB_TYPE::PICKUP) {
            continue;
          }

          // Matching delivery rank in source route.
          unsigned s_d_rank =
            _sol_state.matching_delivery_rank[s_t.first][s_p_rank];

          if (!_input.vehicle_ok_with_job(s_t.second,
                                          _sol[s_t.first].route[s_p_rank]) or
              !_input.vehicle_ok_with_job(s_t.second,
                                          _sol[s_t.first].route[s_d_rank])) {
            continue;
          }

          if (_sol_state.pd_gains[s_t.first][s_p_rank] <=
              best_gains[s_t.first][s_t.second]) {
            // Except if addition cost in route s_t.second is negative
            // (!!), overall gain can't exceed current known best gain.
            continue;
          }

          PDShift pdr(_input,
                      _sol_state,
                      _sol[s_t.first],
                      s_t.first,
                      s_p_rank,
                      s_d_rank,
                      _sol[s_t.second],
                      s_t.second,
                      best_gains[s_t.first][s_t.second]);

          if (pdr.gain() > best_gains[s_t.first][s_t.second] and
              pdr.is_valid()) {
            best_gains[s_t.first][s_t.second] = pdr.gain();
            best_ops[s_t.first][s_t.second] = std::make_unique<PDShift>(pdr);
          }
        }
      }
    }

    if (!_input.has_homogeneous_locations()) {
      // Route exchange stuff
      for (const auto& s_t : s_t_pairs) {
        if (s_t.second <= s_t.first or best_priorities[s_t.first] > 0 or
            best_priorities[s_t.second] > 0 or
            (_sol[s_t.first].size() == 0 and _sol[s_t.second].size() == 0) or
            _sol_state.bwd_skill_rank[s_t.first][s_t.second] > 0 or
            _sol_state.bwd_skill_rank[s_t.second][s_t.first] > 0) {
          // Different routes (and operator is symmetric), at least
          // one non-empty and valid wrt vehicle/job compatibility.
          continue;
        }

        RouteExchange re(_input,
                         _sol_state,
                         _sol[s_t.first],
                         s_t.first,
                         _sol[s_t.second],
                         s_t.second);

        if (re.gain() > best_gains[s_t.first][s_t.second] and re.is_valid()) {
          best_gains[s_t.first][s_t.second] = re.gain();
          best_ops[s_t.first][s_t.second] = std::make_unique<RouteExchange>(re);
        }
      }
    }

    // Find best overall move, first checking priority increase then
    // best gain if no priority increase is available.
    best_priority = 0;
    best_gain = 0;
    Index best_source = 0;
    Index best_target = 0;

    for (unsigned s_v = 0; s_v < _nb_vehicles; ++s_v) {
      if (best_priorities[s_v] > best_priority) {
        best_priority = best_priorities[s_v];
        best_gain = best_gains[s_v][s_v];
        best_source = s_v;
        best_target = s_v;
      }
    }

    if (best_priority == 0) {
      for (unsigned s_v = 0; s_v < _nb_vehicles; ++s_v) {
        for (unsigned t_v = 0; t_v < _nb_vehicles; ++t_v) {
          if (best_gains[s_v][t_v] > best_gain) {
            best_gain = best_gains[s_v][t_v];
            best_source = s_v;
            best_target = t_v;
          }
        }
      }
    }

    // Apply matching operator.
    if (best_priority > 0 or best_gain > 0) {
      assert(best_ops[best_source][best_target] != nullptr);

      best_ops[best_source][best_target]->apply();

      auto update_candidates =
        best_ops[best_source][best_target]->update_candidates();

#ifndef NDEBUG
      // Update route costs.
      const auto previous_cost =
        std::accumulate(update_candidates.begin(),
                        update_candidates.end(),
                        0,
                        [&](auto sum, auto c) {
                          return sum + _sol_state.route_costs[c];
                        });
      for (auto v_rank : update_candidates) {
        _sol_state.update_route_cost(_sol[v_rank].route, v_rank);
      }
      const auto new_cost =
        std::accumulate(update_candidates.begin(),
                        update_candidates.end(),
                        0,
                        [&](auto sum, auto c) {
                          return sum + _sol_state.route_costs[c];
                        });
      assert(new_cost + best_gain == previous_cost);
#endif

      try_job_additions(best_ops[best_source][best_target]
                          ->addition_candidates(),
                        0);

      // Running update_costs only after try_job_additions is fine.
      for (auto v_rank : update_candidates) {
        _sol_state.update_costs(_sol[v_rank].route, v_rank);
      }

      for (auto v_rank : update_candidates) {
        _sol_state.update_skills(_sol[v_rank].route, v_rank);
      }

      // Update candidates.
      for (auto v_rank : update_candidates) {
        _sol_state.set_node_gains(_sol[v_rank].route, v_rank);
        _sol_state.set_edge_gains(_sol[v_rank].route, v_rank);
        _sol_state.set_pd_matching_ranks(_sol[v_rank].route, v_rank);
        _sol_state.set_pd_gains(_sol[v_rank].route, v_rank);
      }

      // Set gains to zero for what needs to be recomputed in the next
      // round and set route pairs accordingly.
      s_t_pairs.clear();
      for (auto v_rank : update_candidates) {
        best_gains[v_rank].assign(_nb_vehicles, 0);
        best_priorities[v_rank] = 0;
        best_ops[v_rank] = std::vector<std::unique_ptr<Operator>>(_nb_vehicles);
      }

      for (unsigned v = 0; v < _nb_vehicles; ++v) {
        for (auto v_rank : update_candidates) {
          if (_input.vehicle_ok_with_vehicle(v, v_rank)) {
            best_gains[v][v_rank] = 0;
            best_ops[v][v_rank] = std::unique_ptr<Operator>();

            s_t_pairs.emplace_back(v, v_rank);
            if (v != v_rank) {
              s_t_pairs.emplace_back(v_rank, v);
            }
          }
        }
      }

      for (unsigned v = 0; v < _nb_vehicles; ++v) {
        if (best_ops[v][v] == nullptr) {
          continue;
        }
        for (auto req_u : best_ops[v][v]->required_unassigned()) {
          if (_sol_state.unassigned.find(req_u) ==
              _sol_state.unassigned.end()) {
            // This move should be invalidated because a required
            // unassigned job has been added by try_job_additions in
            // the meantime.
            best_gains[v][v] = 0;
            best_priorities[v] = 0;
            best_ops[v][v] = std::unique_ptr<Operator>();
            s_t_pairs.emplace_back(v, v);
          }
        }
      }
    }
  }
}

template <class Route,
          class UnassignedExchange,
          class Exchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class PDShift,
          class RouteExchange>
void LocalSearch<Route,
                 UnassignedExchange,
                 Exchange,
                 CrossExchange,
                 MixedExchange,
                 TwoOpt,
                 ReverseTwoOpt,
                 Relocate,
                 OrOpt,
                 IntraExchange,
                 IntraCrossExchange,
                 IntraMixedExchange,
                 IntraRelocate,
                 IntraOrOpt,
                 PDShift,
                 RouteExchange>::run() {
  bool try_ls_step = true;
  bool first_step = true;

  unsigned current_nb_removal = 1;

  while (try_ls_step) {
    // A round of local search.
    run_ls_step();

    // Indicators for current solution.
    Index v_rank = 0;
    utils::SolutionIndicators current_sol_indicators;
    current_sol_indicators.priority_sum =
      std::accumulate(_sol.begin(),
                      _sol.end(),
                      0,
                      [&](auto sum, const auto& r) {
                        return sum +
                               utils::priority_sum_for_route(_input, r.route, v_rank);
                      });

    current_sol_indicators.unassigned = _sol_state.unassigned.size();

    v_rank = 0;
    current_sol_indicators.cost =
      std::accumulate(_sol.begin(),
                      _sol.end(),
                      0,
                      [&](auto sum, const auto& r) {
                        return sum + utils::route_cost_for_vehicle(_input,
                                                                   v_rank,
                                                                   r.route);
                      });

    current_sol_indicators.used_vehicles =
      std::count_if(_sol.begin(), _sol.end(), [](const auto& r) {
        return !r.empty();
      });

    if (current_sol_indicators < _best_sol_indicators) {
      _best_sol_indicators = current_sol_indicators;
      _best_sol = _sol;
    } else {
      if (!first_step) {
        ++current_nb_removal;
      }
      if (_best_sol_indicators < current_sol_indicators) {
        // Back to best known solution for further steps.
        _sol = _best_sol;
        _sol_state.setup(_sol);
      }
    }

    // Try again on each improvement until we reach last job removal
    // level.
    try_ls_step = (current_nb_removal <= _max_nb_jobs_removal);

    if (try_ls_step) {
      // Get a looser situation by removing jobs.
      for (unsigned i = 0; i < current_nb_removal; ++i) {
        remove_from_routes();
        for (std::size_t v = 0; v < _sol.size(); ++v) {
          _sol_state.set_node_gains(_sol[v].route, v);
          _sol_state.set_pd_matching_ranks(_sol[v].route, v);
          _sol_state.set_pd_gains(_sol[v].route, v);
        }
      }

      // Refill jobs.
      try_job_additions(_all_routes, 1.5);

      // Reset what is needed in solution state.
      _sol_state.setup(_sol);
    }

    first_step = false;
  }
}

template <class Route,
          class UnassignedExchange,
          class Exchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class PDShift,
          class RouteExchange>
Gain LocalSearch<Route,
                 UnassignedExchange,
                 Exchange,
                 CrossExchange,
                 MixedExchange,
                 TwoOpt,
                 ReverseTwoOpt,
                 Relocate,
                 OrOpt,
                 IntraExchange,
                 IntraCrossExchange,
                 IntraMixedExchange,
                 IntraRelocate,
                 IntraOrOpt,
                 PDShift,
                 RouteExchange>::job_route_cost(Index v_target,
                                                Index v,
                                                Index r) {
  assert(v != v_target);

  Gain cost = static_cast<Gain>(INFINITE_COST);
  const auto job_index = _input.jobs[_sol[v].route[r]].index();

  if (_input.vehicles[v_target].has_start()) {
    const auto start_index = _input.vehicles[v_target].start.value().index();
    const Gain start_cost = _matrix[start_index][job_index];
    cost = std::min(cost, start_cost);
  }
  if (_input.vehicles[v_target].has_end()) {
    const auto end_index = _input.vehicles[v_target].end.value().index();
    const Gain end_cost = _matrix[job_index][end_index];
    cost = std::min(cost, end_cost);
  }
  if (_sol[v_target].size() != 0) {
    const auto nearest_from_rank =
      _sol_state.nearest_job_rank_in_routes_from[v][v_target][r];
    const auto nearest_from_index =
      _input.jobs[_sol[v_target].route[nearest_from_rank]].index();
    const Gain cost_from = _matrix[nearest_from_index][job_index];
    cost = std::min(cost, cost_from);

    const auto nearest_to_rank =
      _sol_state.nearest_job_rank_in_routes_to[v][v_target][r];
    const auto nearest_to_index =
      _input.jobs[_sol[v_target].route[nearest_to_rank]].index();
    const Gain Costo = _matrix[job_index][nearest_to_index];
    cost = std::min(cost, Costo);
  }

  return cost;
}

template <class Route,
          class UnassignedExchange,
          class Exchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class PDShift,
          class RouteExchange>
Gain LocalSearch<Route,
                 UnassignedExchange,
                 Exchange,
                 CrossExchange,
                 MixedExchange,
                 TwoOpt,
                 ReverseTwoOpt,
                 Relocate,
                 OrOpt,
                 IntraExchange,
                 IntraCrossExchange,
                 IntraMixedExchange,
                 IntraRelocate,
                 IntraOrOpt,
                 PDShift,
                 RouteExchange>::best_relocate_cost(Index v, Index r) {
  Gain best_cost = static_cast<Gain>(INFINITE_COST);

  for (std::size_t other_v = 0; other_v < _sol.size(); ++other_v) {
    if (other_v == v or
        !_input.vehicle_ok_with_job(other_v, _sol[v].route[r])) {
      continue;
    }

    best_cost = std::min(best_cost, job_route_cost(other_v, v, r));
  }

  return best_cost;
}

template <class Route,
          class UnassignedExchange,
          class Exchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class PDShift,
          class RouteExchange>
Gain LocalSearch<Route,
                 UnassignedExchange,
                 Exchange,
                 CrossExchange,
                 MixedExchange,
                 TwoOpt,
                 ReverseTwoOpt,
                 Relocate,
                 OrOpt,
                 IntraExchange,
                 IntraCrossExchange,
                 IntraMixedExchange,
                 IntraRelocate,
                 IntraOrOpt,
                 PDShift,
                 RouteExchange>::best_relocate_cost(Index v,
                                                    Index r1,
                                                    Index r2) {
  Gain best_cost = static_cast<Gain>(INFINITE_COST);

  for (std::size_t other_v = 0; other_v < _sol.size(); ++other_v) {
    if (other_v == v or
        !_input.vehicle_ok_with_job(other_v, _sol[v].route[r1])) {
      continue;
    }

    best_cost =
      std::min(best_cost,
               job_route_cost(other_v, v, r1) + job_route_cost(other_v, v, r2));
  }

  return best_cost;
}

template <class Route,
          class UnassignedExchange,
          class Exchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class PDShift,
          class RouteExchange>
void LocalSearch<Route,
                 UnassignedExchange,
                 Exchange,
                 CrossExchange,
                 MixedExchange,
                 TwoOpt,
                 ReverseTwoOpt,
                 Relocate,
                 OrOpt,
                 IntraExchange,
                 IntraCrossExchange,
                 IntraMixedExchange,
                 IntraRelocate,
                 IntraOrOpt,
                 PDShift,
                 RouteExchange>::remove_from_routes() {
  // Store nearest job from and to any job in any route for constant
  // time access down the line.
  for (std::size_t v1 = 0; v1 < _nb_vehicles; ++v1) {
    for (std::size_t v2 = 0; v2 < _nb_vehicles; ++v2) {
      if (v2 == v1) {
        continue;
      }
      _sol_state.update_nearest_job_rank_in_routes(_sol[v1].route,
                                                   _sol[v2].route,
                                                   v1,
                                                   v2);
    }
  }

  // Remove best node candidate from all routes.
  std::vector<std::pair<Index, Index>> routes_and_ranks;

  for (std::size_t v = 0; v < _sol.size(); ++v) {
    if (_sol[v].empty()) {
      continue;
    }

    // Try removing the best node (good gain on current route and
    // small cost to closest node in another compatible route).
    Index best_rank = 0;
    Gain best_gain = std::numeric_limits<Gain>::min();

    for (std::size_t r = 0; r < _sol[v].size(); ++r) {
      const auto& current_job = _input.jobs[_sol[v].route[r]];
      if (current_job.type == JOB_TYPE::DELIVERY) {
        continue;
      }

      Gain current_gain;
      bool valid_removal;

      if (current_job.type == JOB_TYPE::SINGLE) {
        current_gain = _sol_state.node_gains[v][r] - best_relocate_cost(v, r);

        if (current_gain > best_gain) {
          // Only check validity if required.
          valid_removal = _sol[v].is_valid_removal(_input, r, 1);
        }
      } else {
        assert(current_job.type == JOB_TYPE::PICKUP);
        auto delivery_r = _sol_state.matching_delivery_rank[v][r];
        current_gain =
          _sol_state.pd_gains[v][r] - best_relocate_cost(v, r, delivery_r);

        if (current_gain > best_gain) {
          // Only check validity if required.
          if (delivery_r == r + 1) {
            valid_removal = _sol[v].is_valid_removal(_input, r, 2);
          } else {
            std::vector<Index> between_pd(_sol[v].route.begin() + r + 1,
                                          _sol[v].route.begin() + delivery_r);

            valid_removal = _sol[v].is_valid_addition_for_tw(_input,
                                                             between_pd.begin(),
                                                             between_pd.end(),
                                                             r,
                                                             delivery_r + 1);
          }
        }
      }

      if (current_gain > best_gain and valid_removal) {
        best_gain = current_gain;
        best_rank = r;
      }
    }

    if (best_gain > std::numeric_limits<Gain>::min()) {
      routes_and_ranks.push_back(std::make_pair(v, best_rank));
    }
  }

  for (const auto& r_r : routes_and_ranks) {
    const auto v = r_r.first;
    const auto r = r_r.second;

    _sol_state.unassigned.insert(_sol[v].route[r]);

    const auto& current_job = _input.jobs[_sol[v].route[r]];
    if (current_job.type == JOB_TYPE::SINGLE) {
      _sol[v].remove(_input, r, 1);
    } else {
      assert(current_job.type == JOB_TYPE::PICKUP);
      const auto delivery_r = _sol_state.matching_delivery_rank[v][r];
      _sol_state.unassigned.insert(_sol[v].route[delivery_r]);

      if (delivery_r == r + 1) {
        _sol[v].remove(_input, r, 2);
      } else {
        std::vector<Index> between_pd(_sol[v].route.begin() + r + 1,
                                      _sol[v].route.begin() + delivery_r);
        _sol[v].replace(_input,
                        between_pd.begin(),
                        between_pd.end(),
                        r,
                        delivery_r + 1);
      }
    }
  }
}

template <class Route,
          class UnassignedExchange,
          class Exchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class PDShift,
          class RouteExchange>
utils::SolutionIndicators LocalSearch<Route,
                                      UnassignedExchange,
                                      Exchange,
                                      CrossExchange,
                                      MixedExchange,
                                      TwoOpt,
                                      ReverseTwoOpt,
                                      Relocate,
                                      OrOpt,
                                      IntraExchange,
                                      IntraCrossExchange,
                                      IntraMixedExchange,
                                      IntraRelocate,
                                      IntraOrOpt,
                                      PDShift,
                                      RouteExchange>::indicators() const {
  return _best_sol_indicators;
}

template class LocalSearch<TWRoute,
                           vrptw::UnassignedExchange,
                           vrptw::Exchange,
                           vrptw::CrossExchange,
                           vrptw::MixedExchange,
                           vrptw::TwoOpt,
                           vrptw::ReverseTwoOpt,
                           vrptw::Relocate,
                           vrptw::OrOpt,
                           vrptw::IntraExchange,
                           vrptw::IntraCrossExchange,
                           vrptw::IntraMixedExchange,
                           vrptw::IntraRelocate,
                           vrptw::IntraOrOpt,
                           vrptw::PDShift,
                           vrptw::RouteExchange>;

template class LocalSearch<RawRoute,
                           cvrp::UnassignedExchange,
                           cvrp::Exchange,
                           cvrp::CrossExchange,
                           cvrp::MixedExchange,
                           cvrp::TwoOpt,
                           cvrp::ReverseTwoOpt,
                           cvrp::Relocate,
                           cvrp::OrOpt,
                           cvrp::IntraExchange,
                           cvrp::IntraCrossExchange,
                           cvrp::IntraMixedExchange,
                           cvrp::IntraRelocate,
                           cvrp::IntraOrOpt,
                           cvrp::PDShift,
                           cvrp::RouteExchange>;

} // namespace ls
} // namespace vroom
