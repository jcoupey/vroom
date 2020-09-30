#ifndef LOCAL_SEARCH_H
#define LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/raw_route.h"
#include "structures/vroom/solution_state.h"
#include "structures/vroom/tw_route.h"

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
class LocalSearch {
private:
  const Input& _input;
  const Matrix<Cost>& _matrix;
  const std::size_t _nb_vehicles;

  const unsigned _max_nb_jobs_removal;
  std::vector<Index> _all_routes;

  utils::SolutionState _sol_state;

  std::vector<Route> _sol;

  std::vector<Route>& _best_sol;
  utils::SolutionIndicators _best_sol_indicators;

  void try_job_additions(const std::vector<Index>& routes, double regret_coeff);

  void run_ls_step();

  // Compute "cost" between route at rank v_target and job with rank r
  // in route at rank v. Relies on
  // _sol_state.nearest_job_rank_in_routes_* being up to date.
  Gain job_route_cost(Index v_target, Index v, Index r);

  // Compute best cost of relocating job at rank r (resp. jobs at rank
  // r1 and r2) in route v to any other (compatible) route.
  Gain best_relocate_cost(Index v, Index r);
  Gain best_relocate_cost(Index v, Index r1, Index r2);

  void remove_from_routes();

public:
  LocalSearch(const Input& input,
              std::vector<Route>& tw_sol,
              unsigned max_nb_jobs_removal);

  utils::SolutionIndicators indicators() const;

  void run();
};

} // namespace ls
} // namespace vroom

#endif
