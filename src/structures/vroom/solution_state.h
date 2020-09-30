#ifndef SOLUTION_STATE_H
#define SOLUTION_STATE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <unordered_set>

#include "structures/typedefs.h"
#include "structures/vroom/amount.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/raw_route.h"
#include "structures/vroom/tw_route.h"

namespace vroom {
namespace utils {

using RawSolution = std::vector<RawRoute>;
using TWSolution = std::vector<TWRoute>;

struct SolutionIndicators {
  Priority priority_sum;
  unsigned unassigned;
  Cost cost;
  unsigned used_vehicles;

  friend bool operator<(const SolutionIndicators& lhs,
                        const SolutionIndicators& rhs) {
    if (lhs.priority_sum > rhs.priority_sum) {
      return true;
    }
    if (lhs.priority_sum == rhs.priority_sum) {
      if (lhs.unassigned < rhs.unassigned) {
        return true;
      }
      if (lhs.unassigned == rhs.unassigned) {
        if (lhs.cost < rhs.cost) {
          return true;
        }
        if (lhs.cost == rhs.cost and lhs.used_vehicles < rhs.used_vehicles) {
          return true;
        }
      }
    }

    return false;
  }
};

class SolutionState {
private:
  const Input& _input;
  const Matrix<Cost>& _m;
  const std::size_t _nb_vehicles;

public:
  // Store unassigned jobs.
  std::unordered_set<Index> unassigned;

  // fwd_costs[v][i] stores the total cost from job at rank 0 to job
  // at rank i in the route for vehicle v, while bwd_costs[v][i]
  // stores the total cost from job at rank i to job at rank 0
  // (i.e. when *reversing* all edges).
  std::vector<std::vector<Cost>> fwd_costs;
  std::vector<std::vector<Cost>> bwd_costs;

  // fwd_skill_rank[v1][v2] stores the maximum rank r for a step in
  // route for vehicle v1 such that v2 can handle all jobs from step 0
  // to r -- excluded -- in that route. bwd_skill_rank[v1][v2] stores
  // the minimum rank r for a step in route for vehicle v1 such that
  // v2 can handle all jobs after step r -- included -- up to the end
  // of that route.
  std::vector<std::vector<Index>> fwd_skill_rank;
  std::vector<std::vector<Index>> bwd_skill_rank;

  // edge_costs_around_node[v][i] stores the sum of costs for edges
  // that appear before and after job at rank i in route for vehicle v
  // (handling cases where those edges are absent or linked with
  // start/end of vehicle). node_gains[v][i] stores potential gain
  // when removing job at rank i in route for vehicle
  // v. node_candidates[v] is the rank that yields the biggest such
  // gain for vehicle v.
  std::vector<std::vector<Gain>> edge_costs_around_node;
  std::vector<std::vector<Gain>> node_gains;
  std::vector<Index> node_candidates;

  // edge_costs_around_edge[v][i] stores the sum of costs for edges
  // that appear before and after edge starting at rank i in route for
  // vehicle v (handling cases where those edges are absent or linked
  // with start/end of vehicle). edge_gains[v][i] stores potential
  // gain when removing edge starting at rank i in route for vehicle
  // v. edge_candidates[v] is the rank that yields the biggest such
  // gain for vehicle v.
  std::vector<std::vector<Gain>> edge_costs_around_edge;
  std::vector<std::vector<Gain>> edge_gains;
  std::vector<Index> edge_candidates;

  // pd_gains[v][i] stores potential gain when removing pickup at rank
  // i in route for vehicle v along with it's associated delivery.
  std::vector<std::vector<Gain>> pd_gains;

  // If job at rank i in route for vehicle v is a pickup
  // (resp. delivery), then matching_delivery_rank[v][i]
  // (resp. _matching_pickup_rank[v][i]) stores the rank of the
  // matching delivery (resp. pickup).
  std::vector<std::vector<Index>> matching_delivery_rank;
  std::vector<std::vector<Index>> matching_pickup_rank;

  // nearest_job_rank_in_routes_from[v1][v2][r1] stores the rank of
  // job in route v2 that minimize cost from job at rank r1 in v1.
  std::vector<std::vector<std::vector<Index>>> nearest_job_rank_in_routes_from;
  // nearest_job_rank_in_routes_to[v1][v2][r1] stores the rank of job
  // in route v2 that minimize cost to job at rank r1 in v1.
  std::vector<std::vector<std::vector<Index>>> nearest_job_rank_in_routes_to;

  // Only used for assertions in debug mode.
  std::vector<Cost> route_costs;

  SolutionState(const Input& input);

  void setup(const std::vector<Index>& r, Index v);

  void setup(const RawSolution& sol);

  void setup(const TWSolution& tw_sol);

  void update_costs(const std::vector<Index>& route, Index v);

  void update_skills(const std::vector<Index>& route, Index v1);

  void set_node_gains(const std::vector<Index>& route, Index v);

  void set_edge_gains(const std::vector<Index>& route, Index v);

  void set_pd_gains(const std::vector<Index>& route, Index v);

  void set_pd_matching_ranks(const std::vector<Index>& route, Index v);

  void update_nearest_job_rank_in_routes(const std::vector<Index>& route_1,
                                         const std::vector<Index>& route_2,
                                         Index v1,
                                         Index v2);

  void update_route_cost(const std::vector<Index>& route, Index v);
};

} // namespace utils
} // namespace vroom

#endif
