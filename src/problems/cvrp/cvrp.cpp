/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "cvrp.h"
#include "../../structures/vroom/input/input.h"

cvrp::cvrp(const input& input) : vrp(input) {
}

bool cvrp::empty_cluster(const std::vector<index_t>& cluster, index_t v) const {
  // Checking if the cluster has only start/end.
  return (cluster.size() == 1) or
         ((cluster.size() == 2) and _input._vehicles[v].has_start() and
          _input._vehicles[v].has_end() and
          (_input._vehicles[v].start.get().index() !=
           _input._vehicles[v].end.get().index()));
}

solution cvrp::solve(unsigned nb_threads) const {
  std::vector<solution> tsp_sols;

  double regret_coeff = 1;
  auto c = parallel_clustering(_input, regret_coeff);

  std::cout << "Clustering:" << c.strategy << ";" << c.regret_coeff
            << ";" << c.edges_cost << std::endl;

  for (std::size_t i = 0; i < c.clusters.size(); ++i) {
    if (empty_cluster(c.clusters[i], i)) {
      std::cout << "Empty cluster" << std::endl;
      continue;
    }

    for (const auto& j : c.clusters[i]) {
      std::cout << j << " ; ";
    }
    std::cout << std::endl;

    tsp p(_input, c.clusters[i], i);

    tsp_sols.push_back(p.solve(1));
  }

  std::vector<route_t> routes;
  cost_t total_cost = 0;
  for (const auto& tsp_sol : tsp_sols) {
    routes.push_back(tsp_sol.routes[0]);
    total_cost += tsp_sol.summary.cost;
  }

  return solution(0, std::move(routes), total_cost);
}