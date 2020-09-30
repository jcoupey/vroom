#ifndef TSP_H
#define TSP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrp.h"
#include "structures/generic/matrix.h"
#include "structures/typedefs.h"
#include "structures/vroom/raw_route.h"
#include "structures/vroom/solution/solution.h"

namespace vroom {

using RawSolution = std::vector<RawRoute>;

class TSP : public VRP {
private:
  Index _vehicle_rank;
  // Holds the matching from index in _matrix to rank in input::_jobs.
  std::vector<Index> _job_ranks;
  bool _is_symmetric;
  bool _has_start;
  Index _start;
  bool _has_end;
  Index _end;
  Matrix<Cost> _matrix;
  Matrix<Cost> _symmetrized_matrix;
  bool _round_trip;

public:
  TSP(const Input& input, std::vector<Index> job_ranks, Index vehicle_rank);

  Cost cost(const std::list<Index>& tour) const;

  Cost symmetrized_cost(const std::list<Index>& tour) const;

  std::vector<Index> raw_solve(unsigned nb_threads) const;

  virtual Solution
  solve(unsigned,
        unsigned nb_threads,
        const std::vector<HeuristicParameters>&) const override;
};

} // namespace vroom

#endif
