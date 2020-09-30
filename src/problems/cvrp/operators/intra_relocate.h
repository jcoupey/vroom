#ifndef CVRP_INTRA_RELOCATE_H
#define CVRP_INTRA_RELOCATE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom {
namespace cvrp {

class IntraRelocate : public ls::Operator {
protected:
  virtual void compute_gain() override;

  std::vector<Index> _moved_jobs;
  const Index _first_rank;
  const Index _last_rank;

public:
  IntraRelocate(const Input& input,
                const utils::SolutionState& sol_state,
                RawRoute& s_route,
                Index s_vehicle,
                Index s_rank,
                Index t_rank); // relocate rank *after* removal.

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<Index> addition_candidates() const override;

  virtual std::vector<Index> update_candidates() const override;
};

} // namespace cvrp
} // namespace vroom

#endif
