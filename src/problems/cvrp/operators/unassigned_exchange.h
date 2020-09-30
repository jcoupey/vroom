#ifndef CVRP_UNASSIGNED_EXCHANGE_H
#define CVRP_UNASSIGNED_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom {
namespace cvrp {

class UnassignedExchange : public ls::Operator {
protected:
  const Index _u; // Unassigned job to insert.
  std::unordered_set<Index>& _unassigned;
  const Index _first_rank;
  const Index _last_rank;
  std::vector<Index> _moved_jobs;
  const Index _removed;

  virtual void compute_gain() override;

public:
  UnassignedExchange(const Input& input,
                     const utils::SolutionState& sol_state,
                     std::unordered_set<Index>& unassigned,
                     RawRoute& s_route,
                     Index s_vehicle,
                     Index s_rank,
                     Index t_rank,
                     Index u);

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<Index> addition_candidates() const override;

  virtual std::vector<Index> update_candidates() const override;

  virtual std::vector<Index> required_unassigned() const override;
};

} // namespace cvrp
} // namespace vroom

#endif
