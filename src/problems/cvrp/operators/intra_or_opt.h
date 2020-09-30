#ifndef CVRP_INTRA_OR_OPT_H
#define CVRP_INTRA_OR_OPT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom {
namespace cvrp {

class IntraOrOpt : public ls::Operator {
private:
  bool _gain_upper_bound_computed;
  Gain _s_gain;
  Gain _normal_t_gain;
  Gain _reversed_t_gain;

protected:
  bool reverse_s_edge;

  bool is_normal_valid;
  bool is_reverse_valid;
  const bool check_reverse;

  std::vector<Index> _moved_jobs;
  const Index _first_rank;
  const Index _last_rank;
  Index _s_edge_first;
  Index _s_edge_last;

  virtual void compute_gain() override;

public:
  IntraOrOpt(const Input& input,
             const utils::SolutionState& sol_state,
             RawRoute& s_route,
             Index s_vehicle,
             Index s_rank,
             Index t_rank, // rank *after* removal.
             bool check_reverse);

  // Compute and store all possible cost depending on whether edges
  // are reversed or not. Return only an upper bound for gain as
  // precise gain requires validity information.
  Gain gain_upper_bound();

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<Index> addition_candidates() const override;

  virtual std::vector<Index> update_candidates() const override;
};

} // namespace cvrp
} // namespace vroom

#endif
