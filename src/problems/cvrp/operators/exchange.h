#ifndef CVRP_EXCHANGE_H
#define CVRP_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom {
namespace cvrp {

class Exchange : public ls::Operator {
protected:
  virtual void compute_gain() override;

public:
  Exchange(const Input& input,
           const utils::SolutionState& sol_state,
           RawRoute& s_route,
           Index s_vehicle,
           Index s_rank,
           RawRoute& t_route,
           Index t_vehicle,
           Index t_rank);

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<Index> addition_candidates() const override;

  virtual std::vector<Index> update_candidates() const override;
};

} // namespace cvrp
} // namespace vroom

#endif
