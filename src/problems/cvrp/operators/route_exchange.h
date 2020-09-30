#ifndef CVRP_ROUTE_EXCHANGE_H
#define CVRP_ROUTE_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom {
namespace cvrp {

class RouteExchange : public ls::Operator {
protected:
  virtual void compute_gain() override;

public:
  RouteExchange(const Input& input,
                const utils::SolutionState& sol_state,
                RawRoute& s_route,
                Index s_vehicle,
                RawRoute& t_route,
                Index t_vehicle);

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<Index> addition_candidates() const override;

  virtual std::vector<Index> update_candidates() const override;
};

} // namespace cvrp
} // namespace vroom

#endif
