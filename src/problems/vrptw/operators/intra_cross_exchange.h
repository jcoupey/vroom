#ifndef VRPTW_INTRA_CROSS_EXCHANGE_H
#define VRPTW_INTRA_CROSS_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/intra_cross_exchange.h"
#include "structures/vroom/tw_route.h"

namespace vroom {
namespace vrptw {

class IntraCrossExchange : public cvrp::IntraCrossExchange {
private:
  TWRoute& _tw_s_route;

public:
  IntraCrossExchange(const Input& input,
                     const utils::SolutionState& sol_state,
                     TWRoute& tw_s_route,
                     Index s_vehicle,
                     Index s_rank,
                     Index t_rank,
                     bool check_s_reverse,
                     bool check_t_reverse);

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<Index> addition_candidates() const override;
};

} // namespace vrptw
} // namespace vroom

#endif
