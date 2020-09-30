#ifndef VRPTW_UNASSIGNED_EXCHANGE_H
#define VRPTW_UNASSIGNED_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/unassigned_exchange.h"
#include "structures/vroom/tw_route.h"

namespace vroom {
namespace vrptw {

class UnassignedExchange : public cvrp::UnassignedExchange {
private:
  TWRoute& _tw_s_route;

public:
  UnassignedExchange(const Input& input,
                     const utils::SolutionState& sol_state,
                     std::unordered_set<Index>& unassigned,
                     TWRoute& tw_s_route,
                     Index s_vehicle,
                     Index s_rank,
                     Index t_rank,
                     Index u);

  virtual bool is_valid() override;

  virtual void apply() override;
};

} // namespace vrptw
} // namespace vroom

#endif
