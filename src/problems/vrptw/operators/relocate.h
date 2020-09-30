#ifndef VRPTW_RELOCATE_H
#define VRPTW_RELOCATE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/relocate.h"
#include "structures/vroom/tw_route.h"

namespace vroom {
namespace vrptw {

class Relocate : public cvrp::Relocate {
private:
  TWRoute& _tw_s_route;
  TWRoute& _tw_t_route;

public:
  Relocate(const Input& input,
           const utils::SolutionState& sol_state,
           TWRoute& tw_s_route,
           Index s_vehicle,
           Index s_rank,
           TWRoute& tw_t_route,
           Index t_vehicle,
           Index t_rank);

  virtual bool is_valid() override;

  virtual void apply() override;
};

} // namespace vrptw
} // namespace vroom

#endif
