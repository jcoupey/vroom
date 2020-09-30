#ifndef VRPTW_PD_SHIFT_H
#define VRPTW_PD_SHIFT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/pd_shift.h"
#include "structures/vroom/tw_route.h"

namespace vroom {
namespace vrptw {

class PDShift : public cvrp::PDShift {
private:
  bool _is_valid_removal;
  std::vector<Index> _source_without_pd;
  TWRoute& _tw_s_route;
  TWRoute& _tw_t_route;

  virtual void compute_gain() override;

public:
  PDShift(const Input& input,
          const utils::SolutionState& sol_state,
          TWRoute& tw_s_route,
          Index s_vehicle,
          Index s_p_rank,
          Index s_d_rank,
          TWRoute& tw_t_route,
          Index t_vehicle,
          Gain gain_threshold);

  void log_route(const std::vector<Index>& route) const;

  virtual void apply() override;
};

} // namespace vrptw
} // namespace vroom

#endif
