#ifndef CVRP_PD_SHIFT_H
#define CVRP_PD_SHIFT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom {
namespace cvrp {

class PDShift : public ls::Operator {
protected:
  const Index _s_p_rank;
  const Index _s_d_rank;
  const Gain _remove_gain;
  Index _best_t_p_rank;
  Index _best_t_d_rank;
  bool _valid;

  virtual void compute_gain() override;

public:
  // The gain_threshold parameter serves as a filter to NOT even test
  // validity for possible P&D insertions in target route if they are
  // too expensive.
  PDShift(const Input& input,
          const utils::SolutionState& sol_state,
          RawRoute& s_route,
          Index s_vehicle,
          Index s_p_rank,
          Index s_d_rank,
          RawRoute& t_route,
          Index t_vehicle,
          Gain gain_threshold);

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<Index> addition_candidates() const override;

  virtual std::vector<Index> update_candidates() const override;
};

} // namespace cvrp
} // namespace vroom

#endif
