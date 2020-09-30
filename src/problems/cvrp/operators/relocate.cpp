/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/relocate.h"
#include "utils/helpers.h"

namespace vroom {
namespace cvrp {

Relocate::Relocate(const Input& input,
                   const utils::SolutionState& sol_state,
                   RawRoute& s_route,
                   Index s_vehicle,
                   Index s_rank,
                   RawRoute& t_route,
                   Index t_vehicle,
                   Index t_rank)
  : Operator(input,
             sol_state,
             s_route,
             s_vehicle,
             s_rank,
             t_route,
             t_vehicle,
             t_rank) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 1);
  assert(s_rank < s_route.size());
  assert(t_rank <= t_route.size());

  assert(_input.vehicle_ok_with_job(t_vehicle, this->s_route[s_rank]));
}

void Relocate::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v = _input.vehicles[t_vehicle];

  // For source vehicle, we consider the cost of removing job at rank
  // s_rank, already stored in
  // _sol_state.node_gains[s_vehicle][s_rank].

  // For target vehicle, we consider the cost of adding source job at
  // rank t_rank.
  Gain t_gain =
    -utils::addition_cost(_input, m, s_route[s_rank], v, t_route, t_rank);

  stored_gain = _sol_state.node_gains[s_vehicle][s_rank] + t_gain;
  gain_computed = true;
}

bool Relocate::is_valid() {
  return target
    .is_valid_addition_for_capacity(_input,
                                    _input.jobs[s_route[s_rank]].pickup,
                                    _input.jobs[s_route[s_rank]].delivery,
                                    t_rank);
}

void Relocate::apply() {
  auto relocate_job_rank = s_route[s_rank];
  s_route.erase(s_route.begin() + s_rank);
  t_route.insert(t_route.begin() + t_rank, relocate_job_rank);

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> Relocate::addition_candidates() const {
  return {s_vehicle};
}

std::vector<Index> Relocate::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace cvrp
} // namespace vroom
