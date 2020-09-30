/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/intra_relocate.h"
#include "utils/helpers.h"

namespace vroom {
namespace cvrp {

IntraRelocate::IntraRelocate(const Input& input,
                             const utils::SolutionState& sol_state,
                             RawRoute& s_raw_route,
                             Index s_vehicle,
                             Index s_rank,
                             Index t_rank)
  : Operator(input,
             sol_state,
             s_raw_route,
             s_vehicle,
             s_rank,
             s_raw_route,
             s_vehicle,
             t_rank),
    _moved_jobs((s_rank < t_rank) ? t_rank - s_rank + 1 : s_rank - t_rank + 1),
    _first_rank(std::min(s_rank, t_rank)),
    _last_rank(std::max(s_rank, t_rank) + 1) {
  assert(s_route.size() >= 2);
  assert(s_rank < s_route.size());
  assert(t_rank <= s_route.size() - 1);
  assert(s_rank != t_rank);

  if (t_rank < s_rank) {
    _moved_jobs[0] = s_route[s_rank];
    std::copy(s_route.begin() + t_rank,
              s_route.begin() + s_rank,
              _moved_jobs.begin() + 1);
  } else {
    std::copy(s_route.begin() + s_rank + 1,
              s_route.begin() + t_rank + 1,
              _moved_jobs.begin());
    _moved_jobs.back() = s_route[s_rank];
  }
}

void IntraRelocate::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v_target = _input.vehicles[s_vehicle];

  // For removal, we consider the cost of removing job at rank s_rank,
  // already stored in _sol_state.node_gains[s_vehicle][s_rank].

  // For addition, consider the cost of adding source job at new rank
  // *after* removal.
  auto new_rank = t_rank;
  if (s_rank < t_rank) {
    ++new_rank;
  }
  Gain t_gain = -utils::addition_cost(_input,
                                      m,
                                      s_route[s_rank],
                                      v_target,
                                      t_route,
                                      new_rank);

  stored_gain = _sol_state.node_gains[s_vehicle][s_rank] + t_gain;
  gain_computed = true;
}

bool IntraRelocate::is_valid() {
  return source
    .is_valid_addition_for_capacity_inclusion(_input,
                                              source
                                                .delivery_in_range(_first_rank,
                                                                   _last_rank),
                                              _moved_jobs.begin(),
                                              _moved_jobs.end(),
                                              _first_rank,
                                              _last_rank);
}

void IntraRelocate::apply() {
  auto relocate_job_rank = s_route[s_rank];
  s_route.erase(s_route.begin() + s_rank);
  s_route.insert(t_route.begin() + t_rank, relocate_job_rank);

  source.update_amounts(_input);
}

std::vector<Index> IntraRelocate::addition_candidates() const {
  return {};
}

std::vector<Index> IntraRelocate::update_candidates() const {
  return {s_vehicle};
}

} // namespace cvrp
} // namespace vroom
