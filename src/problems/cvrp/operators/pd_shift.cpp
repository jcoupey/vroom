/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/pd_shift.h"
#include "utils/helpers.h"

namespace vroom {
namespace cvrp {

PDShift::PDShift(const Input& input,
                 const utils::SolutionState& sol_state,
                 RawRoute& s_route,
                 Index s_vehicle,
                 Index s_p_rank,
                 Index s_d_rank,
                 RawRoute& t_route,
                 Index t_vehicle,
                 Gain gain_threshold)
  : Operator(input, sol_state, s_route, s_vehicle, 0, t_route, t_vehicle, 0),
    _s_p_rank(s_p_rank),
    _s_d_rank(s_d_rank),
    _remove_gain(_sol_state.pd_gains[s_vehicle][_s_p_rank]),
    _valid(false) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 2);
  assert(s_p_rank < s_d_rank);
  assert(s_d_rank < s_route.size());

  stored_gain = gain_threshold;
}

void PDShift::compute_gain() {
  const auto& m = _input.get_matrix();
  const auto& v = _input.vehicles[t_vehicle];

  // For source vehicle, we consider the cost of removing P&D already
  // stored in remove_gain.

  // For target vehicle, we go through all insertion options for
  // pickup and delivery target ranks, storing best ranks along the
  // way.

  std::vector<Gain> t_d_gains(t_route.size() + 1);
  for (unsigned t_d_rank = 0; t_d_rank <= t_route.size(); ++t_d_rank) {
    t_d_gains[t_d_rank] = -utils::addition_cost(_input,
                                                m,
                                                s_route[_s_d_rank],
                                                v,
                                                t_route,
                                                t_d_rank);
  }

  for (unsigned t_p_rank = 0; t_p_rank < t_route.size(); ++t_p_rank) {
    Gain t_p_gain = -utils::addition_cost(_input,
                                          m,
                                          s_route[_s_p_rank],
                                          v,
                                          t_route,
                                          t_p_rank);

    if (_remove_gain + t_p_gain < stored_gain) {
      // Even without delivery insertion, the gain is lower than best
      // known stored gain.
      continue;
    }

    if (!target
           .is_valid_addition_for_load(_input,
                                       _input.jobs[s_route[_s_p_rank]].pickup,
                                       t_p_rank)) {
      continue;
    }

    std::vector<Index> modified_with_pd({s_route[_s_p_rank]});
    Amount modified_delivery = _input.zero_amount();

    for (unsigned t_d_rank = t_p_rank; t_d_rank <= t_route.size(); ++t_d_rank) {
      Gain target_gain;
      if (t_p_rank == t_d_rank) {
        target_gain = -utils::addition_cost(_input,
                                            m,
                                            s_route[_s_p_rank],
                                            v,
                                            t_route,
                                            t_p_rank,
                                            t_p_rank + 1);
      } else {
        target_gain = t_p_gain + t_d_gains[t_d_rank];
        modified_with_pd.push_back(t_route[t_d_rank - 1]);
        const auto& new_modified_job = _input.jobs[t_route[t_d_rank - 1]];
        if (new_modified_job.type == JOB_TYPE::SINGLE) {
          modified_delivery += new_modified_job.delivery;
        }
      }

      if (_remove_gain + target_gain > stored_gain) {
        modified_with_pd.push_back(s_route[_s_d_rank]);
        bool valid =
          target
            .is_valid_addition_for_capacity_inclusion(_input,
                                                      modified_delivery,
                                                      modified_with_pd.begin(),
                                                      modified_with_pd.end(),
                                                      t_p_rank,
                                                      t_d_rank);
        modified_with_pd.pop_back();

        if (valid) {
          _valid = true;
          stored_gain = _remove_gain + target_gain;
          _best_t_p_rank = t_p_rank;
          _best_t_d_rank = t_d_rank;
        }
      }
    }
  }

  gain_computed = true;
}

bool PDShift::is_valid() {
  assert(gain_computed);
  return _valid;
}

void PDShift::apply() {
  std::vector<Index> target_with_pd({s_route[_s_p_rank]});
  std::copy(t_route.begin() + _best_t_p_rank,
            t_route.begin() + _best_t_d_rank,
            std::back_inserter(target_with_pd));
  target_with_pd.push_back(s_route[_s_d_rank]);

  target.replace(_input,
                 target_with_pd.begin(),
                 target_with_pd.end(),
                 _best_t_p_rank,
                 _best_t_d_rank);

  if (_s_d_rank == _s_p_rank + 1) {
    s_route.erase(s_route.begin() + _s_p_rank, s_route.begin() + _s_p_rank + 2);
    source.update_amounts(_input);
  } else {
    std::vector<Index> source_without_pd(s_route.begin() + _s_p_rank + 1,
                                         s_route.begin() + _s_d_rank);
    source.replace(_input,
                   source_without_pd.begin(),
                   source_without_pd.end(),
                   _s_p_rank,
                   _s_d_rank + 1);
  }
}

std::vector<Index> PDShift::addition_candidates() const {
  return {s_vehicle};
}

std::vector<Index> PDShift::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace cvrp
} // namespace vroom
