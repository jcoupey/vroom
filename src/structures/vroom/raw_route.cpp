/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/raw_route.h"

namespace vroom {

RawRoute::RawRoute(const Input& input, Index i)
  : _fwd_peaks(2, input.zero_amount()),
    _bwd_peaks(2, input.zero_amount()),
    vehicle_rank(i),
    has_start(input.vehicles[i].has_start()),
    has_end(input.vehicles[i].has_end()),
    capacity(input.vehicles[i].capacity) {
}

void RawRoute::set_route(const Input& input, const std::vector<Index>& r) {
  route = r;
  update_amounts(input);
}

bool RawRoute::empty() const {
  return route.empty();
}

std::size_t RawRoute::size() const {
  return route.size();
}

void RawRoute::update_amounts(const Input& input) {
  auto step_size = route.size() + 2;
  _fwd_pickups.resize(route.size());
  _bwd_deliveries.resize(route.size());
  _pd_loads.resize(route.size());
  _nb_pickups.resize(route.size());
  _nb_deliveries.resize(route.size());

  _current_loads.resize(step_size);
  _fwd_peaks.resize(step_size);
  _bwd_peaks.resize(step_size);

  if (route.empty()) {
    // So that check in is_valid_addition_for_capacity is consistent
    // with empty routes.
    std::fill(_fwd_peaks.begin(), _fwd_peaks.end(), input.zero_amount());
    std::fill(_bwd_peaks.begin(), _bwd_peaks.end(), input.zero_amount());
    return;
  }

  Amount current_pickups(input.zero_amount());
  Amount current_pd_load(input.zero_amount());
  unsigned current_nb_pickups = 0;
  unsigned current_nb_deliveries = 0;

  for (std::size_t i = 0; i < route.size(); ++i) {
    const auto& job = input.jobs[route[i]];
    switch (job.type) {
    case JOB_TYPE::SINGLE:
      current_pickups += job.pickup;
      break;
    case JOB_TYPE::PICKUP:
      current_pd_load += job.pickup;
      current_nb_pickups += 1;
      break;
    case JOB_TYPE::DELIVERY:
      assert(job.delivery <= current_pd_load);
      current_pd_load -= job.delivery;
      current_nb_deliveries += 1;
      break;
    }
    _fwd_pickups[i] = current_pickups;
    _pd_loads[i] = current_pd_load;
    assert(current_nb_deliveries <= current_nb_pickups);
    _nb_pickups[i] = current_nb_pickups;
    _nb_deliveries[i] = current_nb_deliveries;
  }
  assert(_pd_loads.back() == input.zero_amount());

  Amount current_deliveries(input.zero_amount());

  _current_loads.back() = _fwd_pickups.back();

  for (std::size_t i = 0; i < route.size(); ++i) {
    auto bwd_i = route.size() - i - 1;

    _bwd_deliveries[bwd_i] = current_deliveries;
    _current_loads[bwd_i + 1] =
      _fwd_pickups[bwd_i] + _pd_loads[bwd_i] + current_deliveries;
    const auto& job = input.jobs[route[bwd_i]];
    if (job.type == JOB_TYPE::SINGLE) {
      current_deliveries += job.delivery;
    }
  }
  _current_loads[0] = current_deliveries;

  auto peak = _current_loads[0];
  _fwd_peaks[0] = peak;
  for (std::size_t s = 1; s < _fwd_peaks.size(); ++s) {
    // Handle max component-wise.
    for (std::size_t r = 0; r < input.zero_amount().size(); ++r) {
      peak[r] = std::max(peak[r], _current_loads[s][r]);
    }
    _fwd_peaks[s] = peak;
  }

  peak = _current_loads.back();
  _bwd_peaks.back() = peak;
  for (std::size_t s = 1; s < _bwd_peaks.size(); ++s) {
    auto bwd_s = _bwd_peaks.size() - s - 1;
    // Handle max component-wise.
    for (std::size_t r = 0; r < input.zero_amount().size(); ++r) {
      peak[r] = std::max(peak[r], _current_loads[bwd_s][r]);
    }
    _bwd_peaks[bwd_s] = peak;
  }
}

bool RawRoute::has_pending_delivery_after_rank(const Index rank) const {
  return _nb_deliveries[rank] < _nb_pickups[rank];
}

bool RawRoute::has_delivery_after_rank(const Index rank) const {
  assert(rank < _nb_deliveries.size());
  return _nb_deliveries[rank] < _nb_deliveries.back();
}

bool RawRoute::has_pickup_up_to_rank(const Index rank) const {
  assert(rank < _nb_pickups.size());
  return 0 < _nb_pickups[rank];
}

const Amount& RawRoute::max_load() const {
  return _fwd_peaks.back();
}

bool RawRoute::is_valid_addition_for_capacity(const Input&,
                                              const Amount& pickup,
                                              const Amount& delivery,
                                              const Index rank) const {
  assert(rank <= route.size());

  return (_fwd_peaks[rank] + delivery <= capacity) and
         (_bwd_peaks[rank] + pickup <= capacity);
}

bool RawRoute::is_valid_addition_for_load(const Input& input,
                                          const Amount& pickup,
                                          const Index rank) const {
  assert(rank <= route.size());

  auto& load = route.empty() ? input.zero_amount() : _current_loads[rank];
  return load + pickup <= capacity;
}

bool RawRoute::is_valid_addition_for_capacity_margins(
  const Input& input,
  const Amount& pickup,
  const Amount& delivery,
  const Index first_rank,
  const Index last_rank) const {
  assert(1 <= last_rank);
  assert(last_rank <= route.size() + 1);

  auto& first_deliveries =
    (first_rank == 0) ? _current_loads[0] : _bwd_deliveries[first_rank - 1];

  auto& first_pickups =
    (first_rank == 0) ? input.zero_amount() : _fwd_pickups[first_rank - 1];

  auto replaced_deliveries = first_deliveries - _bwd_deliveries[last_rank - 1];

  return (_fwd_peaks[first_rank] + delivery <=
          capacity + replaced_deliveries) and
         (_bwd_peaks[last_rank] + pickup <=
          capacity + _fwd_pickups[last_rank - 1] - first_pickups);
}

template <class InputIterator>
bool RawRoute::is_valid_addition_for_capacity_inclusion(
  const Input& input,
  const Amount& delivery,
  const InputIterator first_job,
  const InputIterator last_job,
  const Index first_rank,
  const Index last_rank) const {
  assert(first_rank <= last_rank);
  assert(last_rank <= route.size() + 1);

  auto& init_load = (route.empty()) ? input.zero_amount() : _current_loads[0];

  auto& first_deliveries =
    (first_rank == 0) ? init_load : _bwd_deliveries[first_rank - 1];

  auto& last_deliveries =
    (last_rank == 0) ? init_load : _bwd_deliveries[last_rank - 1];

  auto replaced_deliveries = first_deliveries - last_deliveries;

  Amount current_load =
    ((route.empty()) ? input.zero_amount() : _current_loads[first_rank]) -
    replaced_deliveries + delivery;

  bool valid = (current_load <= capacity);

  for (auto job_iter = first_job; valid and job_iter != last_job; ++job_iter) {
    auto& job = input.jobs[*job_iter];
    current_load += job.pickup;
    current_load -= job.delivery;

    valid = (current_load <= capacity);
  }

  return valid;
}

Amount RawRoute::get_startup_load() const {
  return _current_loads[0];
}

Amount RawRoute::pickup_in_range(Index i, Index j) const {
  if (i == j) {
    return Amount(_current_loads[0].size());
  }
  if (i == 0) {
    return _fwd_pickups[j - 1];
  } else {
    return _fwd_pickups[j - 1] - _fwd_pickups[i - 1];
  }
}

Amount RawRoute::delivery_in_range(Index i, Index j) const {
  if (i == j) {
    return Amount(_current_loads[0].size());
  }
  auto& before_deliveries =
    (i == 0) ? _current_loads[0] : _bwd_deliveries[i - 1];
  return before_deliveries - _bwd_deliveries[j - 1];
}

void RawRoute::add(const Input& input, const Index job_rank, const Index rank) {
  route.insert(route.begin() + rank, job_rank);
  update_amounts(input);
}

void RawRoute::remove(const Input& input,
                      const Index rank,
                      const unsigned count) {
  route.erase(route.begin() + rank, route.begin() + rank + count);
  update_amounts(input);
}

template <class InputIterator>
void RawRoute::replace(const Input& input,
                       const InputIterator first_job,
                       const InputIterator last_job,
                       const Index first_rank,
                       const Index last_rank) {
  assert(first_rank <= last_rank);

  route.erase(route.begin() + first_rank, route.begin() + last_rank);
  route.insert(route.begin() + first_rank, first_job, last_job);

  update_amounts(input);
}

template bool RawRoute::is_valid_addition_for_capacity_inclusion(
  const Input& input,
  const Amount& delivery,
  const std::vector<Index>::iterator first_job,
  const std::vector<Index>::iterator last_job,
  const Index first_rank,
  const Index last_rank) const;
template bool RawRoute::is_valid_addition_for_capacity_inclusion(
  const Input& input,
  const Amount& delivery,
  const std::vector<Index>::reverse_iterator first_job,
  const std::vector<Index>::reverse_iterator last_job,
  const Index first_rank,
  const Index last_rank) const;
template void RawRoute::replace(const Input& input,
                                std::vector<Index>::iterator first_job,
                                std::vector<Index>::iterator last_job,
                                const Index first_rank,
                                const Index last_rank);

} // namespace vroom
