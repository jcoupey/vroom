#ifndef TW_ROUTE_H
#define TW_ROUTE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <numeric>
#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/raw_route.h"

namespace vroom {

struct OrderChoice {
  const Input& input;
  Index job_rank;
  bool add_job_first;
  bool add_break_first;
  const std::vector<TimeWindow>::const_iterator j_tw;
  const std::vector<TimeWindow>::const_iterator b_tw;

  OrderChoice(const Input& input,
              Index job_rank,
              const Break& b,
              const Duration current_earliest,
              const Duration previous_travel);
};

class TWRoute : public RawRoute {
private:
  // When inserting job at job_rank in route at rank, retrieve
  // earliest end date (resp. latest start date) for previous
  // (resp. next) step. Last argument holds the travel time from
  // (resp. to) that step.
  Duration previous_earliest_end(const Input& input,
                                 Index job_rank,
                                 Index rank,
                                 Duration& previous_travel) const;
  Duration next_latest_start(const Input& input,
                             Index job_rank,
                             Index rank,
                             Duration& next_travel) const;

  void fwd_update_earliest_from(const Input& input, Index rank);
  void bwd_update_latest_from(const Input& input, Index rank);

  // Define global policy wrt job/break respective insertion rule.
  OrderChoice order_choice(const Input& input,
                           Index job_rank,
                           const Break& b,
                           const Duration current_earliest,
                           const Duration previous_travel,
                           const Duration next_travel,
                           const Duration next_start) const;

public:
  Duration v_start;
  Duration v_end;

  // Margin for job at rank i in route: earliest[i] and latest[i]
  // store earliest and latest date. Those are potentially derived
  // from different time windows in multiple TW situations.
  std::vector<Duration> earliest;
  std::vector<Duration> latest;

  // Store earliest date for route end.
  Duration earliest_end;

  // A vector with route.size() + 1 elements. breaks_at_rank[i] is the
  // number of breaks that are to be taken right before job at
  // route[i]. breaks_at_rank[route.size()] is the number of breaks
  // right before the end of the route. breaks_counts holds the
  // cumulated values for breaks_at_rank.
  std::vector<unsigned> breaks_at_rank;
  std::vector<unsigned> breaks_counts;

  // break_earliest[i] and break_latest[i] store earliest and latest
  // date for break at rank i in vehicle breaks.
  std::vector<Duration> break_earliest;
  std::vector<Duration> break_latest;

  // When a break's earliest date is delayed (resp. latest date is
  // advanced) because of it's time window start (resp. end), then
  // some amount of time before (resp. after) can be used for travel.
  std::vector<Duration> breaks_travel_margin_before;
  std::vector<Duration> breaks_travel_margin_after;

  TWRoute(const Input& input, Index i);

  bool empty() const {
    return route.empty();
  }

  std::size_t size() const {
    return route.size();
  }

  // Check validity for addition of job at job_rank in current route
  // at rank.
  bool is_valid_addition_for_tw(const Input& input,
                                const Index job_rank,
                                const Index rank) const;

  // Check validity for inclusion of the range [first_job; last_job)
  // in the existing route at rank first_rank and before last_rank *in
  // place of* the current jobs that may be there.
  template <class InputIterator>
  bool is_valid_addition_for_tw(const Input& input,
                                const InputIterator first_job,
                                const InputIterator last_job,
                                const Index first_rank,
                                const Index last_rank) const;

  void add(const Input& input, const Index job_rank, const Index rank);

  // Check validity for removing a set of jobs from current route at
  // rank. Required because removing a job can actually lead to an
  // invalid solution (see #172).
  bool is_valid_removal(const Input& input,
                        const Index rank,
                        const unsigned count) const;

  void remove(const Input& input, const Index rank, const unsigned count);

  // Add the range [first_job; last_job) in the existing route at rank
  // first_rank and before last_rank *in place of* the current jobs
  // that may be there.
  template <class InputIterator>
  void replace(const Input& input,
               const InputIterator first_job,
               const InputIterator last_job,
               const Index first_rank,
               const Index last_rank);
};

} // namespace vroom

#endif
