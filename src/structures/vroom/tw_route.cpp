/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>

#include "structures/vroom/tw_route.h"
#include "utils/exception.h"

namespace vroom {

TWRoute::TWRoute(const Input& input, Index v)
  : RawRoute(input, v),
    v_start(input.vehicles[v].tw.start),
    v_end(input.vehicles[v].tw.end),
    breaks_at_rank({static_cast<unsigned>(input.vehicles[v].breaks.size())}),
    breaks_counts({static_cast<unsigned>(input.vehicles[v].breaks.size())}),
    break_earliest(input.vehicles[v].breaks.size()),
    break_latest(input.vehicles[v].breaks.size()),
    breaks_travel_margin_before(input.vehicles[v].breaks.size()),
    breaks_travel_margin_after(input.vehicles[v].breaks.size()) {
  std::string break_error = "Inconsistent breaks for vehicle " +
                            std::to_string(input.vehicles[v].id) + ".";

  const auto& breaks = input.vehicles[v].breaks;

  Duration previous_earliest = v_start;

  for (Index i = 0; i < breaks.size(); ++i) {
    const auto& b = breaks[i];
    const auto b_tw =
      std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
        return previous_earliest <= tw.end;
      });
    if (b_tw == b.tws.end()) {
      throw Exception(ERROR::INPUT, break_error);
    }

    break_earliest[i] = std::max(previous_earliest, b_tw->start);
    breaks_travel_margin_before[i] = break_earliest[i] - previous_earliest;

    previous_earliest = break_earliest[i] + b.service;
  }

  Duration next_latest = v_end;
  for (Index r_i = 0; r_i < breaks.size(); ++r_i) {
    Index i = breaks.size() - 1 - r_i;
    const auto& b = breaks[i];

    if (next_latest < b.service) {
      throw Exception(ERROR::INPUT, break_error);
    }
    next_latest -= b.service;

    const auto b_tw =
      std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
        return tw.start <= next_latest;
      });
    if (b_tw == b.tws.rend()) {
      throw Exception(ERROR::INPUT, break_error);
    }

    break_latest[i] = std::min(next_latest, b_tw->end);
    breaks_travel_margin_after[i] = next_latest - break_latest[i];

    next_latest = break_latest[i];

    if (break_latest[i] < break_earliest[i]) {
      throw Exception(ERROR::INPUT, break_error);
    }
  }
}

Duration TWRoute::previous_earliest_end(const Input& input,
                                        Index job_rank,
                                        Index rank,
                                        Duration& previous_travel) const {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];
  const auto& j = input.jobs[job_rank];

  Duration previous_earliest = v_start;
  Duration previous_service = 0;
  if (rank > 0) {
    const auto& previous_job = input.jobs[route[rank - 1]];
    previous_earliest = earliest[rank - 1];
    previous_service = previous_job.service;
    previous_travel = m[previous_job.index()][j.index()];
  } else {
    if (has_start) {
      previous_travel = m[v.start.value().index()][j.index()];
    }
  }

  return previous_earliest + previous_service;
}

Duration TWRoute::next_latest_start(const Input& input,
                                    Index job_rank,
                                    Index rank,
                                    Duration& next_travel) const {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];
  const auto& j = input.jobs[job_rank];

  Duration next_latest = v_end;
  if (rank == route.size()) {
    if (has_end) {
      next_travel = m[j.index()][v.end.value().index()];
    }
  } else {
    next_latest = latest[rank];
    next_travel = m[j.index()][input.jobs[route[rank]].index()];
  }

  return next_latest;
}

void TWRoute::fwd_update_earliest_from(const Input& input, Index rank) {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];

  Duration current_earliest = earliest[rank];
  bool handle_last_breaks = true;

  for (Index i = rank + 1; i < route.size(); ++i) {
    const auto& next_j = input.jobs[route[i]];
    Duration remaining_travel_time =
      m[input.jobs[route[i - 1]].index()][next_j.index()];
    Duration previous_service = input.jobs[route[i - 1]].service;

    // Update earliest dates and margins for breaks.
    assert(breaks_at_rank[i] <= breaks_counts[i]);
    Index break_rank = breaks_counts[i] - breaks_at_rank[i];

    for (Index r = 0; r < breaks_at_rank[i]; ++r, ++break_rank) {
      const auto& b = v.breaks[break_rank];

      current_earliest += previous_service;

      const auto b_tw =
        std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
          return current_earliest <= tw.end;
        });
      assert(b_tw != b.tws.end());

      if (current_earliest < b_tw->start) {
        auto margin = b_tw->start - current_earliest;
        breaks_travel_margin_before[break_rank] = margin;
        if (margin < remaining_travel_time) {
          remaining_travel_time -= margin;
        } else {
          remaining_travel_time = 0;
        }

        current_earliest = b_tw->start;
      } else {
        breaks_travel_margin_before[break_rank] = 0;
      }

      break_earliest[break_rank] = current_earliest;
      previous_service = v.breaks[break_rank].service;
    }

    // Back to the job after breaks.
    current_earliest += previous_service + remaining_travel_time;

    const auto j_tw =
      std::find_if(next_j.tws.begin(), next_j.tws.end(), [&](const auto& tw) {
        return current_earliest <= tw.end;
      });
    assert(j_tw != next_j.tws.end());

    current_earliest = std::max(current_earliest, j_tw->start);

    assert(current_earliest <= latest[i]);
    if (current_earliest == earliest[i]) {
      // There won't be any further update so stop earliest date
      // propagation.
      handle_last_breaks = false;
      break;
    }

    earliest[i] = current_earliest;
  }

  if (handle_last_breaks) {
    // Update earliest dates and margins for potential breaks right
    // before route end.
    Index i = route.size();
    Duration remaining_travel_time =
      (v.has_end()) ? m[input.jobs[route[i - 1]].index()][v.end.value().index()]
                    : 0;

    Duration previous_service = input.jobs[route[i - 1]].service;

    assert(breaks_at_rank[i] <= breaks_counts[i]);
    Index break_rank = breaks_counts[i] - breaks_at_rank[i];

    for (Index r = 0; r < breaks_at_rank[i]; ++r, ++break_rank) {
      const auto& b = v.breaks[break_rank];
      current_earliest += previous_service;

      const auto b_tw =
        std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
          return current_earliest <= tw.end;
        });
      assert(b_tw != b.tws.end());

      if (current_earliest < b_tw->start) {
        auto margin = b_tw->start - current_earliest;
        breaks_travel_margin_before[break_rank] = margin;
        if (margin < remaining_travel_time) {
          remaining_travel_time -= margin;
        } else {
          remaining_travel_time = 0;
        }

        current_earliest = b_tw->start;
      } else {
        breaks_travel_margin_before[break_rank] = 0;
      }

      break_earliest[break_rank] = current_earliest;
      previous_service = v.breaks[break_rank].service;
    }

    earliest_end = current_earliest + previous_service + remaining_travel_time;
    assert(earliest_end <= v_end);
  }
}

void TWRoute::bwd_update_latest_from(const Input& input, Index rank) {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];

  Duration current_latest = latest[rank];
  bool handle_first_breaks = true;

  for (Index next_i = rank; next_i > 0; --next_i) {
    const auto& previous_j = input.jobs[route[next_i - 1]];
    Duration remaining_travel_time =
      m[previous_j.index()][input.jobs[route[next_i]].index()];

    // Update latest dates and margins for breaks.
    assert(breaks_at_rank[next_i] <= breaks_counts[next_i]);
    Index break_rank = breaks_counts[next_i];

    for (Index r = 0; r < breaks_at_rank[next_i]; ++r) {
      --break_rank;

      const auto& b = v.breaks[break_rank];
      assert(b.service <= current_latest);
      current_latest -= b.service;

      const auto b_tw =
        std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
          return tw.start <= current_latest;
        });
      assert(b_tw != b.tws.rend());

      if (b_tw->end < current_latest) {
        auto margin = current_latest - b_tw->end;
        breaks_travel_margin_after[break_rank] = margin;
        if (margin < remaining_travel_time) {
          remaining_travel_time -= margin;
        } else {
          remaining_travel_time = 0;
        }

        current_latest = b_tw->end;
      } else {
        breaks_travel_margin_after[break_rank] = 0;
      }

      break_latest[break_rank] = current_latest;
    }

    // Back to the job after breaks.
    auto gap = previous_j.service + remaining_travel_time;
    assert(gap <= current_latest);
    current_latest -= gap;

    const auto j_tw =
      std::find_if(previous_j.tws.rbegin(),
                   previous_j.tws.rend(),
                   [&](const auto& tw) { return tw.start <= current_latest; });
    assert(j_tw != previous_j.tws.rend());

    current_latest = std::min(current_latest, j_tw->end);

    assert(earliest[next_i - 1] <= current_latest);
    if (current_latest == latest[next_i - 1]) {
      // There won't be any further update so stop latest date
      // propagation.
      handle_first_breaks = false;
      break;
    }

    latest[next_i - 1] = current_latest;
  }

  if (handle_first_breaks) {
    // Update latest dates and margins for breaks right before the
    // first job.
    Index next_i = 0;

    assert(breaks_at_rank[next_i] <= breaks_counts[next_i]);
    Index break_rank = breaks_counts[next_i];

    for (Index r = 0; r < breaks_at_rank[next_i]; ++r) {
      --break_rank;
      const auto& b = v.breaks[break_rank];

      assert(b.service <= current_latest);
      current_latest -= b.service;

      const auto b_tw =
        std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
          return tw.start <= current_latest;
        });
      assert(b_tw != b.tws.rend());
      if (b_tw->end < current_latest) {
        breaks_travel_margin_after[break_rank] = current_latest - b_tw->end;

        current_latest = b_tw->end;
      } else {
        breaks_travel_margin_after[break_rank] = 0;
      }

      break_latest[break_rank] = current_latest;
    }
  }
}

OrderChoice::OrderChoice(const Input& input,
                         Index job_rank,
                         const Break& b,
                         const Duration current_earliest,
                         const Duration previous_travel)
  : input(input),
    job_rank(job_rank),
    add_job_first(false),
    add_break_first(false),
    j_tw(std::find_if(input.jobs[job_rank].tws.begin(),
                      input.jobs[job_rank].tws.end(),
                      [&](const auto& tw) {
                        return current_earliest + previous_travel <= tw.end;
                      })),
    b_tw(std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
      return current_earliest <= tw.end;
    })) {
}

OrderChoice TWRoute::order_choice(const Input& input,
                                  Index job_rank,
                                  const Break& b,
                                  const Duration current_earliest,
                                  const Duration previous_travel,
                                  const Duration next_travel,
                                  const Duration next_start) const {
  OrderChoice oc(input, job_rank, b, current_earliest, previous_travel);
  const auto& j = input.jobs[job_rank];

  if (oc.j_tw == j.tws.end() or oc.b_tw == b.tws.end()) {
    // If either job or break can't fit first, then none of the
    // orderings are valid.
    return oc;
  }

  Duration job_then_break_end;
  Duration break_then_job_end;

  // Try putting job first then break.
  const Duration earliest_job_end =
    std::max(current_earliest + previous_travel, oc.j_tw->start) + j.service;
  Duration job_then_break_margin = 0;

  const auto new_b_tw =
    std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
      return earliest_job_end <= tw.end;
    });
  if (new_b_tw == b.tws.end()) {
    // Break does not fit after job due to its time windows. Only
    // option is to choose break first.
    oc.add_break_first = true;
    return oc;
  } else {
    Duration travel_after_break = next_travel;
    if (earliest_job_end < new_b_tw->start) {
      job_then_break_margin = new_b_tw->start - earliest_job_end;
      if (job_then_break_margin < travel_after_break) {
        travel_after_break -= job_then_break_margin;
      } else {
        travel_after_break = 0;
      }

      job_then_break_end = oc.b_tw->start + b.service;
    } else {
      job_then_break_end = earliest_job_end + b.service;
    }

    if (job_then_break_end + travel_after_break > next_start) {
      // Starting the break is possible but then next step is not.
      oc.add_break_first = true;
      return oc;
    }
  }

  // Try putting break first then job.
  Duration travel_after_break = previous_travel;
  Duration earliest_job_start = current_earliest;

  if (current_earliest < oc.b_tw->start) {
    auto margin = oc.b_tw->start - current_earliest;
    if (margin < travel_after_break) {
      travel_after_break -= margin;
    } else {
      travel_after_break = 0;
    }

    earliest_job_start = oc.b_tw->start;
  }

  earliest_job_start += b.service + travel_after_break;

  const auto new_j_tw =
    std::find_if(j.tws.begin(), j.tws.end(), [&](const auto& tw) {
      return earliest_job_start <= tw.end;
    });
  if (new_j_tw == j.tws.end()) {
    // Job does not fit after break due to its time windows. Only
    // option is to choose job first.
    oc.add_job_first = true;
    return oc;
  } else {
    break_then_job_end =
      std::max(earliest_job_start, new_j_tw->start) + j.service;

    if (break_then_job_end + next_travel > next_start) {
      // Arrival at the job is valid but next step is not.
      oc.add_job_first = true;
      return oc;
    }
  }

  // In case where both ordering options are doable based on timing
  // constraints for a pickup, we favor putting the pickup first,
  // except if adding the delivery afterwards is not possible. This is
  // mandatory to avoid heuristically forcing a pickup -> break choice
  // resulting in invalid options, while break -> pickup -> delivery
  // might be valid.
  if (j.type == JOB_TYPE::PICKUP) {
    const auto& matching_d = input.jobs[job_rank + 1];
    assert(matching_d.type == JOB_TYPE::DELIVERY);
    const auto& m = input.get_matrix();

    // Try pickup -> break -> delivery.
    auto delivery_travel = m[j.index()][matching_d.index()];
    if (job_then_break_margin < delivery_travel) {
      delivery_travel -= job_then_break_margin;
    } else {
      delivery_travel = 0;
    }
    const Duration pb_d_candidate = job_then_break_end + delivery_travel;
    const auto pb_d_tw =
      std::find_if(matching_d.tws.begin(),
                   matching_d.tws.end(),
                   [&](const auto& tw) { return pb_d_candidate <= tw.end; });
    if (pb_d_tw != matching_d.tws.end()) {
      // pickup -> break -> delivery is doable, choose pickup first.
      oc.add_job_first = true;
      return oc;
    }

    // Previous order not doable, so try pickup -> delivery -> break.
    const Duration delivery_candidate =
      earliest_job_end + m[j.index()][matching_d.index()];
    const auto d_tw = std::find_if(matching_d.tws.begin(),
                                   matching_d.tws.end(),
                                   [&](const auto& tw) {
                                     return delivery_candidate <= tw.end;
                                   });

    if (d_tw != matching_d.tws.end()) {
      const Duration break_candidate =
        std::max(delivery_candidate, d_tw->start) + matching_d.service;

      const auto after_d_b_tw =
        std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
          return break_candidate <= tw.end;
        });
      if (after_d_b_tw != b.tws.end()) {
        // pickup -> delivery -> break is doable, choose pickup first.
        oc.add_job_first = true;
        return oc;
      }
    }

    // Doing pickup first actually leads to infeasible options, so put
    // break first.
    oc.add_break_first = true;
    return oc;
  }

  // In case where both ordering options are doable based on timing
  // constraints for a single job, we pick the ordering minimizing
  // earliest end date for sequence.
  if (break_then_job_end < job_then_break_end) {
    oc.add_break_first = true;
  } else if (break_then_job_end == job_then_break_end) {
    // If end date is the same for both ordering options, decide
    // based on earliest deadline.
    if (oc.j_tw->end <= oc.b_tw->end) {
      oc.add_job_first = true;
    } else {
      oc.add_break_first = true;
    }
  } else {
    oc.add_job_first = true;
  }

  return oc;
}

bool TWRoute::is_valid_addition_for_tw(const Input& input,
                                       const Index job_rank,
                                       const Index rank) const {
  const auto& v = input.vehicles[vehicle_rank];
  const auto& j = input.jobs[job_rank];

  Duration previous_travel = 0;
  Duration current_earliest =
    previous_earliest_end(input, job_rank, rank, previous_travel);

  Duration next_travel = 0;
  Duration next_start = next_latest_start(input, job_rank, rank, next_travel);

  bool job_added = false;

  assert(breaks_at_rank[rank] <= breaks_counts[rank]);

  // Determine break range.
  Index current_break = breaks_counts[rank] - breaks_at_rank[rank];
  const Index last_break = breaks_counts[rank];

  while (!job_added or current_break != last_break) {
    if (current_break == last_break) {
      // Done with all breaks and job not added yet.
      current_earliest += previous_travel;
      const auto j_tw =
        std::find_if(j.tws.begin(), j.tws.end(), [&](const auto& tw) {
          return current_earliest <= tw.end;
        });
      if (j_tw == j.tws.end()) {
        return false;
      }

      current_earliest = std::max(current_earliest, j_tw->start) + j.service;
      break;
    }

    // There is actually a break to consider.
    const auto& b = v.breaks[current_break];

    if (job_added) {
      // Compute earliest end date for current break.
      const auto b_tw =
        std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
          return current_earliest <= tw.end;
        });

      if (b_tw == b.tws.end()) {
        // Break does not fit due to its time windows.
        return false;
      }

      if (current_earliest < b_tw->start) {
        auto margin = b_tw->start - current_earliest;
        if (margin < next_travel) {
          next_travel -= margin;
        } else {
          next_travel = 0;
        }

        current_earliest = b_tw->start;
      }

      current_earliest += b.service;

      ++current_break;
      continue;
    }

    // Decide on ordering between break and added job.
    auto oc = order_choice(input,
                           job_rank,
                           b,
                           current_earliest,
                           previous_travel,
                           next_travel,
                           next_start);

    if (!oc.add_job_first and !oc.add_break_first) {
      // Infeasible insertion.
      return false;
    }

    // Feasible insertion based on time windows, now update next end
    // time with given insertion choice.
    assert(oc.add_job_first xor oc.add_break_first);
    if (oc.add_break_first) {
      if (current_earliest < oc.b_tw->start) {
        auto margin = oc.b_tw->start - current_earliest;
        if (margin < previous_travel) {
          previous_travel -= margin;
        } else {
          previous_travel = 0;
        }

        current_earliest = oc.b_tw->start;
      }

      current_earliest += b.service;

      ++current_break;
    }
    if (oc.add_job_first) {
      current_earliest =
        std::max(current_earliest + previous_travel, oc.j_tw->start) +
        j.service;
      job_added = true;
    }
  }

  return current_earliest + next_travel <= next_start;
}

template <class InputIterator>
bool TWRoute::is_valid_addition_for_tw(const Input& input,
                                       const InputIterator first_job,
                                       const InputIterator last_job,
                                       const Index first_rank,
                                       const Index last_rank) const {
  assert(first_job <= last_job);
  assert(first_rank <= last_rank);

  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];

  Duration previous_travel = 0;
  Duration current_earliest;

  Duration next_travel = 0;
  Duration current_latest;

  // Value initialization differ whether there are actually jobs added
  // or not.
  if (first_job < last_job) {
    current_earliest =
      previous_earliest_end(input, *first_job, first_rank, previous_travel);

    current_latest =
      next_latest_start(input, *(last_job - 1), last_rank, next_travel);
  } else {
    // This is actually a removal as no jobs are inserted.
    current_earliest = v_start;
    current_latest = v_end;

    if (first_rank > 0) {
      const auto& previous_job = input.jobs[route[first_rank - 1]];
      current_earliest = earliest[first_rank - 1] + previous_job.service;

      if (last_rank < route.size()) {
        current_latest = latest[last_rank];
        next_travel =
          m[previous_job.index()][input.jobs[route[last_rank]].index()];
      } else {
        if (has_end) {
          next_travel = m[previous_job.index()][v.end.value().index()];
        }
      }
    } else {
      if (last_rank < route.size()) {
        current_latest = latest[last_rank];
        if (has_start) {
          next_travel =
            m[v.start.value().index()][input.jobs[route[last_rank]].index()];
        }
      } else {
        // Emptying the whole route is valid.
        return true;
      }
    }
  }

  // Determine break range between first_rank and last_rank.
  Index current_break = breaks_counts[first_rank] - breaks_at_rank[first_rank];
  const Index last_break = breaks_counts[last_rank];

  // Propagate earliest dates for all jobs and breaks in their
  // respective addition ranges.
  auto current_job = first_job;
  while (current_job != last_job or current_break != last_break) {
    if (current_job == last_job) {
      // Compute earliest end date for break after last inserted jobs.
      const auto& b = v.breaks[current_break];

      const auto b_tw =
        std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
          return current_earliest <= tw.end;
        });

      if (b_tw == b.tws.end()) {
        // Break does not fit due to its time windows.
        return false;
      }

      if (current_earliest < b_tw->start) {
        auto margin = b_tw->start - current_earliest;
        if (margin < next_travel) {
          next_travel -= margin;
        } else {
          next_travel = 0;
        }

        current_earliest = b_tw->start;
      }

      current_earliest += b.service;

      ++current_break;
      continue;
    }

    if (current_break == last_break) {
      // Compute earliest end date for job after last inserted breaks.
      const auto& j = input.jobs[*current_job];

      current_earliest += previous_travel;

      const auto j_tw =
        std::find_if(j.tws.begin(), j.tws.end(), [&](const auto& tw) {
          return current_earliest <= tw.end;
        });
      if (j_tw == j.tws.end()) {
        return false;
      }

      current_earliest = std::max(current_earliest, j_tw->start) + j.service;

      ++current_job;
      if (current_job != last_job) {
        // Account for travel time to next current job.
        previous_travel = m[j.index()][input.jobs[*current_job].index()];
      }
      continue;
    }

    // We still have both jobs and breaks to go through, so decide on
    // ordering.
    const auto& b = v.breaks[current_break];
    const auto& j = input.jobs[*current_job];

    auto oc = order_choice(input,
                           *current_job,
                           b,
                           current_earliest,
                           previous_travel,
                           next_travel,
                           current_latest);

    if (!oc.add_job_first and !oc.add_break_first) {
      // Infeasible insertion.
      return false;
    }

    // Feasible insertion based on time windows, now update next end
    // time with given insertion choice.
    assert(oc.add_job_first xor oc.add_break_first);
    if (oc.add_break_first) {
      if (current_earliest < oc.b_tw->start) {
        auto margin = oc.b_tw->start - current_earliest;
        if (margin < previous_travel) {
          previous_travel -= margin;
        } else {
          previous_travel = 0;
        }

        current_earliest = oc.b_tw->start;
      }

      current_earliest += b.service;

      ++current_break;
    }
    if (oc.add_job_first) {
      current_earliest =
        std::max(current_earliest + previous_travel, oc.j_tw->start) +
        j.service;

      ++current_job;
      if (current_job != last_job) {
        // Account for travel time to next current job.
        previous_travel = m[j.index()][input.jobs[*current_job].index()];
      }
    }
  }

  return current_earliest + next_travel <= current_latest;
}

void TWRoute::add(const Input& input, const Index job_rank, const Index rank) {
  assert(rank <= route.size());

  const auto& v = input.vehicles[vehicle_rank];
  const auto& j = input.jobs[job_rank];

  Duration previous_travel = 0;
  Duration current_earliest =
    previous_earliest_end(input, job_rank, rank, previous_travel);

  Duration next_travel = 0;
  Duration current_latest =
    next_latest_start(input, job_rank, rank, next_travel);

  bool job_added = false;
  unsigned breaks_before = 0;

  assert(breaks_at_rank[rank] <= breaks_counts[rank]);

  for (Index r = 0; r < breaks_at_rank[rank]; ++r) {
    Index break_rank = (breaks_counts[rank] + r) - breaks_at_rank[rank];
    const auto& b = v.breaks[break_rank];

    // Decide on ordering between break and added job.
    auto oc = order_choice(input,
                           job_rank,
                           b,
                           current_earliest,
                           previous_travel,
                           next_travel,
                           current_latest);
    assert(oc.add_job_first xor oc.add_break_first);

    // Now update next end time based on insertion choice.
    if (oc.add_break_first) {
      // Earliest/rank/margin data for breaks before added job is
      // unchanged.
      ++breaks_before;

      if (breaks_travel_margin_before[break_rank] < previous_travel) {
        previous_travel -= breaks_travel_margin_before[break_rank];
      } else {
        previous_travel = 0;
      }

      current_earliest = break_earliest[break_rank] + b.service;
    }
    if (oc.add_job_first) {
      assert(oc.j_tw != j.tws.end());
      current_earliest =
        std::max(current_earliest + previous_travel, oc.j_tw->start);

      earliest.insert(earliest.begin() + rank, current_earliest);

      current_earliest += j.service;
      job_added = true;

      // Earliest/rank data for breaks after added job will be updated
      // through the call to fwd_update_earliest_from one break counts
      // are straightened up.
      break;
    }
  }

  if (!job_added) {
    current_earliest += previous_travel;
    const auto j_tw =
      std::find_if(j.tws.begin(), j.tws.end(), [&](const auto& tw) {
        return current_earliest <= tw.end;
      });
    assert(j_tw != j.tws.end());

    current_earliest = std::max(current_earliest, j_tw->start);

    earliest.insert(earliest.begin() + rank, current_earliest);
  }

  // Add breaks_* data for breaks before newly inserted job.
  unsigned breaks_after = breaks_at_rank[rank] - breaks_before;
  breaks_at_rank[rank] -= breaks_before;
  breaks_at_rank.insert(breaks_at_rank.begin() + rank, breaks_before);
  breaks_counts.insert(breaks_counts.begin() + rank,
                       breaks_counts[rank] - breaks_after);

  // Update latest dates for breaks after inserted job using updated
  // break data.
  Index break_rank = breaks_counts[rank + 1];
  for (Index r = 0; r < breaks_after; ++r) {
    --break_rank;
    const auto& b = v.breaks[break_rank];

    assert(b.service <= current_latest);
    current_latest -= b.service;

    const auto b_tw =
      std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
        return tw.start <= current_latest;
      });
    assert(b_tw != b.tws.rend());

    if (b_tw->end < current_latest) {
      auto margin = current_latest - b_tw->end;
      breaks_travel_margin_after[break_rank] = margin;
      if (margin < next_travel) {
        next_travel -= margin;
      } else {
        next_travel = 0;
      }

      current_latest = b_tw->end;
    } else {
      breaks_travel_margin_after[break_rank] = 0;
    }

    break_latest[break_rank] = current_latest;
  }

  // Update inserted job latest date.
  assert(j.service + next_travel <= current_latest);
  current_latest -= (j.service + next_travel);

  const auto j_tw =
    std::find_if(j.tws.rbegin(), j.tws.rend(), [&](const auto& tw) {
      return tw.start <= current_latest;
    });
  assert(j_tw != j.tws.rend());

  current_latest = std::min(current_latest, j_tw->end);
  latest.insert(latest.begin() + rank, current_latest);

  // Updating the route needs to be done after TW stuff as
  // previous_earliest_end and next_latest_start rely on route size
  // before addition ; but before earliest/latest date propagation
  // that rely on route structure after addition.
  route.insert(route.begin() + rank, job_rank);

  fwd_update_earliest_from(input, rank);
  bwd_update_latest_from(input, rank);

  update_amounts(input);
}

bool TWRoute::is_valid_removal(const Input& input,
                               const Index rank,
                               const unsigned count) const {
  assert(!route.empty());
  assert(rank + count <= route.size());

  return is_valid_addition_for_tw(input,
                                  route.begin(),
                                  route.begin(),
                                  rank,
                                  rank + count);
}

void TWRoute::remove(const Input& input,
                     const Index rank,
                     const unsigned count) {
  assert(rank + count <= route.size());

  replace(input, route.begin(), route.begin(), rank, rank + count);
}

template <class InputIterator>
void TWRoute::replace(const Input& input,
                      const InputIterator first_job,
                      const InputIterator last_job,
                      const Index first_rank,
                      const Index last_rank) {
  assert(first_job <= last_job);
  assert(first_rank <= last_rank);

  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];

  Duration previous_travel = 0;
  Duration current_earliest;

  Duration next_travel = 0;
  Duration current_latest;

  // Value initialization differ whether there are actually jobs added
  // or not.
  if (first_job < last_job) {
    current_earliest =
      previous_earliest_end(input, *first_job, first_rank, previous_travel);

    current_latest =
      next_latest_start(input, *(last_job - 1), last_rank, next_travel);
  } else {
    // This is actually a removal as no jobs are inserted.
    current_earliest = v_start;
    current_latest = v_end;

    if (first_rank > 0) {
      const auto& previous_job = input.jobs[route[first_rank - 1]];
      current_earliest = earliest[first_rank - 1] + previous_job.service;

      if (last_rank < route.size()) {
        current_latest = latest[last_rank];
        next_travel =
          m[previous_job.index()][input.jobs[route[last_rank]].index()];
      } else {
        if (has_end) {
          next_travel = m[previous_job.index()][v.end.value().index()];
        }
      }
    } else {
      if (last_rank < route.size()) {
        current_latest = latest[last_rank];
        if (has_start) {
          next_travel =
            m[v.start.value().index()][input.jobs[route[last_rank]].index()];
        }
      }
    }
  }

  Duration bwd_next_travel = next_travel; // Used for latest dates.

  // Determine break range between first_rank and last_rank.
  Index current_break = breaks_counts[first_rank] - breaks_at_rank[first_rank];
  const Index last_break = breaks_counts[last_rank];

  unsigned previous_breaks_counts =
    (first_rank != 0) ? breaks_counts[first_rank - 1] : 0;

  // Adjust various vector sizes. Dummy inserted values and unmodified
  // old values in the insertion range will be overwritten below.
  const unsigned erase_count = last_rank - first_rank;
  const unsigned add_count = std::distance(first_job, last_job);

  // Update data structures. For earliest and latest dates, we need to
  // overwrite old values. Otherwise they may happen to be identical
  // to new computed values and stop propagation inside
  // fwd_update_earliest_from and bwd_update_latest_from below.
  if (add_count < erase_count) {
    auto to_erase = erase_count - add_count;
    route.erase(route.begin() + first_rank,
                route.begin() + first_rank + to_erase);
    earliest.erase(earliest.begin() + first_rank,
                   earliest.begin() + first_rank + to_erase);
    latest.erase(latest.begin() + first_rank,
                 latest.begin() + first_rank + to_erase);
    breaks_at_rank.erase(breaks_at_rank.begin() + first_rank,
                         breaks_at_rank.begin() + first_rank + to_erase);
    breaks_counts.erase(breaks_counts.begin() + first_rank,
                        breaks_counts.begin() + first_rank + to_erase);

    std::fill(earliest.begin() + first_rank,
              earliest.begin() + first_rank + add_count,
              std::numeric_limits<Duration>::max());
    std::fill(latest.begin() + first_rank,
              latest.begin() + first_rank + add_count,
              0);
  } else {
    std::fill(earliest.begin() + first_rank,
              earliest.begin() + first_rank + erase_count,
              std::numeric_limits<Duration>::max());
    std::fill(latest.begin() + first_rank,
              latest.begin() + first_rank + erase_count,
              0);

    auto to_insert = add_count - erase_count;
    route.insert(route.begin() + first_rank, to_insert, 0);
    earliest.insert(earliest.begin() + first_rank, to_insert, 0);
    latest.insert(latest.begin() + first_rank, to_insert, 0);
    breaks_at_rank.insert(breaks_at_rank.begin() + first_rank, to_insert, 0);
    breaks_counts.insert(breaks_counts.begin() + first_rank, to_insert, 0);
  }

  // Current rank in route/earliest/latest vectors.
  Index current_job_rank = first_rank;
  unsigned breaks_before = 0;

  // Propagate earliest dates for all jobs and breaks in their
  // respective addition ranges.
  auto current_job = first_job;
  while (current_job != last_job or current_break != last_break) {
    if (current_job == last_job) {
      // Compute earliest end date for break after last inserted jobs.
      const auto& b = v.breaks[current_break];

      const auto b_tw =
        std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
          return current_earliest <= tw.end;
        });
      assert(b_tw != b.tws.end());

      if (current_earliest < b_tw->start) {
        auto margin = b_tw->start - current_earliest;
        breaks_travel_margin_before[current_break] = margin;
        if (margin < next_travel) {
          next_travel -= margin;
        } else {
          next_travel = 0;
        }

        current_earliest = b_tw->start;
      } else {
        breaks_travel_margin_before[current_break] = 0;
      }
      break_earliest[current_break] = current_earliest;

      current_earliest += b.service;

      ++breaks_before;
      ++current_break;
      continue;
    }

    if (current_break == last_break) {
      // Compute earliest end date for job after last inserted breaks.
      const auto& j = input.jobs[*current_job];

      current_earliest += previous_travel;

      const auto j_tw =
        std::find_if(j.tws.begin(), j.tws.end(), [&](const auto& tw) {
          return current_earliest <= tw.end;
        });
      assert(j_tw != j.tws.end());

      current_earliest = std::max(current_earliest, j_tw->start);

      route[current_job_rank] = *current_job;
      earliest[current_job_rank] = current_earliest;
      breaks_at_rank[current_job_rank] = breaks_before;
      breaks_counts[current_job_rank] = previous_breaks_counts + breaks_before;
      ++current_job_rank;
      previous_breaks_counts += breaks_before;
      breaks_before = 0;

      current_earliest += j.service;

      ++current_job;
      if (current_job != last_job) {
        // Account for travel time to next current job.
        previous_travel = m[j.index()][input.jobs[*current_job].index()];
      }
      continue;
    }

    // We still have both jobs and breaks to go through, so decide on
    // ordering.
    const auto& b = v.breaks[current_break];
    const auto& j = input.jobs[*current_job];
    auto oc = order_choice(input,
                           *current_job,
                           b,
                           current_earliest,
                           previous_travel,
                           next_travel,
                           current_latest);

    assert(oc.add_job_first xor oc.add_break_first);
    if (oc.add_break_first) {
      if (current_earliest < oc.b_tw->start) {
        auto margin = oc.b_tw->start - current_earliest;
        breaks_travel_margin_before[current_break] = margin;
        if (margin < previous_travel) {
          previous_travel -= margin;
        } else {
          previous_travel = 0;
        }

        current_earliest = oc.b_tw->start;
      } else {
        breaks_travel_margin_before[current_break] = 0;
      }
      break_earliest[current_break] = current_earliest;

      current_earliest += b.service;

      ++breaks_before;
      ++current_break;
    }
    if (oc.add_job_first) {
      current_earliest =
        std::max(current_earliest + previous_travel, oc.j_tw->start);

      route[current_job_rank] = *current_job;
      earliest[current_job_rank] = current_earliest;
      breaks_at_rank[current_job_rank] = breaks_before;
      breaks_counts[current_job_rank] = previous_breaks_counts + breaks_before;
      ++current_job_rank;
      previous_breaks_counts += breaks_before;
      breaks_before = 0;

      current_earliest += j.service;

      ++current_job;
      if (current_job != last_job) {
        // Account for travel time to next current job.
        previous_travel = m[j.index()][input.jobs[*current_job].index()];
      }
    }
  }

  assert(current_job_rank == first_rank + add_count);

  // Update remaining number of breaks due before next step.
  breaks_at_rank[current_job_rank] = breaks_before;
  assert(previous_breaks_counts + breaks_at_rank[current_job_rank] ==
         breaks_counts[current_job_rank]);

  if (!route.empty()) {
    if (current_job_rank == 0) {
      // First jobs in route have been erased and not replaced, so
      // update new first job earliest date.
      const auto& j = input.jobs[route[0]];

      current_earliest += next_travel;
      const auto j_tw =
        std::find_if(j.tws.begin(), j.tws.end(), [&](const auto& tw) {
          return current_earliest <= tw.end;
        });
      assert(j_tw != j.tws.end());

      earliest[0] = std::max(current_earliest, j_tw->start);
      assert(earliest[0] <= latest[0]);

      fwd_update_earliest_from(input, 0);
    } else {
      if (current_job_rank < route.size()) {
        // If a valid rank, current_job_rank is the rank of the first
        // job with known latest date (see bwd_update_latest_from call
        // below). Also update earliest dates forward in end of route.
        fwd_update_earliest_from(input, current_job_rank - 1);
      } else {
        // Replacing last job(s) in route, so we need to update
        // earliest end for route, latest date for breaks before end
        // and new last job.
        earliest_end = current_earliest + next_travel;

        // Latest date for breaks before end.
        Index break_rank = breaks_counts[current_job_rank];
        for (Index r = 0; r < breaks_at_rank[current_job_rank]; ++r) {
          --break_rank;
          const auto& b = v.breaks[break_rank];

          assert(b.service <= current_latest);
          current_latest -= b.service;

          const auto b_tw =
            std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
              return tw.start <= current_latest;
            });
          assert(b_tw != b.tws.rend());

          if (b_tw->end < current_latest) {
            auto margin = current_latest - b_tw->end;
            breaks_travel_margin_after[break_rank] = margin;
            if (margin < bwd_next_travel) {
              bwd_next_travel -= margin;
            } else {
              bwd_next_travel = 0;
            }

            current_latest = b_tw->end;
          } else {
            breaks_travel_margin_after[break_rank] = 0;
          }

          break_latest[break_rank] = current_latest;
        }

        // Latest date for new last job.
        const auto& j = input.jobs[route.back()];
        auto gap = j.service + bwd_next_travel;
        assert(gap <= current_latest);
        current_latest -= gap;

        const auto j_tw =
          std::find_if(j.tws.rbegin(), j.tws.rend(), [&](const auto& tw) {
            return tw.start <= current_latest;
          });
        assert(j_tw != j.tws.rend());

        latest.back() = std::min(current_latest, j_tw->end);

        // Set current_job_rank back to the rank of the first job with
        // known latest date (the one that was just updated).
        --current_job_rank;
      }
    }

    // Update latest dates backward.
    bwd_update_latest_from(input, current_job_rank);
  }

  update_amounts(input);
}

template bool
TWRoute::is_valid_addition_for_tw(const Input& input,
                                  const std::vector<Index>::iterator first_job,
                                  const std::vector<Index>::iterator last_job,
                                  const Index first_rank,
                                  const Index last_rank) const;
template bool TWRoute::is_valid_addition_for_tw(
  const Input& input,
  const std::vector<Index>::reverse_iterator first_job,
  const std::vector<Index>::reverse_iterator last_job,
  const Index first_rank,
  const Index last_rank) const;

template void TWRoute::replace(const Input& input,
                               const std::vector<Index>::iterator first_job,
                               const std::vector<Index>::iterator last_job,
                               const Index first_rank,
                               const Index last_rank);
template void
TWRoute::replace(const Input& input,
                 const std::vector<Index>::reverse_iterator first_job,
                 const std::vector<Index>::reverse_iterator last_job,
                 const Index first_rank,
                 const Index last_rank);

} // namespace vroom
