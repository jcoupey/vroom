/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <numeric>
#include <set>

#include "algorithms/heuristics/solomon.h"
#include "structures/vroom/raw_route.h"
#include "structures/vroom/tw_route.h"
#include "utils/helpers.h"

namespace vroom {
namespace heuristics {

template <class T> T basic(const Input& input, INIT init, float lambda) {
  T routes;
  for (Index v = 0; v < input.vehicles.size(); ++v) {
    routes.emplace_back(input, v);
  }

  std::set<Index> unassigned;
  for (Index j = 0; j < input.jobs.size(); ++j) {
    unassigned.insert(j);
  }

  // One level of indirection to allow easy ordering of the vehicles
  // within the heuristic.
  std::vector<Index> vehicles_ranks(input.vehicles.size());
  std::iota(vehicles_ranks.begin(), vehicles_ranks.end(), 0);
  // Sort vehicles by "higher" capacity or by time window in case of
  // capacities ties.
  std::stable_sort(vehicles_ranks.begin(),
                   vehicles_ranks.end(),
                   [&](const auto lhs, const auto rhs) {
                     auto& v_lhs = input.vehicles[lhs];
                     auto& v_rhs = input.vehicles[rhs];
                     return v_rhs.capacity << v_lhs.capacity or
                            (v_lhs.capacity == v_rhs.capacity and
                             v_lhs.tw.length > v_rhs.tw.length);
                   });

  const auto& m = input.get_matrix();

  // For a single job j, costs[j] is the cost of fetching job j in an
  // empty route from one of the vehicles (consistent across vehicles
  // in the homogeneous case). For a pickup (resp. delivery) job j,
  // costs[j] is the cost of fetching job j **and** associated
  // delivery (resp. pickup) in an empty route from one of the
  // vehicles.
  const auto& v = input.vehicles[0];

  std::vector<Cost> costs(input.jobs.size());
  for (std::size_t j = 0; j < input.jobs.size(); ++j) {
    Index j_index = input.jobs[j].index();
    bool is_pickup = (input.jobs[j].type == JOB_TYPE::PICKUP);

    Cost current_cost = 0;
    if (v.has_start()) {
      current_cost += m[v.start.value().index()][j_index];
    }

    Index last_job_index = j_index;
    if (is_pickup) {
      assert((j + 1 < input.jobs.size()) and
             (input.jobs[j + 1].type == JOB_TYPE::DELIVERY));

      // Add delivery cost.
      last_job_index = input.jobs[j + 1].index();
      current_cost += m[j_index][last_job_index];
    }

    if (v.has_end()) {
      current_cost += m[last_job_index][v.end.value().index()];
    }
    costs[j] = current_cost;
    if (is_pickup) {
      // Assign same cost to delivery and skip it.
      costs[j + 1] = current_cost;
      ++j;
    }
  }

  for (Index v = 0; v < input.vehicles.size(); ++v) {
    auto v_rank = vehicles_ranks[v];
    auto& current_r = routes[v_rank];

    const auto& vehicle = input.vehicles[v_rank];

    if (init != INIT::NONE) {
      // Initialize current route with the "best" valid job.
      bool init_ok = false;

      Amount higher_amount(input.zero_amount());
      Cost furthest_cost = 0;
      Cost nearest_cost = std::numeric_limits<Cost>::max();
      Duration earliest_deadline = std::numeric_limits<Duration>::max();
      Index best_job_rank = 0;
      for (const auto job_rank : unassigned) {
        if (!input.vehicle_ok_with_job(v_rank, job_rank) or
            input.jobs[job_rank].type == JOB_TYPE::DELIVERY) {
          continue;
        }

        bool is_pickup = (input.jobs[job_rank].type == JOB_TYPE::PICKUP);

        bool try_validity = false;

        if (init == INIT::HIGHER_AMOUNT) {
          try_validity |= (higher_amount << input.jobs[job_rank].pickup or
                           higher_amount << input.jobs[job_rank].delivery);
        }
        if (init == INIT::EARLIEST_DEADLINE) {
          Duration current_deadline =
            (is_pickup) ? input.jobs[job_rank + 1].tws.back().end
                        : input.jobs[job_rank].tws.back().end;
          try_validity |= (current_deadline < earliest_deadline);
        }
        if (init == INIT::FURTHEST) {
          try_validity |= (furthest_cost < costs[job_rank]);
        }
        if (init == INIT::NEAREST) {
          try_validity |= (costs[job_rank] < nearest_cost);
        }

        if (!try_validity) {
          continue;
        }

        bool is_valid =
          current_r
            .is_valid_addition_for_capacity(input,
                                            input.jobs[job_rank].pickup,
                                            input.jobs[job_rank].delivery,
                                            0);
        if (is_pickup) {
          std::vector<Index> p_d({job_rank, static_cast<Index>(job_rank + 1)});
          is_valid = is_valid && current_r.is_valid_addition_for_tw(input,
                                                                    p_d.begin(),
                                                                    p_d.end(),
                                                                    0,
                                                                    0);
        } else {
          assert(input.jobs[job_rank].type == JOB_TYPE::SINGLE);
          is_valid =
            is_valid && current_r.is_valid_addition_for_tw(input, job_rank, 0);
        }

        if (is_valid) {
          init_ok = true;
          best_job_rank = job_rank;

          switch (init) {
          case INIT::NONE:
            assert(false);
            break;
          case INIT::HIGHER_AMOUNT:
            if (higher_amount << input.jobs[job_rank].pickup) {
              higher_amount = input.jobs[job_rank].pickup;
            }
            if (higher_amount << input.jobs[job_rank].delivery) {
              higher_amount = input.jobs[job_rank].delivery;
            }
            break;
          case INIT::EARLIEST_DEADLINE:
            earliest_deadline = (is_pickup)
                                  ? input.jobs[job_rank + 1].tws.back().end
                                  : input.jobs[job_rank].tws.back().end;
            break;
          case INIT::FURTHEST:
            furthest_cost = costs[job_rank];
            break;
          case INIT::NEAREST:
            nearest_cost = costs[job_rank];
            break;
          }
        }
      }

      if (init_ok) {
        if (input.jobs[best_job_rank].type == JOB_TYPE::SINGLE) {
          current_r.add(input, best_job_rank, 0);
          unassigned.erase(best_job_rank);
        }
        if (input.jobs[best_job_rank].type == JOB_TYPE::PICKUP) {
          std::vector<Index> p_d(
            {best_job_rank, static_cast<Index>(best_job_rank + 1)});
          current_r.replace(input, p_d.begin(), p_d.end(), 0, 0);
          unassigned.erase(best_job_rank);
          unassigned.erase(best_job_rank + 1);
        }
      }
    }

    bool keep_going = true;
    while (keep_going) {
      keep_going = false;
      float best_cost = std::numeric_limits<float>::max();
      Index best_job_rank = 0;
      Index best_r = 0;
      Index best_pickup_r = 0;
      Index best_delivery_r = 0;

      for (const auto job_rank : unassigned) {
        if (!input.vehicle_ok_with_job(v_rank, job_rank)) {
          continue;
        }

        if (input.jobs[job_rank].type == JOB_TYPE::DELIVERY) {
          continue;
        }

        if (input.jobs[job_rank].type == JOB_TYPE::SINGLE) {
          for (Index r = 0; r <= current_r.size(); ++r) {
            float current_add = utils::addition_cost(input,
                                                     m,
                                                     job_rank,
                                                     vehicle,
                                                     current_r.route,
                                                     r);

            float current_cost =
              current_add - lambda * static_cast<float>(costs[job_rank]);

            if (current_cost < best_cost and
                current_r
                  .is_valid_addition_for_capacity(input,
                                                  input.jobs[job_rank].pickup,
                                                  input.jobs[job_rank].delivery,
                                                  r) and
                current_r.is_valid_addition_for_tw(input, job_rank, r)) {
              best_cost = current_cost;
              best_job_rank = job_rank;
              best_r = r;
            }
          }
        }

        if (input.jobs[job_rank].type == JOB_TYPE::PICKUP) {
          // Pre-compute cost of addition for matching delivery.
          std::vector<Gain> d_adds(current_r.route.size() + 1);
          std::vector<unsigned char> valid_delivery_insertions(
            current_r.route.size() + 1);

          for (unsigned d_rank = 0; d_rank <= current_r.route.size();
               ++d_rank) {
            d_adds[d_rank] = utils::addition_cost(input,
                                                  m,
                                                  job_rank + 1,
                                                  vehicle,
                                                  current_r.route,
                                                  d_rank);
            valid_delivery_insertions[d_rank] =
              current_r.is_valid_addition_for_tw(input, job_rank + 1, d_rank);
          }

          for (Index pickup_r = 0; pickup_r <= current_r.size(); ++pickup_r) {
            Gain p_add = utils::addition_cost(input,
                                              m,
                                              job_rank,
                                              vehicle,
                                              current_r.route,
                                              pickup_r);

            if (!current_r
                   .is_valid_addition_for_load(input,
                                               input.jobs[job_rank].pickup,
                                               pickup_r) or
                !current_r.is_valid_addition_for_tw(input,
                                                    job_rank,
                                                    pickup_r)) {
              continue;
            }

            // Build replacement sequence for current insertion.
            std::vector<Index> modified_with_pd({job_rank});
            Amount modified_delivery = input.zero_amount();

            for (Index delivery_r = pickup_r; delivery_r <= current_r.size();
                 ++delivery_r) {
              // Update state variables along the way before potential
              // early abort.
              if (pickup_r < delivery_r) {
                modified_with_pd.push_back(current_r.route[delivery_r - 1]);
                const auto& new_modified_job =
                  input.jobs[current_r.route[delivery_r - 1]];
                if (new_modified_job.type == JOB_TYPE::SINGLE) {
                  modified_delivery += new_modified_job.delivery;
                }
              }

              if (!(bool)valid_delivery_insertions[delivery_r]) {
                continue;
              }

              float current_add;
              if (pickup_r == delivery_r) {
                current_add = utils::addition_cost(input,
                                                   m,
                                                   job_rank,
                                                   vehicle,
                                                   current_r.route,
                                                   pickup_r,
                                                   pickup_r + 1);
              } else {
                current_add = p_add + d_adds[delivery_r];
              }

              float current_cost =
                current_add - lambda * static_cast<float>(costs[job_rank]);

              if (current_cost < best_cost) {
                modified_with_pd.push_back(job_rank + 1);

                // Update best cost depending on validity.
                bool valid =
                  current_r
                    .is_valid_addition_for_capacity_inclusion(input,
                                                              modified_delivery,
                                                              modified_with_pd
                                                                .begin(),
                                                              modified_with_pd
                                                                .end(),
                                                              pickup_r,
                                                              delivery_r);

                valid =
                  valid &&
                  current_r.is_valid_addition_for_tw(input,
                                                     modified_with_pd.begin(),
                                                     modified_with_pd.end(),
                                                     pickup_r,
                                                     delivery_r);

                modified_with_pd.pop_back();

                if (valid) {
                  best_cost = current_cost;
                  best_job_rank = job_rank;
                  best_pickup_r = pickup_r;
                  best_delivery_r = delivery_r;
                }
              }
            }
          }
        }
      }

      if (best_cost < std::numeric_limits<float>::max()) {
        if (input.jobs[best_job_rank].type == JOB_TYPE::SINGLE) {
          current_r.add(input, best_job_rank, best_r);
          unassigned.erase(best_job_rank);
          keep_going = true;
        }
        if (input.jobs[best_job_rank].type == JOB_TYPE::PICKUP) {
          std::vector<Index> modified_with_pd({best_job_rank});
          std::copy(current_r.route.begin() + best_pickup_r,
                    current_r.route.begin() + best_delivery_r,
                    std::back_inserter(modified_with_pd));
          modified_with_pd.push_back(best_job_rank + 1);

          current_r.replace(input,
                            modified_with_pd.begin(),
                            modified_with_pd.end(),
                            best_pickup_r,
                            best_delivery_r);
          unassigned.erase(best_job_rank);
          unassigned.erase(best_job_rank + 1);
          keep_going = true;
        }
      }
    }
  }

  return routes;
}

template <class T>
T dynamic_vehicle_choice(const Input& input, INIT init, float lambda) {
  T routes;
  for (Index v = 0; v < input.vehicles.size(); ++v) {
    routes.emplace_back(input, v);
  }

  std::set<Index> unassigned;
  for (Index j = 0; j < input.jobs.size(); ++j) {
    unassigned.insert(j);
  }

  std::vector<Index> vehicles_ranks(input.vehicles.size());
  std::iota(vehicles_ranks.begin(), vehicles_ranks.end(), 0);

  const auto& m = input.get_matrix();

  // For a single job j, costs[j][v] is the cost of fetching job j in
  // an empty route from vehicle at vehicles_ranks[v]. For a pickup
  // job j, costs[j][v] is the cost of fetching job j **and**
  // associated delivery in an empty route from vehicle at
  // vehicles_ranks[v].
  std::vector<std::vector<Cost>> costs(input.jobs.size(),
                                       std::vector<Cost>(
                                         input.vehicles.size()));
  for (std::size_t j = 0; j < input.jobs.size(); ++j) {
    Index j_index = input.jobs[j].index();
    bool is_pickup = (input.jobs[j].type == JOB_TYPE::PICKUP);

    Index last_job_index = j_index;
    if (is_pickup) {
      assert((j + 1 < input.jobs.size()) and
             (input.jobs[j + 1].type == JOB_TYPE::DELIVERY));
      last_job_index = input.jobs[j + 1].index();
    }

    for (std::size_t v = 0; v < vehicles_ranks.size(); ++v) {
      const auto& vehicle = input.vehicles[vehicles_ranks[v]];
      Cost current_cost = is_pickup ? m[j_index][last_job_index] : 0;
      if (vehicle.has_start()) {
        current_cost += m[vehicle.start.value().index()][j_index];
      }
      if (vehicle.has_end()) {
        current_cost += m[last_job_index][vehicle.end.value().index()];
      }
      costs[j][v] = current_cost;
      if (is_pickup) {
        // Assign same cost to delivery.
        costs[j + 1][v] = current_cost;
      }
    }

    if (is_pickup) {
      // Skip delivery.
      ++j;
    }
  }

  while (!vehicles_ranks.empty() and !unassigned.empty()) {
    // For any unassigned job at j, jobs_min_costs[j]
    // (resp. jobs_second_min_costs[j]) holds the min cost
    // (resp. second min cost) of picking the job in an empty route
    // for any remaining vehicle.
    std::vector<Cost> jobs_min_costs(input.jobs.size(),
                                     std::numeric_limits<Cost>::max());
    std::vector<Cost> jobs_second_min_costs(input.jobs.size(),
                                            std::numeric_limits<Cost>::max());
    for (const auto job_rank : unassigned) {
      for (const auto v_rank : vehicles_ranks) {
        if (costs[job_rank][v_rank] <= jobs_min_costs[job_rank]) {
          jobs_second_min_costs[job_rank] = jobs_min_costs[job_rank];
          jobs_min_costs[job_rank] = costs[job_rank][v_rank];
        } else {
          if (costs[job_rank][v_rank] < jobs_second_min_costs[job_rank]) {
            jobs_second_min_costs[job_rank] = costs[job_rank][v_rank];
          }
        }
      }
    }

    // Pick vehicle that has the biggest number of compatible jobs
    // closest to him than to any other different vehicle.
    std::vector<unsigned> closest_jobs_count(input.vehicles.size(), 0);
    for (const auto job_rank : unassigned) {
      for (const auto v_rank : vehicles_ranks) {
        if (costs[job_rank][v_rank] == jobs_min_costs[job_rank]) {
          ++closest_jobs_count[v_rank];
        }
      }
    }

    const auto chosen_vehicle =
      std::min_element(vehicles_ranks.begin(),
                       vehicles_ranks.end(),
                       [&](const auto lhs, const auto rhs) {
                         auto& v_lhs = input.vehicles[lhs];
                         auto& v_rhs = input.vehicles[rhs];
                         return closest_jobs_count[lhs] >
                                  closest_jobs_count[rhs] or
                                (closest_jobs_count[lhs] ==
                                   closest_jobs_count[rhs] and
                                 (v_rhs.capacity << v_lhs.capacity or
                                  (v_lhs.capacity == v_rhs.capacity and
                                   v_lhs.tw.length > v_rhs.tw.length)));
                       });
    auto v_rank = *chosen_vehicle;
    vehicles_ranks.erase(chosen_vehicle);

    // Once current vehicle is decided, regrets[j] holds the min cost
    // of picking the job in an empty route for other remaining
    // vehicles.
    std::vector<Cost> regrets(input.jobs.size(),
                              std::numeric_limits<Cost>::max());
    for (const auto job_rank : unassigned) {
      if (jobs_min_costs[job_rank] < costs[job_rank][v_rank]) {
        regrets[job_rank] = jobs_min_costs[job_rank];
      } else {
        regrets[job_rank] = jobs_second_min_costs[job_rank];
      }
    }

    const auto& vehicle = input.vehicles[v_rank];
    auto& current_r = routes[v_rank];

    if (init != INIT::NONE) {
      // Initialize current route with the "best" valid job that is
      // closest for current vehicle than to any other remaining
      // vehicle.
      bool init_ok = false;

      Amount higher_amount(input.zero_amount());
      Cost furthest_cost = 0;
      Cost nearest_cost = std::numeric_limits<Cost>::max();
      Duration earliest_deadline = std::numeric_limits<Duration>::max();
      Index best_job_rank = 0;
      for (const auto job_rank : unassigned) {
        if (jobs_min_costs[job_rank] < costs[job_rank][v_rank] or
            // One of the remaining vehicles is closest to that job.
            !input.vehicle_ok_with_job(v_rank, job_rank) or
            input.jobs[job_rank].type == JOB_TYPE::DELIVERY) {
          continue;
        }

        bool is_pickup = (input.jobs[job_rank].type == JOB_TYPE::PICKUP);

        bool try_validity = false;

        if (init == INIT::HIGHER_AMOUNT) {
          try_validity |= (higher_amount << input.jobs[job_rank].pickup or
                           higher_amount << input.jobs[job_rank].delivery);
        }
        if (init == INIT::EARLIEST_DEADLINE) {
          Duration current_deadline =
            (is_pickup) ? input.jobs[job_rank + 1].tws.back().end
                        : input.jobs[job_rank].tws.back().end;
          try_validity |= (current_deadline < earliest_deadline);
        }
        if (init == INIT::FURTHEST) {
          try_validity |= (furthest_cost < costs[job_rank][v_rank]);
        }
        if (init == INIT::NEAREST) {
          try_validity |= (costs[job_rank][v_rank] < nearest_cost);
        }

        if (!try_validity) {
          continue;
        }

        bool is_valid =
          current_r
            .is_valid_addition_for_capacity(input,
                                            input.jobs[job_rank].pickup,
                                            input.jobs[job_rank].delivery,
                                            0);

        if (is_pickup) {
          std::vector<Index> p_d({job_rank, static_cast<Index>(job_rank + 1)});
          is_valid = is_valid && current_r.is_valid_addition_for_tw(input,
                                                                    p_d.begin(),
                                                                    p_d.end(),
                                                                    0,
                                                                    0);
        } else {
          assert(input.jobs[job_rank].type == JOB_TYPE::SINGLE);
          is_valid =
            is_valid && current_r.is_valid_addition_for_tw(input, job_rank, 0);
        }

        if (is_valid) {
          init_ok = true;
          best_job_rank = job_rank;

          switch (init) {
          case INIT::NONE:
            assert(false);
            break;
          case INIT::HIGHER_AMOUNT:
            if (higher_amount << input.jobs[job_rank].pickup) {
              higher_amount = input.jobs[job_rank].pickup;
            }
            if (higher_amount << input.jobs[job_rank].delivery) {
              higher_amount = input.jobs[job_rank].delivery;
            }
            break;
          case INIT::EARLIEST_DEADLINE:
            earliest_deadline = (is_pickup)
                                  ? input.jobs[job_rank + 1].tws.back().end
                                  : input.jobs[job_rank].tws.back().end;
            break;
          case INIT::FURTHEST:
            furthest_cost = costs[job_rank][v_rank];
            break;
          case INIT::NEAREST:
            nearest_cost = costs[job_rank][v_rank];
            break;
          }
        }
      }

      if (init_ok) {
        if (input.jobs[best_job_rank].type == JOB_TYPE::SINGLE) {
          current_r.add(input, best_job_rank, 0);
          unassigned.erase(best_job_rank);
        }
        if (input.jobs[best_job_rank].type == JOB_TYPE::PICKUP) {
          std::vector<Index> p_d(
            {best_job_rank, static_cast<Index>(best_job_rank + 1)});
          current_r.replace(input, p_d.begin(), p_d.end(), 0, 0);
          unassigned.erase(best_job_rank);
          unassigned.erase(best_job_rank + 1);
        }
      }
    }

    bool keep_going = true;
    while (keep_going) {
      keep_going = false;
      float best_cost = std::numeric_limits<float>::max();
      Index best_job_rank = 0;
      Index best_r = 0;
      Index best_pickup_r = 0;
      Index best_delivery_r = 0;

      for (const auto job_rank : unassigned) {
        if (!input.vehicle_ok_with_job(v_rank, job_rank)) {
          continue;
        }

        if (input.jobs[job_rank].type == JOB_TYPE::DELIVERY) {
          continue;
        }

        if (input.jobs[job_rank].type == JOB_TYPE::SINGLE) {
          for (Index r = 0; r <= current_r.size(); ++r) {
            float current_add = utils::addition_cost(input,
                                                     m,
                                                     job_rank,
                                                     vehicle,
                                                     current_r.route,
                                                     r);

            float current_cost =
              current_add - lambda * static_cast<float>(regrets[job_rank]);

            if (current_cost < best_cost and
                current_r
                  .is_valid_addition_for_capacity(input,
                                                  input.jobs[job_rank].pickup,
                                                  input.jobs[job_rank].delivery,
                                                  r) and
                current_r.is_valid_addition_for_tw(input, job_rank, r)) {
              best_cost = current_cost;
              best_job_rank = job_rank;
              best_r = r;
            }
          }
        }

        if (input.jobs[job_rank].type == JOB_TYPE::PICKUP) {
          // Pre-compute cost of addition for matching delivery.
          std::vector<Gain> d_adds(current_r.route.size() + 1);
          std::vector<unsigned char> valid_delivery_insertions(
            current_r.route.size() + 1);

          for (unsigned d_rank = 0; d_rank <= current_r.route.size();
               ++d_rank) {
            d_adds[d_rank] = utils::addition_cost(input,
                                                  m,
                                                  job_rank + 1,
                                                  vehicle,
                                                  current_r.route,
                                                  d_rank);
            valid_delivery_insertions[d_rank] =
              current_r.is_valid_addition_for_tw(input, job_rank + 1, d_rank);
          }

          for (Index pickup_r = 0; pickup_r <= current_r.size(); ++pickup_r) {
            Gain p_add = utils::addition_cost(input,
                                              m,
                                              job_rank,
                                              vehicle,
                                              current_r.route,
                                              pickup_r);

            if (!current_r
                   .is_valid_addition_for_load(input,
                                               input.jobs[job_rank].pickup,
                                               pickup_r) or
                !current_r.is_valid_addition_for_tw(input,
                                                    job_rank,
                                                    pickup_r)) {
              continue;
            }

            // Build replacement sequence for current insertion.
            std::vector<Index> modified_with_pd({job_rank});
            Amount modified_delivery = input.zero_amount();

            for (Index delivery_r = pickup_r; delivery_r <= current_r.size();
                 ++delivery_r) {
              // Update state variables along the way before potential
              // early abort.
              if (pickup_r < delivery_r) {
                modified_with_pd.push_back(current_r.route[delivery_r - 1]);
                const auto& new_modified_job =
                  input.jobs[current_r.route[delivery_r - 1]];
                if (new_modified_job.type == JOB_TYPE::SINGLE) {
                  modified_delivery += new_modified_job.delivery;
                }
              }

              if (!(bool)valid_delivery_insertions[delivery_r]) {
                continue;
              }

              float current_add;
              if (pickup_r == delivery_r) {
                current_add = utils::addition_cost(input,
                                                   m,
                                                   job_rank,
                                                   vehicle,
                                                   current_r.route,
                                                   pickup_r,
                                                   pickup_r + 1);
              } else {
                current_add = p_add + d_adds[delivery_r];
              }

              float current_cost =
                current_add - lambda * static_cast<float>(regrets[job_rank]);

              if (current_cost < best_cost) {
                modified_with_pd.push_back(job_rank + 1);

                // Update best cost depending on validity.
                bool is_valid =
                  current_r
                    .is_valid_addition_for_capacity_inclusion(input,
                                                              modified_delivery,
                                                              modified_with_pd
                                                                .begin(),
                                                              modified_with_pd
                                                                .end(),
                                                              pickup_r,
                                                              delivery_r);

                is_valid =
                  is_valid &&
                  current_r.is_valid_addition_for_tw(input,
                                                     modified_with_pd.begin(),
                                                     modified_with_pd.end(),
                                                     pickup_r,
                                                     delivery_r);

                modified_with_pd.pop_back();

                if (is_valid) {
                  best_cost = current_cost;
                  best_job_rank = job_rank;
                  best_pickup_r = pickup_r;
                  best_delivery_r = delivery_r;
                }
              }
            }
          }
        }
      }

      if (best_cost < std::numeric_limits<float>::max()) {
        if (input.jobs[best_job_rank].type == JOB_TYPE::SINGLE) {
          current_r.add(input, best_job_rank, best_r);
          unassigned.erase(best_job_rank);
          keep_going = true;
        }
        if (input.jobs[best_job_rank].type == JOB_TYPE::PICKUP) {
          std::vector<Index> modified_with_pd({best_job_rank});
          std::copy(current_r.route.begin() + best_pickup_r,
                    current_r.route.begin() + best_delivery_r,
                    std::back_inserter(modified_with_pd));
          modified_with_pd.push_back(best_job_rank + 1);

          current_r.replace(input,
                            modified_with_pd.begin(),
                            modified_with_pd.end(),
                            best_pickup_r,
                            best_delivery_r);
          unassigned.erase(best_job_rank);
          unassigned.erase(best_job_rank + 1);
          keep_going = true;
        }
      }
    }
  }

  return routes;
}

using RawSolution = std::vector<RawRoute>;
using TWSolution = std::vector<TWRoute>;

template RawSolution basic(const Input& input, INIT init, float lambda);

template RawSolution dynamic_vehicle_choice(const Input& input,
                                            INIT init,
                                            float lambda);

template TWSolution basic(const Input& input, INIT init, float lambda);

template TWSolution dynamic_vehicle_choice(const Input& input,
                                           INIT init,
                                           float lambda);

} // namespace heuristics
} // namespace vroom
