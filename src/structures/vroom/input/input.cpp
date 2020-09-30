/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/
#include <array>

#include "problems/cvrp/cvrp.h"
#include "problems/tsp/tsp.h"
#include "problems/vrp.h"
#include "problems/vrptw/vrptw.h"
#include "structures/vroom/input/input.h"
#include "utils/exception.h"
#include "utils/helpers.h"

namespace vroom {

Input::Input(unsigned amount_size)
  : _start_loading(std::chrono::high_resolution_clock::now()),
    _no_addition_yet(true),
    _has_TW(false),
    _homogeneous_locations(true),
    _geometry(false),
    _has_jobs(false),
    _has_shipments(false),
    _has_custom_matrix(false),
    _all_locations_have_coords(true),
    _amount_size(amount_size),
    _zero(_amount_size) {
}

void Input::set_geometry(bool geometry) {
  _geometry = geometry;
}

void Input::set_routing(std::unique_ptr<routing::Wrapper> routing_wrapper) {
  _routing_wrapper = std::move(routing_wrapper);
}

void Input::check_job(Job& job) {
  // Ensure delivery size consistency.
  const auto& delivery_size = job.delivery.size();
  if (delivery_size != _amount_size) {
    throw Exception(ERROR::INPUT,
                    "Inconsistent delivery length: " +
                      std::to_string(delivery_size) + " instead of " +
                      std::to_string(_amount_size) + '.');
  }

  // Ensure pickup size consistency.
  const auto& pickup_size = job.pickup.size();
  if (pickup_size != _amount_size) {
    throw Exception(ERROR::INPUT,
                    "Inconsistent pickup length: " +
                      std::to_string(pickup_size) + " instead of " +
                      std::to_string(_amount_size) + '.');
  }

  // Ensure that skills are either always or never provided.
  if (_no_addition_yet) {
    _has_skills = !job.skills.empty();
    _no_addition_yet = false;
  } else {
    if (_has_skills != !job.skills.empty()) {
      throw Exception(ERROR::INPUT, "Missing skills.");
    }
  }

  // Check for time-windows.
  _has_TW |= (!(job.tws.size() == 1) or !job.tws[0].is_default());

  if (!job.location.user_index()) {
    // Index of this job in the matrix was not specified upon job
    // creation.
    auto search = _locations_to_index.find(job.location);
    if (search != _locations_to_index.end()) {
      // Using stored index for existing location.
      job.location.set_index(search->second);
    } else {
      // Append new location and store corresponding index.
      auto new_index = _locations.size();
      job.location.set_index(new_index);
      _locations.push_back(job.location);
      _locations_to_index.insert(std::make_pair(job.location, new_index));
    }
  }

  _matrix_used_index.insert(job.index());
  _all_locations_have_coords =
    _all_locations_have_coords && job.location.has_coordinates();
}

void Input::add_job(const Job& job) {
  if (job.type != JOB_TYPE::SINGLE) {
    throw Exception(ERROR::INPUT, "Wrong job type.");
  }
  jobs.push_back(job);
  check_job(jobs.back());
  _has_jobs = true;
}

void Input::add_shipment(const Job& pickup, const Job& delivery) {
  if (pickup.priorities != delivery.priorities) {
    throw Exception(ERROR::INPUT, "Inconsistent shipment priority.");
  }
  if (!(pickup.pickup == delivery.delivery)) {
    throw Exception(ERROR::INPUT, "Inconsistent shipment amount.");
  }
  if (pickup.skills.size() != delivery.skills.size()) {
    throw Exception(ERROR::INPUT, "Inconsistent shipment skills.");
  }
  for (const auto s : pickup.skills) {
    if (delivery.skills.find(s) == delivery.skills.end()) {
      throw Exception(ERROR::INPUT, "Inconsistent shipment skills.");
    }
  }

  if (pickup.type != JOB_TYPE::PICKUP) {
    throw Exception(ERROR::INPUT, "Wrong pickup type.");
  }
  jobs.push_back(pickup);
  check_job(jobs.back());

  if (delivery.type != JOB_TYPE::DELIVERY) {
    throw Exception(ERROR::INPUT, "Wrong delivery type.");
  }
  jobs.push_back(delivery);
  check_job(jobs.back());
  _has_shipments = true;
}

void Input::add_vehicle(const Vehicle& vehicle) {
  vehicles.push_back(vehicle);

  auto& current_v = vehicles.back();

  // Ensure amount size consistency.
  const auto& vehicle_amount_size = current_v.capacity.size();
  if (vehicle_amount_size != _amount_size) {
    throw Exception(ERROR::INPUT,
                    "Inconsistent capacity length: " +
                      std::to_string(vehicle_amount_size) + " instead of " +
                      std::to_string(_amount_size) + '.');
  }

  // Ensure that skills are either always or never provided.
  if (_no_addition_yet) {
    _has_skills = !current_v.skills.empty();
    _no_addition_yet = false;
  } else {
    if (_has_skills != !current_v.skills.empty()) {
      throw Exception(ERROR::INPUT, "Missing skills.");
    }
  }

  // Check for time-windows.
  _has_TW = _has_TW || !vehicle.tw.is_default();

  bool has_start = current_v.has_start();
  bool has_end = current_v.has_end();

  if (has_start) {
    auto& start_loc = current_v.start.value();

    if (!start_loc.user_index()) {
      // Index of this start in the matrix was not specified upon
      // vehicle creation.
      assert(start_loc.has_coordinates());
      auto search = _locations_to_index.find(start_loc);
      if (search != _locations_to_index.end()) {
        // Using stored index for existing location.
        start_loc.set_index(search->second);
      } else {
        // Append new location and store corresponding index.
        auto new_index = _locations.size();
        start_loc.set_index(new_index);
        _locations.push_back(start_loc);
        _locations_to_index.insert(std::make_pair(start_loc, new_index));
      }
    }

    _matrix_used_index.insert(start_loc.index());
    _all_locations_have_coords =
      _all_locations_have_coords && start_loc.has_coordinates();
  }

  if (has_end) {
    auto& end_loc = current_v.end.value();

    if (!end_loc.user_index()) {
      // Index of this end in the matrix was not specified upon
      // vehicle creation.
      assert(end_loc.has_coordinates());
      auto search = _locations_to_index.find(end_loc);
      if (search != _locations_to_index.end()) {
        // Using stored index for existing location.
        end_loc.set_index(search->second);
      } else {
        // Append new location and store corresponding index.
        auto new_index = _locations.size();
        end_loc.set_index(new_index);
        _locations.push_back(end_loc);
        _locations_to_index.insert(std::make_pair(end_loc, new_index));
      }
    }

    _matrix_used_index.insert(end_loc.index());
    _all_locations_have_coords =
      _all_locations_have_coords && end_loc.has_coordinates();
  }

  // Check for homogeneous locations among vehicles.
  if (vehicles.size() > 1) {
    _homogeneous_locations =
      _homogeneous_locations &&
      vehicles.front().has_same_locations(vehicles.back());
  }
}

void Input::set_matrix(Matrix<Cost>&& m) {
  _has_custom_matrix = true;
  _matrix = std::move(m);
}

bool Input::has_skills() const {
  return _has_skills;
}

bool Input::has_jobs() const {
  return _has_jobs;
}

bool Input::has_shipments() const {
  return _has_shipments;
}

bool Input::has_homogeneous_locations() const {
  return _homogeneous_locations;
}

bool Input::vehicle_ok_with_vehicle(Index v1_index, Index v2_index) const {
  return _vehicle_to_vehicle_compatibility[v1_index][v2_index];
}

Matrix<Cost> Input::get_sub_matrix(const std::vector<Index>& indices) const {
  return _matrix.get_sub_matrix(indices);
}

void Input::check_cost_bound() const {
  // Check that we don't have any overflow while computing an upper
  // bound for solution cost.

  std::vector<Cost> max_cost_per_line(_matrix.size(), 0);
  std::vector<Cost> max_cost_per_column(_matrix.size(), 0);

  for (const auto i : _matrix_used_index) {
    for (const auto j : _matrix_used_index) {
      max_cost_per_line[i] = std::max(max_cost_per_line[i], _matrix[i][j]);
      max_cost_per_column[j] = std::max(max_cost_per_column[j], _matrix[i][j]);
    }
  }

  Cost jobs_departure_bound = 0;
  Cost jobs_arrival_bound = 0;
  for (const auto& j : jobs) {
    jobs_departure_bound =
      utils::add_without_overflow(jobs_departure_bound,
                                  max_cost_per_line[j.index()]);
    jobs_arrival_bound =
      utils::add_without_overflow(jobs_arrival_bound,
                                  max_cost_per_column[j.index()]);
  }

  Cost jobs_bound = std::max(jobs_departure_bound, jobs_arrival_bound);

  Cost start_bound = 0;
  Cost end_bound = 0;
  for (const auto& v : vehicles) {
    if (v.has_start()) {
      start_bound =
        utils::add_without_overflow(start_bound,
                                    max_cost_per_line[v.start.value().index()]);
    }
    if (v.has_end()) {
      end_bound =
        utils::add_without_overflow(end_bound,
                                    max_cost_per_column[v.end.value().index()]);
    }
  }

  Cost bound = utils::add_without_overflow(start_bound, jobs_bound);
  bound = utils::add_without_overflow(bound, end_bound);
}

void Input::set_compatibility() {
  // Default to no restriction when no skills are provided.
  _vehicle_to_job_compatibility = std::vector<
    std::vector<unsigned char>>(vehicles.size(),
                                std::vector<unsigned char>(jobs.size(), true));
  if (_has_skills) {
    for (std::size_t v = 0; v < vehicles.size(); ++v) {
      const auto& v_skills = vehicles[v].skills;
      assert(!v_skills.empty());

      for (std::size_t j = 0; j < jobs.size(); ++j) {
        bool is_compatible = true;
        assert(!jobs[j].skills.empty());
        for (const auto& s : jobs[j].skills) {
          if (v_skills.find(s) == v_skills.end()) {
            is_compatible = false;
            break;
          }
        }
        _vehicle_to_job_compatibility[v][j] = is_compatible;
      }
    }
  }

  // Derive potential extra incompatibilities : jobs or shipments with
  // amount that does not fit into vehicle or that cannot be added to
  // an empty route for vehicle based on the timing constraints (when
  // they apply).
  for (std::size_t v = 0; v < vehicles.size(); ++v) {
    TWRoute empty_route(*this, v);
    for (Index j = 0; j < jobs.size(); ++j) {
      if (_vehicle_to_job_compatibility[v][j]) {
        bool is_compatible =
          empty_route.is_valid_addition_for_capacity(*this,
                                                     jobs[j].pickup,
                                                     jobs[j].delivery,
                                                     0);

        bool is_shipment_pickup = (jobs[j].type == JOB_TYPE::PICKUP);

        if (is_compatible and _has_TW) {
          if (jobs[j].type == JOB_TYPE::SINGLE) {
            is_compatible = is_compatible &&
                            empty_route.is_valid_addition_for_tw(*this, j, 0);
          } else {
            assert(is_shipment_pickup);
            std::vector<Index> p_d({j, static_cast<Index>(j + 1)});
            is_compatible =
              is_compatible && empty_route.is_valid_addition_for_tw(*this,
                                                                    p_d.begin(),
                                                                    p_d.end(),
                                                                    0,
                                                                    0);
          }
        }

        _vehicle_to_job_compatibility[v][j] = is_compatible;
        if (is_shipment_pickup) {
          // Skipping matching delivery which is next in line in jobs.
          _vehicle_to_job_compatibility[v][j + 1] = is_compatible;
          ++j;
        }
      }
    }
  }

  _vehicle_to_vehicle_compatibility =
    std::vector<std::vector<bool>>(vehicles.size(),
                                   std::vector<bool>(vehicles.size(), false));
  for (std::size_t v1 = 0; v1 < vehicles.size(); ++v1) {
    _vehicle_to_vehicle_compatibility[v1][v1] = true;
    for (std::size_t v2 = v1 + 1; v2 < vehicles.size(); ++v2) {
      for (std::size_t j = 0; j < jobs.size(); ++j) {
        if (_vehicle_to_job_compatibility[v1][j] and
            _vehicle_to_job_compatibility[v2][j]) {
          _vehicle_to_vehicle_compatibility[v1][v2] = true;
          _vehicle_to_vehicle_compatibility[v2][v1] = true;
          break;
        }
      }
    }
  }
}

std::unique_ptr<VRP> Input::get_problem() const {
  if (_has_TW) {
    return std::make_unique<VRPTW>(*this);
  } else {
    return std::make_unique<CVRP>(*this);
  }
}

Solution Input::solve(unsigned exploration_level,
                      unsigned nb_thread,
                      const std::vector<HeuristicParameters>& h_param) {
  if (_geometry and !_all_locations_have_coords) {
    // Early abort when info is required with missing coordinates.
    throw Exception(ERROR::INPUT,
                    "Route geometry request with missing coordinates.");
  }

  if (!_has_custom_matrix) {
    if (_locations.size() == 1) {
      _matrix = Matrix<Cost>({{0}});
    } else {
      assert(_routing_wrapper);
      _matrix = _routing_wrapper->get_matrix(_locations);
    }
  }

  // Check for potential overflow in solution cost.
  this->check_cost_bound();

  // Fill vehicle/job compatibility matrix.
  this->set_compatibility();

  // Load relevant problem.
  auto instance = this->get_problem();
  _end_loading = std::chrono::high_resolution_clock::now();

  auto loading = std::chrono::duration_cast<std::chrono::milliseconds>(
                   _end_loading - _start_loading)
                   .count();

  // Solve.
  auto sol = instance->solve(exploration_level, nb_thread, h_param);

  // Update timing info.
  sol.summary.computing_times.loading = loading;

  _end_solving = std::chrono::high_resolution_clock::now();
  sol.summary.computing_times.solving =
    std::chrono::duration_cast<std::chrono::milliseconds>(_end_solving -
                                                          _end_loading)
      .count();

  if (_geometry) {
    for (auto& route : sol.routes) {
      _routing_wrapper->add_route_info(route);
      sol.summary.distance += route.distance;
    }

    _end_routing = std::chrono::high_resolution_clock::now();
    auto routing = std::chrono::duration_cast<std::chrono::milliseconds>(
                     _end_routing - _end_solving)
                     .count();

    sol.summary.computing_times.routing = routing;
  }

  return sol;
}

} // namespace vroom
