#ifndef INPUT_H
#define INPUT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <chrono>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "routing/wrapper.h"
#include "structures/generic/matrix.h"
#include "structures/typedefs.h"
#include "structures/vroom/job.h"
#include "structures/vroom/solution/solution.h"
#include "structures/vroom/vehicle.h"

namespace vroom {

class VRP;

class Input {
private:
  std::chrono::high_resolution_clock::time_point _start_loading;
  std::chrono::high_resolution_clock::time_point _end_loading;
  std::chrono::high_resolution_clock::time_point _end_solving;
  std::chrono::high_resolution_clock::time_point _end_routing;
  std::unique_ptr<routing::Wrapper> _routing_wrapper;
  bool _no_addition_yet;
  bool _has_skills;
  bool _has_TW;
  bool _homogeneous_locations;
  bool _geometry;
  bool _has_jobs;
  bool _has_shipments;
  bool _has_custom_matrix;
  Matrix<Cost> _matrix;
  std::vector<Location> _locations;
  std::unordered_map<Location, Index> _locations_to_index;
  std::vector<std::vector<unsigned char>> _vehicle_to_job_compatibility;
  std::vector<std::vector<bool>> _vehicle_to_vehicle_compatibility;
  std::unordered_set<Index> _matrix_used_index;
  bool _all_locations_have_coords;

  const unsigned _amount_size;
  const Amount _zero;

  std::unique_ptr<VRP> get_problem() const;

  void check_job(Job& job);

  void check_cost_bound() const;

  void set_compatibility();

public:
  std::vector<Job> jobs;
  std::vector<Vehicle> vehicles;

  Input(unsigned amount_size);

  void set_geometry(bool geometry);

  void set_routing(std::unique_ptr<routing::Wrapper> routing_wrapper);

  void add_job(const Job& job);

  void add_shipment(const Job& pickup, const Job& delivery);

  void add_vehicle(const Vehicle& vehicle);

  void set_matrix(Matrix<Cost>&& m);

  const Amount& zero_amount() const {
    return _zero;
  }

  bool has_skills() const;

  bool has_jobs() const;

  bool has_shipments() const;

  bool has_homogeneous_locations() const;

  bool vehicle_ok_with_job(size_t v_index, size_t j_index) const {
    return (bool)_vehicle_to_job_compatibility[v_index][j_index];
  }

  // Returns true iff both vehicles have common job candidates.
  bool vehicle_ok_with_vehicle(Index v1_index, Index v2_index) const;

  const Matrix<Cost>& get_matrix() const {
    return _matrix;
  }

  Matrix<Cost> get_sub_matrix(const std::vector<Index>& indices) const;

  Solution solve(unsigned exploration_level,
                 unsigned nb_thread,
                 const std::vector<HeuristicParameters>& h_param =
                   std::vector<HeuristicParameters>());
};

} // namespace vroom

#endif
