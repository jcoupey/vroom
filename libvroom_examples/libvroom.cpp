#include <iostream>

#include "routing/osrm_routed_wrapper.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/job.h"
#include "structures/vroom/vehicle.h"
#include "utils/exception.h"

void log_solution(const vroom::Solution& sol, bool geometry) {
  std::cout << "Total cost: " << sol.summary.cost << std::endl;
  std::cout << "Unassigned: " << sol.summary.unassigned << std::endl;

  // Log unassigned jobs if any.
  std::cout << "Unassigned job ids: ";
  for (const auto& j : sol.unassigned) {
    std::cout << j.id << ", ";
  }
  std::cout << std::endl;

  // Describe routes in solution.
  for (const auto& route : sol.routes) {
    std::cout << "Steps for vehicle " << route.vehicle
              << " (cost: " << route.cost;
    std::cout << " - duration: " << route.duration;
    std::cout << " - service: " << route.service;
    if (geometry) {
      std::cout << " - distance: " << route.distance;
    }

    std::cout << ")" << std::endl;

    // Describe all route steps.
    for (const auto& step : route.steps) {
      std::string type;
      switch (step.step_type) {
      case vroom::STEP_TYPE::START:
        type = "Start";
        break;
      case vroom::STEP_TYPE::END:
        type = "End";
        break;
      case vroom::STEP_TYPE::BREAK:
        type = "Break";
        break;
      case vroom::STEP_TYPE::JOB:
        switch (step.job_type) {
        case vroom::JOB_TYPE::SINGLE:
          type = "Job";
          break;
        case vroom::JOB_TYPE::PICKUP:
          type = "Pickup";
          break;
        case vroom::JOB_TYPE::DELIVERY:
          type = "Delivery";
          break;
        }
        break;
      }
      std::cout << type;

      // Add job/pickup/delivery/break ids.
      if (step.step_type != vroom::STEP_TYPE::START and
          step.step_type != vroom::STEP_TYPE::END) {
        std::cout << " " << step.id;
      }

      // Add location if known.
      if (step.location.has_coordinates()) {
        std::cout << " - " << step.location.lon() << ";" << step.location.lat();
      }

      std::cout << " - arrival: " << step.arrival;
      std::cout << " - duration: " << step.duration;
      std::cout << " - service: " << step.service;

      // Add extra step info if geometry is required.
      if (geometry) {
        std::cout << " - distance: " << step.distance;
      }
      std::cout << std::endl;
    }
  }
}

void run_example_with_osrm() {
  bool GEOMETRY = true;
  unsigned amount_dimension = 1;

  // Set OSRM host and port for "car" profile.
  vroom::io::Servers servers;
  servers["car"] = vroom::Server("localhost", "5000");

  vroom::Input problem_instance(amount_dimension, servers, vroom::ROUTER::OSRM);

  problem_instance.set_geometry(GEOMETRY); // Query for route geometry
                                           // after solving.

  // Create one-dimension capacity restrictions to model the situation
  // where one vehicle can handle 4 jobs with deliveries.
  vroom::Amount vehicle_capacity(1);
  vroom::TimeWindow vehicle_tw(28800, 43200); // Working hours.
  // Default "zero" amount data structures with relevant dimension.
  vroom::Amount job_delivery(amount_dimension);
  vroom::Amount job_empty_delivery(amount_dimension);
  job_delivery[0] = 1;

  vroom::Amount job_pickup(amount_dimension);
  vroom::Amount job_empty_pickup(amount_dimension);
  job_pickup[0] = 1;

  vroom::Duration setup = 0;
  vroom::Duration service = 5 * 60; // 5 minutes
  vehicle_capacity[0] = 4;

  // Define vehicle breaks.
  vroom::Break break_1(1, {vroom::TimeWindow(32400, 34200)}, 300);
  vroom::Break break_2(2, {vroom::TimeWindow(34200, 36000)}, 300);

  // Define vehicles (use std::nullopt for no start or no end).
  vroom::Location depot(vroom::Coordinates({{2.35044, 48.71764}}));
  vroom::Vehicle v1(1,                // id
                    depot,            // start
                    depot,            // end
                    "car",            // profile
                    vehicle_capacity, // capacity
                    {1, 14},          // skills
                    vehicle_tw,       // time window
                    {break_1});       // breaks
  problem_instance.add_vehicle(v1);

  vroom::Vehicle v2(2,                // id
                    depot,            // start
                    depot,            // end
                    "car",            // profile
                    vehicle_capacity, // capacity
                    {2, 14},          // skills
                    vehicle_tw,       // time window
                    {break_2});       // breaks

  problem_instance.add_vehicle(v2);

  // Job to be done between 9 and 10 AM.
  std::vector<vroom::TimeWindow> job_1_tws({{32400, 36000}});

  // Set jobs id, location, service time, amount, required skills,
  // priority and time windows. Constraints that are not required can
  // be omitted.
  std::vector<vroom::Job> jobs;
  jobs.push_back(vroom::Job(1,
                            vroom::Coordinates({{1.98935, 48.701}}),
                            setup,
                            service,
                            job_delivery,
                            job_empty_pickup,
                            {1}, // skills
                            0,   // default priority
                            job_1_tws));
  jobs.push_back(vroom::Job(2,
                            vroom::Coordinates({{2.03655, 48.61128}}),
                            setup,
                            service,
                            job_empty_delivery,
                            job_pickup,
                            {1}));
  jobs.push_back(vroom::Job(5,
                            vroom::Coordinates({{2.28325, 48.5958}}),
                            setup,
                            service,
                            job_delivery,
                            job_empty_pickup,
                            {14}));
  jobs.push_back(vroom::Job(6,
                            vroom::Coordinates({{2.89357, 48.90736}}),
                            setup,
                            service,
                            job_delivery,
                            job_empty_pickup,
                            {14}));

  for (const auto& j : jobs) {
    problem_instance.add_job(j);
  }

  // Define a shipment.
  vroom::Skills pd_skills({2});
  vroom::Amount pd_amount(amount_dimension);
  pd_amount[0] = 1;

  vroom::Job pickup(4,
                    vroom::JOB_TYPE::PICKUP,
                    vroom::Coordinates({{2.41808, 49.22619}}),
                    setup,
                    service,
                    pd_amount,
                    pd_skills);

  vroom::Job delivery(3,
                      vroom::JOB_TYPE::DELIVERY,
                      vroom::Coordinates({{2.39719, 49.07611}}),
                      setup,
                      service,
                      pd_amount,
                      pd_skills);
  problem_instance.add_shipment(pickup, delivery);

  // Skills definitions set the following constraints:
  // - jobs 1 and 2 can only be served by vehicle 1
  // - jobs 3 and 4 can only be served by vehicle 2
  // - jobs 5 and 6 can be served by either one of the vehicles

  // Solve!
  auto sol = problem_instance.solve(5,  // Exploration level.
                                    4); // Use 4 threads.

  log_solution(sol, GEOMETRY);
}

void run_example_with_custom_matrix() {
  bool GEOMETRY = false;
  unsigned amount_dimension = 0; // No capacity constraint.

  vroom::Input problem_instance(amount_dimension);

  // Define custom matrix and bypass OSRM call.
  vroom::Matrix<vroom::Cost> matrix_input(4);

  matrix_input[0][0] = 0;
  matrix_input[0][1] = 2104;
  matrix_input[0][2] = 197;
  matrix_input[0][3] = 1299;
  matrix_input[1][0] = 2103;
  matrix_input[1][1] = 0;
  matrix_input[1][2] = 2255;
  matrix_input[1][3] = 3152;
  matrix_input[2][0] = 197;
  matrix_input[2][1] = 2256;
  matrix_input[2][2] = 0;
  matrix_input[2][3] = 1102;
  matrix_input[3][0] = 1299;
  matrix_input[3][1] = 3153;
  matrix_input[3][2] = 1102;
  matrix_input[3][3] = 0;

  problem_instance.set_matrix("car", std::move(matrix_input));

  // Define vehicles (use std::nullopt for no start or no end).
  vroom::Location v_start(0); // index in the provided matrix.
  vroom::Location v_end(3);   // index in the provided matrix.

  vroom::Vehicle v(0,       // id
                   v_start, // start
                   v_end);  // end
  problem_instance.add_vehicle(v);

  // Define jobs with id and index of location in the matrix
  // (coordinates are optional). Constraints that are not required can
  // be omitted.
  std::vector<vroom::Job> jobs;
  jobs.push_back(vroom::Job(1414, 1));
  jobs.push_back(vroom::Job(1515, 2));

  for (const auto& j : jobs) {
    problem_instance.add_job(j);
  }

  // Solve!
  auto sol = problem_instance.solve(5,  // Exploration level.
                                    4); // Use 4 threads.

  log_solution(sol, GEOMETRY);
}

int main() {
  try {
    run_example_with_osrm();
    // run_example_with_custom_matrix();
  } catch (const vroom::Exception& e) {
    std::cerr << "[Error] " << e.message << std::endl;
  }
  return 0;
}
