/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "osrm/coordinate.hpp"
#include "osrm/json_container.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/status.hpp"
#include "osrm/table_parameters.hpp"

#include "routing/libosrm_wrapper.h"

namespace vroom {
namespace routing {

osrm::EngineConfig LibosrmWrapper::get_config(const std::string& profile) {
  osrm::EngineConfig config;

  // Only update non-default values.
  config.max_alternatives = 1;
  config.dataset_name = profile;

  return config;
}

LibosrmWrapper::LibosrmWrapper(const std::string& profile)
  : Wrapper(profile), _config(get_config(profile)), _osrm(_config) {
}

Matrix<Cost>
LibosrmWrapper::get_matrix(const std::vector<Location>& locs) const {
  osrm::TableParameters params;
  for (auto const& location : locs) {
    assert(location.has_coordinates());
    params.coordinates
      .emplace_back(osrm::util::FloatLongitude({location.lon()}),
                    osrm::util::FloatLatitude({location.lat()}));
  }

  osrm::json::Object result;
  osrm::Status status = _osrm.Table(params, result);

  if (status == osrm::Status::Error) {
    throw Exception(ERROR::ROUTING,
                    "libOSRM: " +
                      result.values["code"].get<osrm::json::String>().value +
                      ": " +
                      result.values["message"].get<osrm::json::String>().value);
  }

  auto& table = result.values["durations"].get<osrm::json::Array>();

  // Expected matrix size.
  std::size_t m_size = locs.size();
  assert(table.values.size() == m_size);

  // Build matrix while checking for unfound routes to avoid
  // unexpected behavior (OSRM raises 'null').
  Matrix<Cost> m(m_size);

  std::vector<unsigned> nb_unfound_from_loc(m_size, 0);
  std::vector<unsigned> nb_unfound_to_loc(m_size, 0);

  std::string reason;
  for (std::size_t i = 0; i < m_size; ++i) {
    const auto& line = table.values.at(i).get<osrm::json::Array>();
    assert(line.values.size() == m_size);
    for (std::size_t j = 0; j < m_size; ++j) {
      const auto& el = line.values.at(j);
      if (el.is<osrm::json::Null>()) {
        // No route found between i and j. Just storing info as we
        // don't know yet which location is responsible between i
        // and j.
        ++nb_unfound_from_loc[i];
        ++nb_unfound_to_loc[j];
      } else {
        auto cost = round_cost(el.get<osrm::json::Number>().value);
        m[i][j] = cost;
      }
    }
  }

  check_unfound(locs, nb_unfound_from_loc, nb_unfound_to_loc);

  return m;
}

void LibosrmWrapper::add_route_info(Route& route) const {
  // Default options for routing.
  osrm::RouteParameters params(false, // steps
                               false, // alternatives
                               false, // annotations
                               osrm::RouteParameters::GeometriesType::Polyline,
                               osrm::RouteParameters::OverviewType::Full,
                               false // continue_straight
  );

  // Ordering locations for the given steps, excluding
  // breaks.
  std::vector<unsigned> number_breaks_after;
  for (auto& step : route.steps) {
    if (step.step_type == STEP_TYPE::BREAK) {
      if (!number_breaks_after.empty()) {
        ++(number_breaks_after.back());
      }
    } else {
      params.coordinates
        .emplace_back(osrm::util::FloatLongitude({step.location.lon()}),
                      osrm::util::FloatLatitude({step.location.lat()}));
      number_breaks_after.push_back(0);
    }
  }
  assert(!number_breaks_after.empty());

  osrm::json::Object result;
  osrm::Status status = _osrm.Route(params, result);

  if (status == osrm::Status::Error) {
    throw Exception(ERROR::ROUTING,
                    result.values["code"].get<osrm::json::String>().value +
                      ": " +
                      result.values["message"].get<osrm::json::String>().value);
  }

  auto& result_routes = result.values["routes"].get<osrm::json::Array>();
  auto& json_route = result_routes.values.at(0).get<osrm::json::Object>();

  // Total distance and route geometry.
  route.distance =
    round_cost(json_route.values["distance"].get<osrm::json::Number>().value);
  route.geometry =
    std::move(json_route.values["geometry"].get<osrm::json::String>().value);

  auto& legs = json_route.values["legs"].get<osrm::json::Array>();
  auto nb_legs = legs.values.size();
  assert(nb_legs == number_breaks_after.size() - 1);

  double sum_distance = 0;

  // Locate first non-break stop.
  const auto first_non_break =
    std::find_if(route.steps.begin(), route.steps.end(), [&](const auto& s) {
      return s.step_type != STEP_TYPE::BREAK;
    });
  unsigned steps_rank = std::distance(route.steps.begin(), first_non_break);

  // Zero distance up to first non-break step.
  for (unsigned i = 0; i <= steps_rank; ++i) {
    route.steps[i].distance = 0;
  }

  for (unsigned i = 0; i < nb_legs; ++i) {
    const auto& step = route.steps[steps_rank];

    // Next element in steps that is not a break and associated
    // distance after current route leg.
    auto& next_step = route.steps[steps_rank + number_breaks_after[i] + 1];
    Duration next_duration = next_step.duration - step.duration;

    auto& leg = legs.values.at(i).get<osrm::json::Object>();
    double next_distance =
      leg.values["distance"].get<osrm::json::Number>().value;

    // Pro rata temporis distance update for breaks between current
    // non-breaks steps.
    for (unsigned b = 1; b <= number_breaks_after[i]; ++b) {
      auto& break_step = route.steps[steps_rank + b];
      break_step.distance = round_cost(
        sum_distance + ((break_step.duration - step.duration) * next_distance) /
                         next_duration);
    }

    sum_distance += next_distance;
    next_step.distance = round_cost(sum_distance);

    steps_rank += number_breaks_after[i] + 1;
  }

  // Unchanged distance after last non-break step.
  for (auto i = steps_rank; i < route.steps.size(); ++i) {
    route.steps[i].distance = sum_distance;
  }
}

} // namespace routing
} // namespace vroom
