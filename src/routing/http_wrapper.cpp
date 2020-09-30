/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <asio.hpp>
#include <asio/ssl.hpp>

#include "routing/http_wrapper.h"

using asio::ip::tcp;

namespace vroom {
namespace routing {

const std::string HttpWrapper::HTTPS_PORT = "443";

HttpWrapper::HttpWrapper(const std::string& profile,
                         const Server& server,
                         const std::string& matrix_service,
                         const std::string& route_service,
                         const std::string& extra_args)
  : Wrapper(profile),
    _server(server),
    _matrix_service(matrix_service),
    _route_service(route_service),
    _extra_args(extra_args) {
}

std::string HttpWrapper::send_then_receive(const std::string& query) const {
  std::string response;

  try {
    asio::io_service io_service;

    tcp::resolver r(io_service);

    tcp::resolver::query q(_server.host, _server.port);

    tcp::socket s(io_service);
    asio::connect(s, r.resolve(q));

    asio::write(s, asio::buffer(query));

    char buf[512];
    std::error_code error;
    for (;;) {
      std::size_t len = s.read_some(asio::buffer(buf), error);
      response.append(buf, len);
      if (error == asio::error::eof) {
        // Connection closed cleanly.
        break;
      } else {
        if (error) {
          throw std::system_error(error);
        }
      }
    }
  } catch (std::system_error& e) {
    throw Exception(ERROR::ROUTING,
                    "Failed to connect to " + _server.host + ":" +
                      _server.port);
  }

  // Removing headers.
  auto start = response.find("{");
  assert(start != std::string::npos);
  auto end = response.rfind("}");
  assert(end != std::string::npos);

  std::string json_content = response.substr(start, end - start + 1);

  return json_content;
}

std::string HttpWrapper::ssl_send_then_receive(const std::string& query) const {
  std::string response;

  try {
    asio::io_service io_service;

    asio::ssl::context ctx(asio::ssl::context::method::sslv23_client);
    asio::ssl::stream<asio::ip::tcp::socket> ssock(io_service, ctx);

    tcp::resolver r(io_service);

    tcp::resolver::query q(_server.host, _server.port);

    asio::connect(ssock.lowest_layer(), r.resolve(q));
    ssock.handshake(asio::ssl::stream_base::handshake_type::client);

    asio::write(ssock, asio::buffer(query));

    char buf[512];
    std::error_code error;
    for (;;) {
      std::size_t len = ssock.read_some(asio::buffer(buf), error);
      response.append(buf, len);
      if (error == asio::error::eof) {
        // Connection closed cleanly.
        break;
      } else {
        if (error) {
          throw std::system_error(error);
        }
      }
    }
  } catch (std::system_error& e) {
    throw Exception(ERROR::ROUTING,
                    "Failed to connect to " + _server.host + ":" +
                      _server.port);
  }

  // Removing headers.
  auto start = response.find("{");
  assert(start != std::string::npos);
  auto end = response.rfind("}");
  assert(end != std::string::npos);

  std::string json_content = response.substr(start, end - start + 1);

  return json_content;
}

std::string HttpWrapper::run_query(const std::string& query) const {
  return (_server.port == HTTPS_PORT) ? ssl_send_then_receive(query)
                                      : send_then_receive(query);
}

Matrix<Cost> HttpWrapper::get_matrix(const std::vector<Location>& locs) const {
  std::string query = this->build_query(locs, _matrix_service);
  std::string json_content = this->run_query(query);

  // Expected matrix size.
  std::size_t m_size = locs.size();

  rapidjson::Document infos;
  this->parse_response(infos, json_content);

  assert(infos["durations"].Size() == m_size);

  // Build matrix while checking for unfound routes ('null' values) to
  // avoid unexpected behavior.
  Matrix<Cost> m(m_size);

  std::vector<unsigned> nb_unfound_from_loc(m_size, 0);
  std::vector<unsigned> nb_unfound_to_loc(m_size, 0);

  for (rapidjson::SizeType i = 0; i < infos["durations"].Size(); ++i) {
    const auto& line = infos["durations"][i];
    assert(line.Size() == m_size);
    for (rapidjson::SizeType j = 0; j < line.Size(); ++j) {
      if (line[j].IsNull()) {
        // No route found between i and j. Just storing info as we
        // don't know yet which location is responsible between i
        // and j.
        ++nb_unfound_from_loc[i];
        ++nb_unfound_to_loc[j];
      } else {
        auto cost = round_cost(line[j].GetDouble());
        m[i][j] = cost;
      }
    }
  }

  check_unfound(locs, nb_unfound_from_loc, nb_unfound_to_loc);

  return m;
}

void HttpWrapper::add_route_info(Route& route) const {
  // Ordering locations for the given steps, excluding
  // breaks.
  std::vector<Location> non_break_locations;
  std::vector<unsigned> number_breaks_after;

  for (const auto& step : route.steps) {
    if (step.step_type == STEP_TYPE::BREAK) {
      if (!number_breaks_after.empty()) {
        ++(number_breaks_after.back());
      }
    } else {
      non_break_locations.push_back(step.location);
      number_breaks_after.push_back(0);
    }
  }
  assert(!non_break_locations.empty());

  std::string query =
    build_query(non_break_locations, _route_service, _extra_args);

  std::string json_content = this->run_query(query);

  rapidjson::Document infos;
  parse_response(infos, json_content);

  // Total distance and route geometry.
  route.distance = round_cost(get_total_distance(infos["routes"][0]));
  route.geometry = std::move(infos["routes"][0]["geometry"].GetString());

  auto nb_legs = get_legs_number(infos["routes"][0]);
  assert(nb_legs == non_break_locations.size() - 1);

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

  for (rapidjson::SizeType i = 0; i < nb_legs; ++i) {
    const auto& step = route.steps[steps_rank];

    // Next element in steps that is not a break and associated
    // distance after current route leg.
    auto& next_step = route.steps[steps_rank + number_breaks_after[i] + 1];
    Duration next_duration = next_step.duration - step.duration;
    double next_distance = get_distance_for_leg(infos["routes"][0], i);

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
