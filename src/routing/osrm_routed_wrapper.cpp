/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/osrm_routed_wrapper.h"

namespace vroom {
namespace routing {

OsrmRoutedWrapper::OsrmRoutedWrapper(const std::string& profile,
                                     const Server& server)
  : HttpWrapper(profile,
                server,
                "table",
                "route",
                "alternatives=false&steps=false&overview=full&continue_"
                "straight=false") {
}

std::string
OsrmRoutedWrapper::build_query(const std::vector<Location>& locations,
                               const std::string& service,
                               const std::string& extra_args) const {
  // Building query for osrm-routed
  std::string query = "GET /" + service;

  query += "/v1/" + profile + "/";

  // Adding locations.
  for (auto const& location : locations) {
    query += std::to_string(location.lon()) + "," +
             std::to_string(location.lat()) + ";";
  }
  query.pop_back(); // Remove trailing ';'.

  if (!extra_args.empty()) {
    query += "?" + extra_args;
  }

  query += " HTTP/1.1\r\n";
  query += "Host: " + _server.host + "\r\n";
  query += "Accept: */*\r\n";
  query += "Connection: close\r\n\r\n";

  return query;
}

void OsrmRoutedWrapper::parse_response(rapidjson::Document& infos,
                                       const std::string& json_content) const {
#ifdef NDEBUG
  infos.Parse(json_content.c_str());
#else
  assert(!infos.Parse(json_content.c_str()).HasParseError());
  assert(infos.HasMember("code"));
#endif
  if (infos["code"] != "Ok") {
    throw Exception(ERROR::ROUTING, std::string(infos["message"].GetString()));
  }
}

double
OsrmRoutedWrapper::get_total_distance(const rapidjson::Value& route) const {
  return route["distance"].GetDouble();
}

unsigned
OsrmRoutedWrapper::get_legs_number(const rapidjson::Value& route) const {
  return route["legs"].Size();
}

double OsrmRoutedWrapper::get_distance_for_leg(const rapidjson::Value& route,
                                               rapidjson::SizeType i) const {
  return route["legs"][i]["distance"].GetDouble();
}

} // namespace routing
} // namespace vroom
