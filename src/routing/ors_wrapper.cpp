/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/ors_wrapper.h"

namespace vroom {
namespace routing {

OrsWrapper::OrsWrapper(const std::string& profile, const Server& server)
  : HttpWrapper(profile,
                server,
                "matrix",
                "directions",
                "\"geometry_simplify\":\"false\",\"continue_straight\":"
                "\"false\"") {
}

std::string OrsWrapper::build_query(const std::vector<Location>& locations,
                                    const std::string& service,
                                    const std::string& extra_args) const {
  // Adding locations.
  std::string body = "{\"";
  if (service == "directions") {
    body += "coordinates";
  } else {
    body += "locations";
  }
  body += "\":[";
  for (auto const& location : locations) {
    body += "[" + std::to_string(location.lon()) + "," +
            std::to_string(location.lat()) + "],";
  }
  body.pop_back(); // Remove trailing ','.
  body += "]";
  if (!extra_args.empty()) {
    body += "," + extra_args;
  }
  body += "}";

  // Building query for ORS
  std::string query = "POST /ors/v2/" + service + "/" + profile;

  query += " HTTP/1.0\r\n";
  query += "Accept: */*\r\n";
  query += "Content-Type: application/json\r\n";
  query += "Content-Length: " + std::to_string(body.size()) + "\r\n";
  query += "Host: " + _server.host + ":" + _server.port + "\r\n";
  query += "Connection: close\r\n";
  query += "\r\n" + body;

  return query;
}

void OrsWrapper::parse_response(rapidjson::Document& infos,
                                const std::string& json_content) const {
#ifdef NDEBUG
  infos.Parse(json_content.c_str());
#else
  assert(!infos.Parse(json_content.c_str()).HasParseError());
#endif
  if (infos.HasMember("error")) {
    throw Exception(ERROR::ROUTING,
                    std::string(infos["error"]["message"].GetString()));
  }
}

double OrsWrapper::get_total_distance(const rapidjson::Value& route) const {
  return route["summary"]["distance"].GetDouble();
}

unsigned OrsWrapper::get_legs_number(const rapidjson::Value& route) const {
  return route["segments"].Size();
}

double OrsWrapper::get_distance_for_leg(const rapidjson::Value& route,
                                        rapidjson::SizeType i) const {
  return route["segments"][i]["distance"].GetDouble();
}

} // namespace routing
} // namespace vroom
