/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>

#include "structures/cl_args.h"

namespace vroom {
namespace io {

const unsigned CLArgs::max_exploration_level = 5;

// Default values.
CLArgs::CLArgs()
  : geometry(false), router(ROUTER::OSRM), nb_threads(4), exploration_level(5) {
}

void update_host(Servers& servers, const std::string& value) {
  // Determine profile and host from a "car:0.0.0.0"-like value.
  std::string profile = DEFAULT_PROFILE;
  std::string host;

  auto index = value.find(':');
  if (index == std::string::npos) {
    host = value;
  } else {
    profile = value.substr(0, index);
    host = value.substr(index + 1);
  }

  auto existing_profile = servers.find(profile);
  if (existing_profile != servers.end()) {
    existing_profile->second.host = host;
  } else {
    auto add_result = servers.emplace(profile, Server());
    assert(add_result.second);
    add_result.first->second.host = host;
  }
}

void update_port(Servers& servers, const std::string& value) {
  // Determine profile and port from a "car:0.0.0.0"-like value.
  std::string profile = DEFAULT_PROFILE;
  std::string port;

  auto index = value.find(':');
  if (index == std::string::npos) {
    port = value;
  } else {
    profile = value.substr(0, index);
    port = value.substr(index + 1);
  }

  auto existing_profile = servers.find(profile);
  if (existing_profile != servers.end()) {
    existing_profile->second.port = port;
  } else {
    auto add_result = servers.emplace(profile, Server());
    assert(add_result.second);
    add_result.first->second.port = port;
  }
}

} // namespace io
} // namespace vroom
