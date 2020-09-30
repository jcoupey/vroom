#ifndef VROOM_TYPEDEFS_H
#define VROOM_TYPEDEFS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <array>
#include <limits>
#include <list>
#include <string>
#include <unordered_set>
#include <vector>
#include <map>

namespace vroom {

// To easily differentiate variable types.
using Id = uint64_t;
using Index = uint16_t;
using Cost = uint32_t;
using Gain = int64_t;
using Distance = uint32_t;
using Duration = uint32_t;
using Coordinate = double;
using Capacity = int64_t;
using Skill = uint32_t;
using Priority = uint32_t;

// Type helpers.
using Coordinates = std::array<Coordinate, 2>;
using OptionalCoordinates = std::optional<Coordinates>;
using Skills = std::unordered_set<Skill>;
using Priorities = std::map<Id, Priority>;

// Setting max value would cause trouble with further additions.
constexpr Cost INFINITE_COST = 3 * (std::numeric_limits<Cost>::max() / 4);

const std::string DEFAULT_PROFILE = "car";

constexpr Priority MAX_PRIORITY = 100;

// Available routing engines.
enum class ROUTER { OSRM, LIBOSRM, ORS };

// Used to describe a routing server.
struct Server {
  std::string host;
  std::string port;

  Server() : host("0.0.0.0"), port("5000") {
  }

  Server(const std::string& host, const std::string& port)
    : host(host), port(port) {
  }
};

// Specific error statuses used when handling exceptions.
enum class ERROR { INTERNAL, INPUT, ROUTING };

// 'Single' job is a regular one-stop job without precedence
// constraints.
enum class JOB_TYPE { SINGLE, PICKUP, DELIVERY };

// Available location status.
enum class STEP_TYPE { START, JOB, BREAK, END };

// Heuristic options.
enum class HEURISTIC { BASIC, DYNAMIC };
enum class INIT { NONE, HIGHER_AMOUNT, NEAREST, FURTHEST, EARLIEST_DEADLINE };

struct HeuristicParameters {
  HEURISTIC heuristic;
  INIT init;
  float regret_coeff;

  constexpr HeuristicParameters(HEURISTIC heuristic,
                                INIT init,
                                float regret_coeff)
    : heuristic(heuristic), init(init), regret_coeff(regret_coeff) {
  }
};

} // namespace vroom

#endif
