#ifndef OUTPUT_JSON_H
#define OUTPUT_JSON_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

#include "../include/rapidjson/document.h"
#include "structures/vroom/solution/solution.h"

namespace vroom {
namespace io {

rapidjson::Document to_json(const Solution& sol, bool geometry);

rapidjson::Value to_json(const Summary& summary,
                         bool geometry,
                         rapidjson::Document::AllocatorType& allocator);

rapidjson::Value to_json(const ComputingTimes& computing_times,
                         bool geometry,
                         rapidjson::Document::AllocatorType& allocator);

rapidjson::Value to_json(const Route& route,
                         bool geometry,
                         rapidjson::Document::AllocatorType& allocator);

rapidjson::Value to_json(const Step& s,
                         bool geometry,
                         rapidjson::Document::AllocatorType& allocator);

rapidjson::Value to_json(const Location& loc,
                         rapidjson::Document::AllocatorType& allocator);

void write_to_json(const Solution& sol,
                   bool geometry,
                   const std::string& output_file);

} // namespace io
} // namespace vroom

#endif
