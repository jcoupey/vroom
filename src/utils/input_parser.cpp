/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>

#include "../include/rapidjson/document.h"
#include "../include/rapidjson/error/en.h"

#include "structures/cl_args.h"
#include "utils/input_parser.h"

namespace vroom {
namespace io {

// Helper to get optional array of coordinates.
inline Coordinates parse_coordinates(const rapidjson::Value& object,
                                     const char* key) {
  if (!object[key].IsArray() or (object[key].Size() < 2) or
      !object[key][0].IsNumber() or !object[key][1].IsNumber()) {
    throw Exception(ERROR::INPUT, "Invalid " + std::string(key) + " array.");
  }
  return {{object[key][0].GetDouble(), object[key][1].GetDouble()}};
}

inline std::string get_string(const rapidjson::Value& object, const char* key) {
  std::string value;
  if (object.HasMember(key)) {
    if (!object[key].IsString()) {
      throw Exception(ERROR::INPUT, "Invalid " + std::string(key) + " value.");
    }
    value = object[key].GetString();
  }
  return value;
}

inline double get_double(const rapidjson::Value& object, const char* key) {
  double value = 1.;
  if (object.HasMember(key)) {
    if (!object[key].IsNumber()) {
      throw Exception(ERROR::INPUT, "Invalid " + std::string(key) + " value.");
    }
    value = object[key].GetDouble();
  }
  return value;
}

inline Amount get_amount(const rapidjson::Value& object,
                         const char* key,
                         unsigned size) {
  // Default to zero amount with provided size.
  Amount amount(size);

  if (object.HasMember(key)) {
    if (!object[key].IsArray()) {
      throw Exception(ERROR::INPUT, "Invalid " + std::string(key) + " array.");
    }

    if (object[key].Size() != size) {
      throw Exception(ERROR::INPUT,
                      "Inconsistent " + std::string(key) +
                        " length: " + std::to_string(object[key].Size()) +
                        " and " + std::to_string(size) + '.');
    }

    for (rapidjson::SizeType i = 0; i < object[key].Size(); ++i) {
      if (!object[key][i].IsUint()) {
        throw Exception(ERROR::INPUT,
                        "Invalid " + std::string(key) + " value.");
      }
      amount[i] = object[key][i].GetUint();
    }
  }

  return amount;
}

inline Skills get_skills(const rapidjson::Value& object) {
  Skills skills;
  if (object.HasMember("skills")) {
    if (!object["skills"].IsArray()) {
      throw Exception(ERROR::INPUT, "Invalid skills object.");
    }
    for (rapidjson::SizeType i = 0; i < object["skills"].Size(); ++i) {
      if (!object["skills"][i].IsUint()) {
        throw Exception(ERROR::INPUT, "Invalid skill value.");
      }
      skills.insert(object["skills"][i].GetUint());
    }
  }

  return skills;
}

inline Duration get_duration(const rapidjson::Value& object, const char* key) {
  Duration duration = 0;
  if (object.HasMember(key)) {
    if (!object[key].IsUint()) {
      throw Exception(ERROR::INPUT,
                      "Invalid " + std::string(key) + " duration.");
    }
    duration = object[key].GetUint();
  }
  return duration;
}

inline Duration get_priority(const rapidjson::Value& object) {
  Priority priority = 0;
  if (object.HasMember("priority")) {
    if (!object["priority"].IsUint()) {
      throw Exception(ERROR::INPUT, "Invalid priority value.");
    }
    priority = object["priority"].GetUint();
    if (priority > MAX_PRIORITY) {
      throw Exception(ERROR::INPUT, "Invalid priority value.");
    }
  }
  return priority;
}

inline void check_id(const rapidjson::Value& v, const std::string& type) {
  if (!v.IsObject()) {
    throw Exception(ERROR::INPUT, "Invalid " + type + ".");
  }
  if (!v.HasMember("id") or !v["id"].IsUint64()) {
    throw Exception(ERROR::INPUT, "Invalid or missing id for " + type + ".");
  }
}

inline void check_shipment(const rapidjson::Value& v) {
  if (!v.IsObject()) {
    throw Exception(ERROR::INPUT, "Invalid shipment.");
  }
  if (!v.HasMember("pickup") or !v["pickup"].IsObject()) {
    throw Exception(ERROR::INPUT, "Missing pickup for shipment.");
  }
  if (!v.HasMember("delivery") or !v["delivery"].IsObject()) {
    throw Exception(ERROR::INPUT, "Missing delivery for shipment.");
  }
}

inline void check_location(const rapidjson::Value& v, const std::string& type) {
  if (!v.HasMember("location") or !v["location"].IsArray()) {
    throw Exception(ERROR::INPUT,
                    "Invalid location for " + type + " " +
                      std::to_string(v["id"].GetUint64()) + ".");
  }
}

inline TimeWindow get_time_window(const rapidjson::Value& tw) {
  if (!tw.IsArray() or tw.Size() < 2 or !tw[0].IsUint() or !tw[1].IsUint()) {
    throw Exception(ERROR::INPUT, "Invalid time-window.");
  }
  return TimeWindow(tw[0].GetUint(), tw[1].GetUint());
}

inline TimeWindow get_vehicle_time_window(const rapidjson::Value& v) {
  TimeWindow v_tw = TimeWindow();
  if (v.HasMember("time_window")) {
    v_tw = get_time_window(v["time_window"]);
  }
  return v_tw;
}

inline std::vector<TimeWindow> get_job_time_windows(const rapidjson::Value& j) {
  std::vector<TimeWindow> tws;
  if (j.HasMember("time_windows")) {
    if (!j["time_windows"].IsArray() or j["time_windows"].Empty()) {
      throw Exception(ERROR::INPUT,
                      "Invalid time_windows array for job " +
                        std::to_string(j["id"].GetUint64()) + ".");
    }

    std::transform(j["time_windows"].Begin(),
                   j["time_windows"].End(),
                   std::back_inserter(tws),
                   [](auto& tw) { return get_time_window(tw); });

    std::sort(tws.begin(), tws.end());
  } else {
    tws = std::vector<TimeWindow>(1, TimeWindow());
  }

  return tws;
}

inline std::vector<TimeWindow>
get_break_time_windows(const rapidjson::Value& b) {
  std::vector<TimeWindow> tws;
  if (!b.HasMember("time_windows") or !b["time_windows"].IsArray() or
      b["time_windows"].Empty()) {
    throw Exception(ERROR::INPUT,
                    "Invalid time_windows array for break " +
                      std::to_string(b["id"].GetUint64()) + ".");
  }

  std::transform(b["time_windows"].Begin(),
                 b["time_windows"].End(),
                 std::back_inserter(tws),
                 [](auto& tw) { return get_time_window(tw); });

  std::sort(tws.begin(), tws.end());

  return tws;
}

inline Break get_break(const rapidjson::Value& b) {
  check_id(b, "break");
  return Break(b["id"].GetUint64(),
               get_break_time_windows(b),
               get_duration(b, "service"),
               get_string(b, "description"));
}

inline std::vector<Break> get_vehicle_breaks(const rapidjson::Value& v) {
  std::vector<Break> breaks;
  if (v.HasMember("breaks")) {
    if (!v["breaks"].IsArray()) {
      throw Exception(ERROR::INPUT,
                      "Invalid breaks for vehicle " +
                        std::to_string(v["id"].GetUint64()) + ".");
    }

    std::transform(v["breaks"].Begin(),
                   v["breaks"].End(),
                   std::back_inserter(breaks),
                   [](auto& b) { return get_break(b); });
  }

  std::sort(breaks.begin(), breaks.end(), [](const auto& a, const auto& b) {
    return a.tws[0].start < b.tws[0].start or
           (a.tws[0].start == b.tws[0].start and a.tws[0].end < b.tws[0].end);
  });

  return breaks;
}

inline std::vector<VehicleStep> get_vehicle_steps(const rapidjson::Value& v) {
  std::vector<VehicleStep> steps;

  if (v.HasMember("steps")) {
    if (!v["steps"].IsArray()) {
      throw Exception(ERROR::INPUT,
                      "Invalid steps for vehicle " +
                        std::to_string(v["id"].GetUint64()) + ".");
    }

    for (rapidjson::SizeType i = 0; i < v["steps"].Size(); ++i) {
      const auto& json_step = v["steps"][i];

      std::optional<Duration> at;
      if (json_step.HasMember("service_at")) {
        if (!json_step["service_at"].IsUint()) {
          throw Exception(ERROR::INPUT, "Invalid service_at value.");
        }

        at = json_step["service_at"].GetUint();
      }
      std::optional<Duration> after;
      if (json_step.HasMember("service_after")) {
        if (!json_step["service_after"].IsUint()) {
          throw Exception(ERROR::INPUT, "Invalid service_after value.");
        }

        after = json_step["service_after"].GetUint();
      }
      std::optional<Duration> before;
      if (json_step.HasMember("service_before")) {
        if (!json_step["service_before"].IsUint()) {
          throw Exception(ERROR::INPUT, "Invalid service_before value.");
        }

        before = json_step["service_before"].GetUint();
      }
      ForcedService forced_service(std::move(at),
                                   std::move(after),
                                   std::move(before));

      const auto type_str = get_string(json_step, "type");

      if (type_str == "start") {
        steps.emplace_back(STEP_TYPE::START, std::move(forced_service));
        continue;
      }
      if (type_str == "end") {
        steps.emplace_back(STEP_TYPE::END, std::move(forced_service));
        continue;
      }

      if (!json_step.HasMember("id") or !json_step["id"].IsUint64()) {
        throw Exception(ERROR::INPUT,
                        "Invalid id in steps for vehicle " +
                          std::to_string(v["id"].GetUint64()) + ".");
      }

      if (type_str == "job") {
        steps.emplace_back(JOB_TYPE::SINGLE,
                           json_step["id"].GetUint64(),
                           std::move(forced_service));
      } else if (type_str == "pickup") {
        steps.emplace_back(JOB_TYPE::PICKUP,
                           json_step["id"].GetUint64(),
                           std::move(forced_service));
      } else if (type_str == "delivery") {
        steps.emplace_back(JOB_TYPE::DELIVERY,
                           json_step["id"].GetUint64(),
                           std::move(forced_service));
      } else if (type_str == "break") {
        steps.emplace_back(STEP_TYPE::BREAK,
                           json_step["id"].GetUint64(),
                           std::move(forced_service));
      } else {
        throw Exception(ERROR::INPUT,
                        "Invalid type in steps for vehicle " +
                          std::to_string(v["id"].GetUint64()) + ".");
      }
    }
  }

  return steps;
}

inline Vehicle get_vehicle(const rapidjson::Value& json_vehicle,
                           unsigned amount_size) {
  check_id(json_vehicle, "vehicle");
  auto v_id = json_vehicle["id"].GetUint64();

  // Check what info are available for vehicle start, then build
  // optional start location.
  bool has_start_coords = json_vehicle.HasMember("start");
  bool has_start_index = json_vehicle.HasMember("start_index");
  if (has_start_index and !json_vehicle["start_index"].IsUint()) {
    throw Exception(ERROR::INPUT,
                    "Invalid start_index for vehicle " + std::to_string(v_id) +
                      ".");
  }

  std::optional<Location> start;
  if (has_start_index) {
    // Custom provided matrices and index.
    Index start_index = json_vehicle["start_index"].GetUint();
    if (has_start_coords) {
      start = Location({start_index, parse_coordinates(json_vehicle, "start")});
    } else {
      start = start_index;
    }
  } else {
    if (has_start_coords) {
      start = parse_coordinates(json_vehicle, "start");
    }
  }

  // Check what info are available for vehicle end, then build
  // optional end location.
  bool has_end_coords = json_vehicle.HasMember("end");
  bool has_end_index = json_vehicle.HasMember("end_index");
  if (has_end_index and !json_vehicle["end_index"].IsUint()) {
    throw Exception(ERROR::INPUT,
                    "Invalid end_index for vehicle" + std::to_string(v_id) +
                      ".");
  }

  std::optional<Location> end;
  if (has_end_index) {
    // Custom provided matrices and index.
    Index end_index = json_vehicle["end_index"].GetUint();
    if (has_end_coords) {
      end = Location({end_index, parse_coordinates(json_vehicle, "end")});
    } else {
      end = end_index;
    }
  } else {
    if (has_end_coords) {
      end = parse_coordinates(json_vehicle, "end");
    }
  }

  std::string profile = get_string(json_vehicle, "profile");
  if (profile.empty()) {
    profile = DEFAULT_PROFILE;
  }

  return Vehicle(v_id,
                 start,
                 end,
                 profile,
                 get_amount(json_vehicle, "capacity", amount_size),
                 get_skills(json_vehicle),
                 get_vehicle_time_window(json_vehicle),
                 get_vehicle_breaks(json_vehicle),
                 get_string(json_vehicle, "description"),
                 get_double(json_vehicle, "speed_factor"),
                 get_vehicle_steps(json_vehicle));
}

inline Location get_task_location(const rapidjson::Value& v,
                                  const std::string& type) {
  // Check what info are available to build task location.
  bool has_location_coords = v.HasMember("location");
  bool has_location_index = v.HasMember("location_index");
  if (has_location_index and !v["location_index"].IsUint()) {
    throw Exception(ERROR::INPUT,
                    "Invalid location_index for " + type + " " +
                      std::to_string(v["id"].GetUint64()) + ".");
  }

  if (has_location_index) {
    // Custom provided matrices and index.
    Index location_index = v["location_index"].GetUint();
    if (has_location_coords) {
      return Location({location_index, parse_coordinates(v, "location")});
    } else {
      return Location(location_index);
    }
  } else {
    check_location(v, type);
    return parse_coordinates(v, "location");
  }
}

inline Job get_job(const rapidjson::Value& json_job, unsigned amount_size) {
  check_id(json_job, "job");

  // Only for retro-compatibility: when no pickup and delivery keys
  // are defined and (deprecated) amount key is present, it should be
  // interpreted as a delivery.
  bool need_amount_compat = json_job.HasMember("amount") and
                            !json_job.HasMember("delivery") and
                            !json_job.HasMember("pickup");

  return Job(json_job["id"].GetUint64(),
             get_task_location(json_job, "job"),
             get_duration(json_job, "setup"),
             get_duration(json_job, "service"),
             need_amount_compat ? get_amount(json_job, "amount", amount_size)
                                : get_amount(json_job, "delivery", amount_size),
             get_amount(json_job, "pickup", amount_size),
             get_skills(json_job),
             get_priority(json_job),
             get_job_time_windows(json_job),
             get_string(json_job, "description"));
}

inline Matrix<Cost> get_matrix(rapidjson::Value& m) {
  if (!m.IsArray()) {
    throw Exception(ERROR::INPUT, "Invalid matrix.");
  }
  // Load custom matrix while checking if it is square.
  rapidjson::SizeType matrix_size = m.Size();

  Matrix<Cost> matrix(matrix_size);
  for (rapidjson::SizeType i = 0; i < matrix_size; ++i) {
    if (!m[i].IsArray() or m[i].Size() != matrix_size) {
      throw Exception(ERROR::INPUT, "Unexpected matrix line length.");
    }
    rapidjson::Document::Array mi = m[i].GetArray();
    for (rapidjson::SizeType j = 0; j < matrix_size; ++j) {
      if (!mi[j].IsUint()) {
        throw Exception(ERROR::INPUT, "Invalid matrix entry.");
      }
      matrix[i][j] = mi[j].GetUint();
    }
  }

  return matrix;
}

Input parse(const CLArgs& cl_args) {
  // Input json object.
  rapidjson::Document json_input;

  // Parsing input string to populate the input object.
  if (json_input.Parse(cl_args.input.c_str()).HasParseError()) {
    std::string error_msg =
      std::string(rapidjson::GetParseError_En(json_input.GetParseError())) +
      " (offset: " + std::to_string(json_input.GetErrorOffset()) + ")";
    throw Exception(ERROR::INPUT, error_msg);
  }

  // Main checks for valid json input.
  bool has_jobs = json_input.HasMember("jobs") and
                  json_input["jobs"].IsArray() and !json_input["jobs"].Empty();
  bool has_shipments = json_input.HasMember("shipments") and
                       json_input["shipments"].IsArray() and
                       !json_input["shipments"].Empty();
  if (!has_jobs and !has_shipments) {
    throw Exception(ERROR::INPUT, "Invalid jobs or shipments.");
  }

  if (!json_input.HasMember("vehicles") or !json_input["vehicles"].IsArray() or
      json_input["vehicles"].Empty()) {
    throw Exception(ERROR::INPUT, "Invalid vehicles.");
  }
  const auto& first_vehicle = json_input["vehicles"][0];
  check_id(first_vehicle, "vehicle");
  bool first_vehicle_has_capacity = (first_vehicle.HasMember("capacity") and
                                     first_vehicle["capacity"].IsArray() and
                                     first_vehicle["capacity"].Size() > 0);
  const unsigned amount_size =
    first_vehicle_has_capacity ? first_vehicle["capacity"].Size() : 0;

  // Custom input object embedding jobs, vehicles and matrices.
  Input input(amount_size, cl_args.servers, cl_args.router);
  input.set_geometry(cl_args.geometry);

  // Add all vehicles.
  for (rapidjson::SizeType i = 0; i < json_input["vehicles"].Size(); ++i) {
    auto& json_vehicle = json_input["vehicles"][i];

    input.add_vehicle(get_vehicle(json_vehicle, amount_size));
  }

  // Add all tasks.
  if (has_jobs) {
    // Add the jobs.
    for (rapidjson::SizeType i = 0; i < json_input["jobs"].Size(); ++i) {
      input.add_job(get_job(json_input["jobs"][i], amount_size));
    }
  }

  if (has_shipments) {
    // Add the shipments.
    for (rapidjson::SizeType i = 0; i < json_input["shipments"].Size(); ++i) {
      auto& json_shipment = json_input["shipments"][i];
      check_shipment(json_shipment);

      // Retrieve common stuff for both pickup and delivery.
      auto amount = get_amount(json_shipment, "amount", amount_size);
      auto skills = get_skills(json_shipment);
      auto priority = get_priority(json_shipment);

      // Defining pickup job.
      auto& json_pickup = json_shipment["pickup"];
      check_id(json_pickup, "pickup");

      Job pickup(json_pickup["id"].GetUint64(),
                 JOB_TYPE::PICKUP,
                 get_task_location(json_pickup, "pickup"),
                 get_duration(json_pickup, "setup"),
                 get_duration(json_pickup, "service"),
                 amount,
                 skills,
                 priority,
                 get_job_time_windows(json_pickup),
                 get_string(json_pickup, "description"));

      // Defining delivery job.
      auto& json_delivery = json_shipment["delivery"];
      check_id(json_delivery, "delivery");

      Job delivery(json_delivery["id"].GetUint64(),
                   JOB_TYPE::DELIVERY,
                   get_task_location(json_delivery, "delivery"),
                   get_duration(json_delivery, "setup"),
                   get_duration(json_delivery, "service"),
                   amount,
                   skills,
                   priority,
                   get_job_time_windows(json_delivery),
                   get_string(json_delivery, "description"));

      input.add_shipment(pickup, delivery);
    }
  }

  if (json_input.HasMember("matrices")) {
    if (!json_input["matrices"].IsObject()) {
      throw Exception(ERROR::INPUT, "Unexpected matrices value.");
    }
    for (auto& profile_entry : json_input["matrices"].GetObject()) {
      if (profile_entry.value.IsObject() and
          profile_entry.value.HasMember("durations")) {
        input.set_matrix(profile_entry.name.GetString(),
                         get_matrix(profile_entry.value["durations"]));
      }
    }
  } else {
    // Deprecated `matrix` key still interpreted as
    // `matrices.DEFAULT_PROFILE.duration` for retro-compatibility.
    if (json_input.HasMember("matrix")) {
      input.set_matrix(DEFAULT_PROFILE, get_matrix(json_input["matrix"]));
    }
  }

  return input;
}

} // namespace io
} // namespace vroom
