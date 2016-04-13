#ifndef OSRM_WRAPPER_H
#define OSRM_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>
#include <limits>
#include <regex>
#include <boost/asio.hpp>
#include "./problem_io.h"
#include "../structures/matrix.h"
#include "../utils/exceptions.h"

using boost::asio::ip::tcp;

class osrm_wrapper : public problem_io<distance_t>{

private:
  const std::string _address;      // OSRM server adress
  const std::string _port;         // OSRM server listening port
  const std::string _osrm_profile; // OSRM profile name
  const bool _use_osrm_v5;         // For backward compat
  std::vector<std::pair<double, double>> _locations;

  std::string build_query(const std::vector<std::pair<double, double>>& locations, 
                          std::string service, 
                          std::string extra_args = "") const{
    // Building query for osrm-routed
    std::string query = "GET /" + service;

    if(_use_osrm_v5){
      query += "/v1/" + _osrm_profile + "/";

      // Adding locations.
      for(auto const& location: locations){
        // OSRM v5 has gone [lon,lat].
        query += std::to_string(location.second)
          + ","
          + std::to_string(location.first)
          + ";";
      }
      query.pop_back();         // Remove trailing ';'.
      
      if(!extra_args.empty()){
        query += "?" + extra_args;
      }
    }
    else{
      // Backward compat.
      query += "?";

      // Adding locations.
      for(auto const& location: locations){
        query += "loc="
          + std::to_string(location.first)
          + ","
          + std::to_string(location.second)
          + "&";
      }

      if(!extra_args.empty()){
        query += extra_args + "&";
      }
      query.pop_back();         // Remove trailing '&'.
    }
    
    query += " HTTP/1.1\r\n";
    query += "Host: " + _address + "\r\n";
    query += "Accept: */*\r\n";
    query += "Connection: close\r\n\r\n";

    return query;
  }

  std::string send_then_receive(std::string query) const{
    std::string response;

    try{
      boost::asio::io_service io_service;
    
      tcp::resolver r (io_service);
      tcp::resolver::query q (_address, _port);

      tcp::socket s (io_service);
      boost::asio::connect(s, r.resolve(q));

      boost::asio::write(s, boost::asio::buffer(query));

      char buf[512];
      boost::system::error_code error;
      for(;;){
        std::size_t len = s.read_some(boost::asio::buffer(buf), error);
        response.append(buf, len);
        if(error == boost::asio::error::eof){
          // Connection closed cleanly.
          break;
        }
        else{
          if(error){
            throw boost::system::system_error(error);
          }
        }
      }
    }
    catch (boost::system::system_error& e)
      {
        throw custom_exception("Failure while connecting to the OSRM server.");
      }
    return response;
  }

  void add_location(const std::string location){
    // Regex check for valid location.
    std::regex valid_loc ("loc=-?[0-9]+\\.?[0-9]*,-?[0-9]+\\.?[0-9]*[[:space:]]*");
    if(!std::regex_match(location, valid_loc)){
      throw custom_exception("Invalid syntax for location "
                             + std::to_string(_locations.size() + 1)
                             + ", see vroom -h for usage display."
                             );
    }

    // Parsing the location is now safe.
    std::size_t separator_rank = location.find(",");
    std::string lat = location.substr(4, separator_rank);
    std::string lon = location.substr(separator_rank + 1, location.length() -1);
    _locations.emplace_back(std::stod(lat, nullptr),
                            std::stod(lon, nullptr));
  }

public:
  osrm_wrapper(std::string address, 
               std::string port,
               std::string osrm_profile,
               std::string loc_input):
    _address(address),
    _port(port),
    _osrm_profile(osrm_profile),
    _use_osrm_v5(!_osrm_profile.empty()){

    // Parsing input in locations.
    std::size_t start = 0;
    std::size_t end = loc_input.find("&", start);
    while(end != std::string::npos){
      this->add_location(loc_input.substr(start, end - start));
      start = end + 1;
      end = loc_input.find("&", start);
    }
    // Adding last element, after last "&".
    end = loc_input.length();
    this->add_location(loc_input.substr(start, end - start));
    
    if(_locations.size() <= 1){
      throw custom_exception("At least two locations required!");
    }
  }

  virtual matrix<distance_t> get_matrix() const override{
    std::string query = this->build_query(_locations, "table");

    std::string response = this->send_then_receive(query);

    if(!_use_osrm_v5){
      // Backward compat. Stop at "Bad Request" error from OSRM.
      assert(response.find("Bad Request") == std::string::npos);
    }

    // Removing headers.
    std::string json_content = response.substr(response.find("{"));

    // Expected matrix size.
    std::size_t m_size = _locations.size();
    // Matrix key label depends on OSRM version.
    const char* durations = _use_osrm_v5 ? "durations": "distance_table";

    // Checking everything is fine in the response (OSRM version
    // dependant).
    rapidjson::Document infos;
    assert(!infos.Parse(json_content.c_str()).HasParseError());
    if(_use_osrm_v5){
      assert(infos.HasMember("code"));
      if(infos["code"] != "Ok"){
        throw custom_exception("OSRM table: "
                               + std::string(infos["message"].GetString()));
      }
    }
    else{
      // Backward compat.
      assert(infos.HasMember(durations));
    }
    assert(infos[durations].Size() == m_size);

    // Building matrix and checking for unfound routes to avoid
    // unexpected behavior (OSRM raises 'null').
    matrix<distance_t> m {m_size};

    std::vector<unsigned> nb_unfound_from_loc (m_size, 0);
    std::vector<unsigned> nb_unfound_to_loc (m_size, 0);

    for(rapidjson::SizeType i = 0; i < infos[durations].Size(); ++i){
      const auto& line = infos[durations][i];
      assert(line.Size() == m_size);
      for(rapidjson::SizeType j = 0; j < line.Size(); ++j){
        if(line[j].IsNull()){
          // No route found between i and j. Just storing info as we
          // don't know yet which location is responsible between i
          // and j.
          ++nb_unfound_from_loc[i];
          ++nb_unfound_to_loc[j];
        }
        else{
          m[i][j] = static_cast<distance_t>(line[j].GetDouble() + 0.5);
        }
      }
    }
    
    unsigned max_unfound_routes_for_a_loc = 0;
    index_t error_loc = 0;    // Initial value never actually used.
    std::string error_direction;
    // Finding the "worst" location for unfound routes.
    for(unsigned i = 0; i < m_size; ++i){
      if(nb_unfound_from_loc[i] > max_unfound_routes_for_a_loc){
        max_unfound_routes_for_a_loc = nb_unfound_from_loc[i];
        error_loc = i;
        error_direction = "from";
      }
      if(nb_unfound_to_loc[i] > max_unfound_routes_for_a_loc){
        max_unfound_routes_for_a_loc = nb_unfound_to_loc[i];
        error_loc = i;
        error_direction = "to";
      }
    }
    if(max_unfound_routes_for_a_loc > 0){
      std::string error_msg = "OSRM has unfound route(s) ";
      error_msg += error_direction;
      error_msg += " location at index: ";
      error_msg += std::to_string(error_loc);
      throw custom_exception(error_msg);
    }

    return m;
  }

  virtual void get_route(const std::list<index_t>& tour,
                         rapidjson::Value& value,
                         rapidjson::Document::AllocatorType& allocator) const override{
    rapidjson::Value route_array(rapidjson::kArrayType);
    for(auto const& step: tour){
      route_array
        .PushBack(rapidjson::Value(rapidjson::kArrayType)
                  .PushBack(_locations[step].first, allocator)
                  .PushBack(_locations[step].second, allocator),
                  allocator);
    }
    value.Swap(route_array);
  }

  virtual void get_tour(const std::list<index_t>& tour,
                        rapidjson::Value& value,
                        rapidjson::Document::AllocatorType& allocator) const override{
    rapidjson::Value tour_array(rapidjson::kArrayType);
    for(auto const& step: tour){
      // Using input index to describe locations.
      tour_array.PushBack(step, allocator);
    }
    value.Swap(tour_array);
  }

  virtual void get_route_infos(const std::list<index_t>& tour,
                               rapidjson::Document& output) const override{
    // Ordering locations for the given tour.
    std::vector<std::pair<double, double>> ordered_locations;
    for(auto& step: tour){
      ordered_locations.push_back(_locations[step]);
    }

    std::string route_service = _use_osrm_v5 ? "route": "viaroute";

    // Backward compat.
    std::string extra_args = "alt=false&uturns=true";
    if(_use_osrm_v5){
      extra_args = "alternatives=false&steps=false&overview=full&continue_straight=false";
    }
    
    std::string query = this->build_query(ordered_locations,
                                          route_service,
                                          extra_args);
    std::string response = this->send_then_receive(query);

    // Removing headers
    std::string json_content = response.substr(response.find("{"));

    // Checking everything is fine in the response (OSRM version
    // dependant). Then parse total time/distance and route geometry.
    rapidjson::Document infos;
    rapidjson::Document::AllocatorType& allocator = output.GetAllocator();

    assert(!infos.Parse(json_content.c_str()).HasParseError());
    if(_use_osrm_v5){
      assert(infos.HasMember("code"));
      if(infos["code"] != "Ok"){
        throw custom_exception("OSRM route: "
                               + std::string(infos["message"].GetString()));
      }

      output.AddMember("total_time",
                       infos["routes"][0]["duration"],
                       allocator);
      output.AddMember("total_distance",
                       infos["routes"][0]["distance"],
                       allocator);
      output.AddMember("route_geometry",
                       rapidjson::Value(infos["routes"][0]["geometry"], allocator),
                       allocator);
    }
    else{
      // Backward compat.
      assert(infos.HasMember("route_summary"));
      assert(infos["route_summary"].HasMember("total_time"));
      assert(infos["route_summary"].HasMember("total_distance"));
      assert(infos.HasMember("route_geometry"));

      output.AddMember("total_time",
                       infos["route_summary"]["total_time"],
                       allocator);
      output.AddMember("total_distance",
                       infos["route_summary"]["total_distance"],
                       allocator);
      output.AddMember("route_geometry",
                       rapidjson::Value(infos["route_geometry"], allocator),
                       allocator);
    }
  }
};

#endif
