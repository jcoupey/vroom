#ifndef HTTP_WRAPPER_H
#define HTTP_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/
#include "../include/rapidjson/document.h"
#include "../include/rapidjson/error/en.h"

#include "routing/wrapper.h"
#include "structures/typedefs.h"
#include "structures/vroom/location.h"

namespace vroom {
namespace routing {

class HttpWrapper : public Wrapper {
private:
  std::string send_then_receive(const std::string& query) const;

  std::string ssl_send_then_receive(const std::string& query) const;

  static const std::string HTTPS_PORT;

protected:
  const Server _server;
  const std::string _matrix_service;
  const std::string _route_service;
  const std::string _extra_args;

  HttpWrapper(const std::string& profile,
              const Server& server,
              const std::string& matrix_service,
              const std::string& route_service,
              const std::string& extra_args);

  std::string run_query(const std::string& query) const;

  virtual std::string build_query(const std::vector<Location>& locations,
                                  const std::string& service,
                                  const std::string& extra_args = "") const = 0;

  virtual void parse_response(rapidjson::Document& input,
                              const std::string& json_content) const = 0;

  virtual Matrix<Cost>
  get_matrix(const std::vector<Location>& locs) const override;

  virtual double get_total_distance(const rapidjson::Value& route) const = 0;

  virtual unsigned get_legs_number(const rapidjson::Value& route) const = 0;

  virtual double get_distance_for_leg(const rapidjson::Value& route,
                                      rapidjson::SizeType i) const = 0;

  virtual void add_route_info(Route& route) const override;
};

} // namespace routing
} // namespace vroom

#endif
