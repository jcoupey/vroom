#ifndef OSRM_ROUTED_WRAPPER_H
#define OSRM_ROUTED_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/http_wrapper.h"

namespace vroom {
namespace routing {

class OsrmRoutedWrapper : public HttpWrapper {
private:
  virtual std::string build_query(const std::vector<Location>& locations,
                                  const std::string& service,
                                  const std::string& extra_args) const override;

  virtual void parse_response(rapidjson::Document& input,
                              const std::string& json_content) const override;

  virtual double
  get_total_distance(const rapidjson::Value& route) const override;

  virtual unsigned
  get_legs_number(const rapidjson::Value& route) const override;

  virtual double get_distance_for_leg(const rapidjson::Value& route,
                                      rapidjson::SizeType i) const override;

public:
  OsrmRoutedWrapper(const std::string& profile, const Server& server);
};

} // namespace routing
} // namespace vroom

#endif
