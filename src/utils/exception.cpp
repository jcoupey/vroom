/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "utils/exception.h"

namespace vroom {

Exception::Exception(ERROR error, const std::string& message)
  : error(error), message(message) {
}

} // namespace vroom
