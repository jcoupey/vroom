#ifndef INPUT_PARSER_H
#define INPUT_PARSER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/input/input.h"

namespace vroom {
namespace io {

struct CLArgs;

Input parse(const CLArgs& cl_args);

} // namespace io
} // namespace vroom

#endif
