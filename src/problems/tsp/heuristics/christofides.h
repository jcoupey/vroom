#ifndef CHRISTOFIDES_H
#define CHRISTOFIDES_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <list>

#include "structures/generic/matrix.h"
#include "structures/typedefs.h"

namespace vroom {
namespace tsp {

// Implementing a variant of the Christofides heuristic.
std::list<Index> christofides(const Matrix<Cost>& sym_matrix);

} // namespace tsp
} // namespace vroom

#endif
