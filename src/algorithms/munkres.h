#ifndef MUNKRES_H
#define MUNKRES_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <unordered_map>

#include "structures/generic/matrix.h"
#include "structures/typedefs.h"

namespace vroom {
namespace utils {

template <class T>
std::unordered_map<Index, Index>
minimum_weight_perfect_matching(const Matrix<T>& m);

template <class T>
std::unordered_map<Index, Index>
greedy_symmetric_approx_mwpm(const Matrix<T>& m);

} // namespace utils
} // namespace vroom

#endif
