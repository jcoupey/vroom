/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/generic/matrix.h"

namespace vroom {

template <class T> Line<T>::Line(std::size_t n) : parent(n) {
}

template <class T> Line<T>::Line(std::initializer_list<T> l) : parent(l) {
}

template <class T> Matrix<T>::Matrix(std::size_t n) : parent(n, Line<T>(n)) {
}

template <class T> Matrix<T>::Matrix() : Matrix(0) {
}

template <class T>
Matrix<T>::Matrix(std::initializer_list<Line<T>> l) : parent(l) {
}

template <class T>
Matrix<T> Matrix<T>::get_sub_matrix(const std::vector<Index>& indices) const {
  Matrix<T> sub_matrix(indices.size());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    for (std::size_t j = 0; j < indices.size(); ++j) {
      sub_matrix[i][j] = (*this)[indices[i]][indices[j]];
    }
  }
  return sub_matrix;
}

template class Line<Cost>;
template class Matrix<Cost>;

} // namespace vroom
