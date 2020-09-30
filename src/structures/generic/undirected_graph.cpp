/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/
#include <cassert>

#include "structures/generic/undirected_graph.h"

namespace vroom {
namespace utils {

template <class T> UndirectedGraph<T>::UndirectedGraph() {
}

template <class T>
UndirectedGraph<T>::UndirectedGraph(const Matrix<T>& m) : _size(m.size()) {
#ifndef NDEBUG
  bool matrix_ok = true;
#endif
  _edges.reserve(_size * _size);
  _adjacency_list.reserve(_size);
  for (Index i = 0; i < _size; ++i) {
    _adjacency_list[i].reserve(_size);
  }
  for (Index i = 0; i < _size; ++i) {
#ifndef NDEBUG
    matrix_ok = matrix_ok && (m[i][i] == INFINITE_COST);
#endif
    for (Index j = i + 1; j < _size; ++j) {
#ifndef NDEBUG
      matrix_ok = matrix_ok && (m[i][j] == m[j][i]);
#endif
      _edges.emplace_back(i, j, m[i][j]);
      _adjacency_list[i].push_back(j);
      _adjacency_list[j].push_back(i);
    }
  }
  assert(matrix_ok);
}

template <class T>
UndirectedGraph<T>::UndirectedGraph(std::vector<Edge<T>> edges)
  : _edges{std::move(edges)} {
  for (auto const& edge : _edges) {
    Index first = edge.get_first_vertex();
    Index second = edge.get_second_vertex();

    _adjacency_list[first].push_back(second);
    _adjacency_list[second].push_back(first);
  }
  _size = _adjacency_list.size();
}

template <class T> std::size_t UndirectedGraph<T>::size() const {
  return _size;
}

template <class T> std::vector<Edge<T>> UndirectedGraph<T>::get_edges() const {
  return _edges;
}

template <class T>
std::unordered_map<Index, std::list<Index>>
UndirectedGraph<T>::get_adjacency_list() const {
  std::unordered_map<Index, std::list<Index>> result;
  for (const auto& pair : _adjacency_list) {
    std::copy(pair.second.begin(),
              pair.second.end(),
              std::back_inserter(result[pair.first]));
  }
  return result;
}

template class UndirectedGraph<Cost>;

} // namespace utils
} // namespace vroom
