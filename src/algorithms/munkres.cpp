/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>
#include <limits>
#include <list>
#include <set>

#include "algorithms/munkres.h"
#include "structures/generic/edge.h"

namespace vroom {
namespace utils {

template <class T>
std::unordered_map<Index, Index>
minimum_weight_perfect_matching(const Matrix<T>& m) {

  // Trivial initial labeling.
  std::vector<T> labeling_x(m.size(), 0);
  std::vector<T> labeling_y(m.size(), 0);
  for (Index i = 0; i < m.size(); ++i) {
    T min_weight = std::numeric_limits<T>::max();
    for (Index j = 0; j < m.size(); ++j) {
      if (m[i][j] < min_weight) {
        min_weight = m[i][j];
      }
    }
    labeling_x[i] = min_weight;
  }

  // Initial empty matching.
  std::unordered_map<Index, Index> matching_xy;
  std::unordered_map<Index, Index> matching_yx;

  // Alternating tree.
  std::unordered_map<Index, Index> alternating_tree;

  while (matching_xy.size() < m.size()) {
    // Step 1.

    alternating_tree.clear();
    std::vector<Index> S_list;
    std::unordered_set<Index> S;
    std::unordered_set<Index> T_set;

    // Finding any unmatched x.
    Index unmatched_x = 0;
    while (matching_xy.find(unmatched_x) != matching_xy.end()) {
      ++unmatched_x;
    }
    S.insert(unmatched_x);
    S_list.push_back(unmatched_x);

    // Saving relevant neighbors in equality graph in alternating_tree
    // and initializing slacks.
    std::vector<T> slack;
    slack.resize(m.size());
    for (Index y = 0; y < m.size(); ++y) {
      if (labeling_x[unmatched_x] + labeling_y[y] == m[unmatched_x][y]) {
        alternating_tree.emplace(y, unmatched_x);
      }
      slack[y] = m[unmatched_x][y] - labeling_x[unmatched_x] - labeling_y[y];
    }

    bool augmented_path = false;

    while (!augmented_path) {
      // Test if neighbors of S in equality graph equals T_set or not
      // (note that T_set is included in S neighbors).
      if (alternating_tree.size() == T_set.size()) {
        // Step 2.

        T alpha = std::numeric_limits<T>::max();
        for (Index y = 0; y < m.size(); ++y) {
          // Computing alpha, the minimum of slack values over
          // complement of T_set.
          if (T_set.find(y) == T_set.end()) {
            T current_slack = slack[y];
            if (current_slack < alpha) {
              alpha = current_slack;
            }
          }
        }

        // Update labelings
        for (auto const& x : S_list) {
          labeling_x[x] = labeling_x[x] + alpha;
        }
        for (auto const& y : T_set) {
          labeling_y[y] = labeling_y[y] - alpha;
        }

        // Updating relevant neighbors in new equality graph and
        // updating slacks.
        for (Index y = 0; y < m.size(); ++y) {
          if (T_set.find(y) == T_set.end()) {
            slack[y] = slack[y] - alpha;

            if (alternating_tree.find(y) == alternating_tree.end()) {
              for (auto const& x : S_list) {
                if (labeling_x[x] + labeling_y[y] == m[x][y]) {
                  alternating_tree.emplace(y, x);
                  break;
                }
              }
            }
          }
        }
      }

      // Step 3.

      // First y in equality neighbors not in T_set.
      Index chosen_y;
      for (auto const& edge : alternating_tree) {
        if (T_set.find(edge.first) == T_set.end()) {
          // MUST happen before endge reaches the end of
          // alternating_tree.
          chosen_y = edge.first;
          break;
        }
      }

      auto matching_y = matching_yx.find(chosen_y);
      if (matching_y != matching_yx.end()) {
        // Chosen y is actually matched in M, update S and T_set and
        // proceed to step 2.
        Index matched_x = matching_y->second;

        auto p = S.insert(matched_x);
        if (p.second) {
          S_list.push_back(matched_x);
        }
        T_set.insert(chosen_y);

        // Updating slacks.
        for (Index y = 0; y < m.size(); ++y) {
          T current_value = slack[y];
          T new_value = m[matched_x][y] - labeling_x[matched_x] - labeling_y[y];
          if (new_value < current_value) {
            slack[y] = new_value;
          }
        }
      } else {
        // Find larger matching using M-alternating path. The path is
        // described at each step by:
        //
        // chosen_y -- chosen_x -- next_y -- [...] -- unmatched_x
        //
        // where (chosen_x, next_y) is already in matching and should
        // be removed and (chosen_x, chosen_y) is to be added.

        Index current_y = chosen_y;
        Index current_x = alternating_tree.at(current_y);

        while (current_x != unmatched_x) {
          Index next_y = matching_xy.at(current_x);

          // Remove alternating edge from current matching.
          matching_xy.erase(matching_xy.find(current_x));
          matching_yx.erase(matching_yx.find(next_y));

          // Add edge from alternating tree to matching.
          matching_xy.emplace(current_x, current_y);
          matching_yx.emplace(current_y, current_x);

          current_y = next_y;
          current_x = alternating_tree.at(current_y);
        }
        // Adding last edge from alternating tree.
        matching_xy.emplace(current_x, current_y);
        matching_yx.emplace(current_y, current_x);

        // Back to step 1.
        augmented_path = true;
      }
    }
  }
  return matching_xy;
}

template <class T>
std::unordered_map<Index, Index>
greedy_symmetric_approx_mwpm(const Matrix<T>& m) {
  // Fast greedy algorithm for finding a symmetric perfect matching,
  // choosing always smaller possible value, no minimality
  // assured. Matrix size should be even!
  assert(m.size() % 2 == 0);

  std::unordered_map<Index, Index> matching;
  std::set<Index> remaining_indices;
  for (Index i = 0; i < m.size(); ++i) {
    remaining_indices.insert(i);
  }

  while (remaining_indices.size() > 0) {
    T min_weight = std::numeric_limits<T>::max();
    std::set<Index>::iterator chosen_i;
    std::set<Index>::iterator chosen_j;
    for (auto i = remaining_indices.begin(); i != remaining_indices.end();
         ++i) {
      auto j = i;
      ++j;
      for (; j != remaining_indices.end(); ++j) {
        T current_weight = m[*i][*j];
        if (current_weight < min_weight) {
          min_weight = current_weight;
          chosen_i = i;
          chosen_j = j;
        }
      }
    }
    matching.emplace(*chosen_i, *chosen_j);
    remaining_indices.erase(chosen_j);
    remaining_indices.erase(chosen_i);
  }

  return matching;
}

template std::unordered_map<Index, Index>
minimum_weight_perfect_matching(const Matrix<Cost>& m);

template std::unordered_map<Index, Index>
greedy_symmetric_approx_mwpm(const Matrix<Cost>& m);

} // namespace utils
} // namespace vroom
