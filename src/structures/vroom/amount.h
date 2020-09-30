#ifndef AMOUNT_H
#define AMOUNT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>
#include <vector>

#include "structures/typedefs.h"

namespace vroom {

template <typename E> class AmountExpression {
public:
  Capacity operator[](size_t i) const {
    return static_cast<const E&>(*this)[i];
  };
  std::size_t size() const {
    return static_cast<const E&>(*this).size();
  };
  bool empty() const {
    return size() == 0;
  };
};

// Lexicographical comparison, useful for situations where a total
// order is required.
template <typename E1, typename E2>
bool operator<<(const AmountExpression<E1>& lhs,
                const AmountExpression<E2>& rhs) {
  assert(lhs.size() == rhs.size());
  if (lhs.empty()) {
    return false;
  }
  auto last_rank = lhs.size() - 1;
  for (std::size_t i = 0; i < last_rank; ++i) {
    if (lhs[i] < rhs[i]) {
      return true;
    }
    if (lhs[i] > rhs[i]) {
      return false;
    }
  }
  return lhs[last_rank] < rhs[last_rank];
}

template <typename E1, typename E2>
bool operator<=(const AmountExpression<E1>& lhs,
                const AmountExpression<E2>& rhs) {
  bool is_inf = true;
  assert(lhs.size() == rhs.size());
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i] > rhs[i]) {
      is_inf = false;
      break;
    }
  }

  return is_inf;
}

template <typename E1, typename E2>
bool operator==(const AmountExpression<E1>& lhs,
                const AmountExpression<E2>& rhs) {
  bool is_equal = true;
  assert(lhs.size() == rhs.size());
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i] != rhs[i]) {
      is_equal = false;
      break;
    }
  }

  return is_equal;
}

class Amount : public AmountExpression<Amount> {

  std::vector<Capacity> elems;

public:
  Amount() = default;

  Amount(std::size_t size) : elems(size, 0){};

  template <typename E> Amount(const AmountExpression<E>& u) {
    elems.resize(u.size());
    for (std::size_t i = 0; i < u.size(); ++i) {
      elems[i] = u[i];
    }
  }

  void push_back(Capacity c) {
    elems.push_back(c);
  };

  Capacity operator[](std::size_t i) const {
    return elems[i];
  };

  Capacity& operator[](std::size_t i) {
    return elems[i];
  };

  std::size_t size() const {
    return elems.size();
  };

  Amount& operator+=(const Amount& rhs) {
    assert(this->size() == rhs.size());
    for (std::size_t i = 0; i < this->size(); ++i) {
      (*this)[i] += rhs[i];
    }
    return *this;
  }

  Amount& operator-=(const Amount& rhs) {
    assert(this->size() == rhs.size());
    for (std::size_t i = 0; i < this->size(); ++i) {
      (*this)[i] -= rhs[i];
    }
    return *this;
  }
};

template <typename E1, typename E2>
class AmountSum : public AmountExpression<AmountSum<E1, E2>> {
  const E1& lhs;
  const E2& rhs;

public:
  AmountSum(const E1& a, const E2& b) : lhs(a), rhs(b) {
    assert(a.size() == b.size());
  };

  Capacity operator[](std::size_t i) const {
    return lhs[i] + rhs[i];
  };

  std::size_t size() const {
    return lhs.size();
  };
};

template <typename E1, typename E2>
auto operator+(const AmountExpression<E1>& u, const AmountExpression<E2>& v)
  -> AmountSum<AmountExpression<E1>, AmountExpression<E2>> {
  return {u, v};
}

template <typename E1, typename E2>
class AmountDiff : public AmountExpression<AmountDiff<E1, E2>> {
  const E1& lhs;
  const E2& rhs;

public:
  AmountDiff(const E1& a, const E2& b) : lhs(a), rhs(b) {
    assert(a.size() == b.size());
  };

  Capacity operator[](std::size_t i) const {
    return lhs[i] - rhs[i];
  };

  std::size_t size() const {
    return lhs.size();
  };
};

template <typename E1, typename E2>
auto operator-(const AmountExpression<E1>& lhs, const AmountExpression<E2>& rhs)
  -> AmountDiff<AmountExpression<E1>, AmountExpression<E2>> {
  return {lhs, rhs};
}

} // namespace vroom

#endif
