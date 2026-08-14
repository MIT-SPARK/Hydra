#pragma once
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>

namespace progress {

template <typename T>
struct ProgressBar {
  struct Iter {
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = typename T::value_type;
    using pointer = value_type*;
    using reference = value_type&;

    Iter(const ProgressBar<T>* bar, typename T::const_iterator start);
    const value_type& operator*() const;
    pointer operator->();
    Iter operator++();
    Iter operator++(int);
    bool operator==(const Iter& other) const;
    bool operator!=(const Iter& other) const;

    const ProgressBar<T>* bar;
    typename T::const_iterator curr_ptr;
  };

  ProgressBar(const T& v, const std::string& desc = "", size_t width = 80)
      : vec(v),
        prefix(desc.empty() ? desc : desc + ": "),
        width(width),
        num_updates(-1),
        curr_idx(0) {}

  Iter begin() const { return Iter(this, vec.begin()); }

  Iter end() const { return Iter(this, vec.end()); }

  void update() const {
    ++curr_idx;
    double percent = static_cast<double>(curr_idx) / vec.size();
    int percent_floor = static_cast<int>(std::floor(100 * percent));
    percent_floor = std::min(static_cast<int>(100), percent_floor);
    if (num_updates == percent_floor) {
      return;
    }

    const size_t lhs = static_cast<size_t>(std::round(percent * width));
    const size_t rhs = width - lhs;
    std::string bar = std::string(lhs, '#') + std::string(rhs, ' ');
    std::cout << prefix << "[" << bar << "] " << std::right << std::setw(6)
              << std::setprecision(3) << 100 * percent << "%\r";
    std::cout.flush();
    num_updates = percent_floor;
  }

  ~ProgressBar() { std::cout << std::endl; }

  const T& vec;
  const std::string prefix;
  const size_t width;
  mutable int num_updates;
  mutable size_t curr_idx;
};

template <typename T>
ProgressBar<T>::Iter::Iter(const ProgressBar<T>* bar, typename T::const_iterator start)
    : bar(bar), curr_ptr(start) {}

template <typename T>
const typename ProgressBar<T>::Iter::value_type& ProgressBar<T>::Iter::operator*()
    const {
  return *curr_ptr;
}

template <typename T>
typename ProgressBar<T>::Iter::pointer ProgressBar<T>::Iter::operator->() {
  return curr_ptr;
}

template <typename T>
typename ProgressBar<T>::Iter ProgressBar<T>::Iter::operator++() {
  ++curr_ptr;
  bar->update();
  return *this;
}

template <typename T>
typename ProgressBar<T>::Iter ProgressBar<T>::Iter::operator++(int) {
  auto tmp = *this;
  ++(*this);
  return tmp;
}

template <typename T>
bool ProgressBar<T>::Iter::operator==(const ProgressBar<T>::Iter& other) const {
  return curr_ptr == other.curr_ptr;
}

template <typename T>
bool ProgressBar<T>::Iter::operator!=(const ProgressBar<T>::Iter& other) const {
  return !(*this == other);
}

template <typename T>
ProgressBar<T> wrap(const T& vec, const std::string& desc = "") {
  return ProgressBar<T>(vec, desc);
}

}  // namespace progress
