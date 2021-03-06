/*
Copyright (c) 2016, Roman Kazantsev
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the name of Octopus nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <stdexcept>

#include "libs/headers/types.h"
#include "libs/headers/knapsack_2d_polytop.h"
#include "libs/headers/algorithms.h"

template <class T> Knapsack2dPolytope<T>::Knapsack2dPolytope(T a, T b, T c) {
  if (a <= 0 || b <= 0 || c <= 0) {
    throw std::invalid_argument("Input parameters must be positive.");
  }
  a_ = a;
  b_ = b;
  c_ = c;
  extreme_points_.clear();
}

template <class T>
Knapsack2dPolytope<T>::Knapsack2dPolytope(Knapsack2dPolytope const &other)
    : Task(other), a_(other.a_), b_(other.b_), c_(other.c_),
      extreme_points_(other.extreme_points_) {}

template <class T> Knapsack2dPolytope<T>::~Knapsack2dPolytope() {}

template <class T> void Knapsack2dPolytope<T>::Reset() {
  extreme_points_.clear();
  status_ = Task::OctopusTaskStatus::kOctopusUnsolved;
}

template <class T>
void Knapsack2dPolytope<T>::GetExtremePoints(
    std::set<std::array<T, 2>> *extreme_points_ptr) const {
  if (status_ != Task::OctopusTaskStatus::kOctopusSolved) {
    throw std::logic_error("The task is not solved.");
  }
  *extreme_points_ptr = extreme_points_;
}

template <class T>
void Knapsack2dPolytope<T>::GetParams(T *a_ptr, T *b_ptr, T *c_ptr) const {
  *a_ptr = a_;
  *b_ptr = b_;
  *c_ptr = c_;
}

template <class T>
void Knapsack2dPolytope<T>::Solve(
    OctopusAlgorithmType alg_type =
        OctopusAlgorithmType::kOctopusIterativeAlg) {
  if (status_ == Task::OctopusTaskStatus::kOctopusSolved) {
    throw std::logic_error("The task has been already solved.");
  }
  if (alg_type_ == Task::OctopusAlgorithmType::kOctopusIterativeAlg) {
    RunIterativeAlgorithm();
  } else {
    throw std::invalid_argument("Not supported algorithm type is specified.");
  }
}

template <class T>
Knapsack2dPolytope<T> &Knapsack2dPolytope<T>::
operator=(Knapsack2dPolytope<T> const &other) {
  Task::operator=(other);
  a_ = other.a_;
  b_ = other.b_;
  c_ = other.c_;
  extreme_points_ = other.extreme_points_;

  return *this;
}

template <class T> void Knapsack2dPolytope<T>::RunIterativeAlgorithm() {
  // clear a set of extreme points
  extreme_points_.clear();
  // add zero point to a set of extreme points
  extreme_points_.insert(std::move(std::array<T, 2>{{0, 0}}));

  std::set<std::array<T, 2>> gmt_extreme_points{};

  gmt_extreme_points.clear();
  // x0 + a1*x1 = b(mod a2)
  FindExtremePointsAt2dGmtPolytop(a_, c_, b_, &gmt_extreme_points);

  for (const auto &point : gmt_extreme_points) {
    if (((c_ - a_ * point[1] - point[0]) / b_) >= 0) {
      extreme_points_.insert(std::move<std::array<T, 2>>(
          {{point[1], (c_ - a_ * point[1] - point[0]) / b_}}));
    }
  }

  gmt_extreme_points.clear();
  // x0 + a2 * x2 = b(mod a1)
  FindExtremePointsAt2dGmtPolytop(b_, c_, a_, &gmt_extreme_points);

  for (const auto &point : gmt_extreme_points) {
    if (((c_ - b_ * point[1] - point[0]) / a_) >= 0) {
      extreme_points_.insert(std::move<std::array<T, 2>>(
          {{(c_ - b_ * point[1] - point[0]) / a_, point[1]}}));
    }
  }

  status_ = Task::OctopusTaskStatus::kOctopusSolved;
}

template <class T> void Knapsack2dPolytope<T>::Write(std::ostream &s) {
  s << "The Knapsack2D Polytope:" << std::endl;
  s << "Conv { (x1, x2) :" << a_ << " * x1 + " << b_ << " * x2 <= " << c_
    << std::endl;
  s << "x1,x2 are non-negative integer }" << std::endl;

  if (status_ == OctopusTaskStatus::kOctopusSolved) {
    s << "Decision:" << std::endl;
    for (const auto &point : extreme_points_) {
      s << point[0] << "\t" << point[1] << std::endl;
    }
    s << "The number of edge points = " << extreme_points_.size() << std::endl;
  } else {
    s << "The polytop has not yet been described." << std::endl;
  }

  Task::Write(s);
}

template class Knapsack2dPolytope<OctopusInt4>;
template class Knapsack2dPolytope<OctopusInt8>;
