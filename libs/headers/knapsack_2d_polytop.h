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

/*!
* \file
* \brief Two-dimensional Knapsack Polytop Solver.
*/
#ifndef LIBS_HEADERS_KNAPSACK_2D_POLYTOP_H_
#define LIBS_HEADERS_KNAPSACK_2D_POLYTOP_H_

#include <iostream>
#include <array>
#include <set>

#include "libs/headers/task.h"

/*!
Class for two dimension knapsack polytop.
*/
template <class T>
class Knapsack2dPolytope : public Task {
 public:
  /// constructor
  Knapsack2dPolytope(T a, T b, T c);
  /// copy constructor
  Knapsack2dPolytope(Knapsack2dPolytope<T> const& other);
  /// get computed extreme points of the polytop
  void GetExtremePoints(std::set<std::array<T, 2>>* extreme_points_ptr) const;
  /// get polytop parameters
  void GetParams(T* a_ptr, T* b_ptr, T* c_ptr) const;
  /// reset problem status
  virtual void Reset();
  /// find extreme points at the polytop
  virtual void Solve(
      OctopusAlgorithmType alg_type = Task::kOctopusIterativeAlg);
  /// write information about polytop to a file
  virtual void Write(std::ostream& s);
  /// assignment operator
  Knapsack2dPolytope<T>& operator=(Knapsack2dPolytope<T> const& other);
  /// destructor
  virtual ~Knapsack2dPolytope();

 private:
  /// iterative algorithm
  void RunIterativeAlgorithm();

 private:
  T a_, b_, c_;
  std::set<std::array<T, 2>> extreme_points_;
};

#endif  // LIBS_HEADERS_KNAPSACK_2D_POLYTOP_H_
