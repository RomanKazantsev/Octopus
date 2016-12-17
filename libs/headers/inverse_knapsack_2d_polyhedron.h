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
* \brief Two-dimensional Inverse Knapsack Polyhedron Solver.
*/
#ifndef LIBS_HEADERS_INVERSE_KNAPSACK_2D_POLYHEDRON_H_
#define LIBS_HEADERS_INVERSE_KNAPSACK_2D_POLYHEDRON_H_

#include <array>
#include <set>

#include "libs/headers/knapsack_2d_polytop.h"

/*!
Class for two dimension inverse knapsack polytop.
*/
template <class T>
class InverseKnapsack2dPolyhedron : public Knapsack2dPolytope<T> {
 public:
  /// constructor
  InverseKnapsack2dPolyhedron(T a, T b, T c);
  /// copy constructor
  InverseKnapsack2dPolyhedron(InverseKnapsack2dPolyhedron<T> const& other);
  /// find extreme points at polyhedrone
  void Solve(
      OctopusAlgorithmType
          alg_type = OctopusAlgorithmType::kOctopusIterativeAlg) override;
  /// write information about polyhedron to file
  void Write(std::ostream& s) override;
  /// assignment operator
  InverseKnapsack2dPolyhedron<T>& operator=(
      InverseKnapsack2dPolyhedron<T> const& other);
  /// destructor
  virtual ~InverseKnapsack2dPolyhedron() {}

 private:
  /// iterative algorithm
  void RunIterativeAlgorithm();
};

#endif  // LIBS_HEADERS_INVERSE_KNAPSACK_2D_POLYHEDRON_H_
