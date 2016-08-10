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

#ifndef LIBS_HEADERS_ALGORITHMS_H_
#define LIBS_HEADERS_ALGORITHMS_H_

/*! \file algorithm.h
\brief Core algorithms used to describe polytopes.
*/

#include <array>
#include <set>

/// Compute a remainder (residue) after division one number by another
template <class T>
void ComputeResidue(T const &a, T const &b, T* res_ptr);

/// Compute reduced remainder (residue) after division one number by another
template <class T>
void ComputeReducedResidue(T const &a, T const &b, T* reduced_res_ptr);

/// Compute Greatest Common Divisor (GCD) of two numbers
template <class T>
void ComputeGcd(T const &a, T const &b, T* gcd_ptr);

/// Compute Greatest Common Divisor of two numbers and Bezout coefficients
template <class T>
void ComputeGcdAndBezoutCoeffs(T const &a, T const &b, T* t1_ptr, T* t2_ptr, T* gcd_ptr);

/// Find extreme points of two dimension Group Minimization Task (GMT) polytop
template <class T>
void FindExtremePointsAt2dGmtPolytop(T const &alpha, T const &gamma, T const &delta,
	std::set<std::array<T, 2>>* extreme_points_ptr);

/// Solve linear congruence equation a * x = b (mod d)
template <class T>
void SolveLinearCongruenceEquation(T const &a, T const &b, T const &d, T* x_ptr);

#endif  // LIBS_HEADERS_ALGORITHMS_H_
