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

/*! \file Algorithms.hpp
    \brief Core algorithms used to scribe polytopes.
*/

#ifndef ALGORITHMS_HPP
#define ALGORITHMS_HPP

#include <array>
#include <set>

using namespace std;

#include "Error.hpp"
#include "AuxiliaryTypes.hpp"
#include "AuxiliaryFuncs.hpp"

/*! \fn Error_t computeResidue(T a, T mod, T &residue)
Compute modulo residue a(mod), where mod > 0 
*/
template<class T>
Error_t computeResidue(T a, T mod, T &residue);

/*! \fn Error_t computeReducedResidue(T a, T mod, T &reducedResidue)
Compute modulo reduced residue a(mod), where mod > 0, so that is in (-mod/2; mod/2] 
*/
template<class T>
Error_t computeReducedResidue(T a, T mod, T &reducedResidue);

/*! \fn Error_t gmt2D(T alpha, T gamma, T delta, set<array<T, 2>> &extremePoints)
Compute all extreme points for Polytope of two-dimensional GMT (group minimization task) defined by
t1 + alpha * t2 = gamma (mod delta), where t1, t2 non-negative integers.
*/
template<class T>
Error_t gmt2D(T alpha, T gamma, T delta, set<array<T, 2>> &extremePoints);

template<class T>
Error_t solveCongruent (T alpha, T gamma, T delta, T &y);

#endif // ALGORITHMS_HPP
