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

// To parameter describe a polytope (to find all edge points)
// a*x1 + b*x2 >= c, where a,b,c > 0 and x1 x2 must be positive integer

#ifndef INVERSEKNAPSACK2D_HPP
#define INVERSEKNAPSACK2D_HPP

#include <array>
#include <set>
#include <time.h>
#include <iostream>

using namespace std;

#include "Error.hpp"
#include "Algorithms.hpp"
#include "AuxiliaryTypes.hpp"
#include "AuxiliaryFuncs.hpp"

template<class T>
class InverseKnapsack2DPolyhedrone{
public:
	InverseKnapsack2DPolyhedrone (T a, T b, T c);
	InverseKnapsack2DPolyhedrone (const InverseKnapsack2DPolyhedrone<T> & inverseKnapsack2DPolyhedrone);
	~InverseKnapsack2DPolyhedrone ();
	Error_t reset ();
	Error_t scribe (Algorithm_t algorithm);
	Error_t getElapsedTime (double &elapsedTime) const;
	Error_t getElapsedCpuClocks (UINT8 &elapsedCpuClocks) const;
	Status_t getStatus () const;
	Error_t getInitializationError () const;
	Error_t getInfo () const;
	Error_t getExtremePoints (set<array<T, 2>> &extremePoints) const;
	T getParamA() const;
	T getParamB() const;
	T getParamC() const;
	InverseKnapsack2DPolyhedrone<T> & operator = (const InverseKnapsack2DPolyhedrone<T> & opInverseKnapsack2DPolyhedrone);
private:
	Error_t IterativeAlgorithm ();
	Error_t CleanNonExtremePoints();
private:
	set<array<T, 2>> extremePoints_;
	T a_, b_, c_;
	Status_t status_;
	double elapsedTime_;
	UINT8 elapsedCpuClocks_;
	Error_t initializationError_;
};

#endif // INVERSEKNAPSACK2D_HPP
