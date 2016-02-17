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

#ifndef FROBENIUS_HPP
#define FROBENIUS_HPP

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
class Frobenius{
public:
	Frobenius (T a, T b, T c);
	Frobenius (const Frobenius<T> & frobenius);
	~Frobenius ();
	Error_t reset ();
	Error_t solve();
	Error_t getElapsedTime (double &elapsedTime) const;
	Error_t getElapsedCpuClocks (UINT8 &elapsedCpuClocks) const;
	Error_t getFrobenius(T &frobenius) const;
	Status_t getStatus () const;
	Error_t getInfo () const;
	Error_t getInitializationError () const;
	T getParamA() const;
	T getParamB() const;
	T getParamC() const;
private:
	Error_t calcNonZeroLaticeExtremePoints (T alpha, T delta, set<array<T, 2>> &optimalPoints);
	Error_t IterativeAlgorithm();
	Error_t findMinimumLatticeVector(set<array<T, 2>> extremePoints, array<T, 2> f, array<T, 2> &q0);
	Error_t findLatticeBasisPairForQ0(array<T, 2> q0, array<T, 2> &y);
	Error_t findQ1Q2LatticeVectors(array<T, 2> q0, array<T, 2> y, array<T, 2> &q1, array<T, 2> &q2 );
	Error_t finalPhase(array<T, 2> q0, array<T, 2> q1, array<T, 2> q2);
private:
	T a_, b_, c_;
	T frobenius_;
	Status_t status_;
	double elapsedTime_;
	UINT8 elapsedCpuClocks_;
	Error_t initializationError_;
};

#endif FROBENIUS_HPP // FROBENIUS_HPP
