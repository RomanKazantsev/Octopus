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

/*! \file Knapsack2D.hpp
* Instances of the classes Knapsack2DPolytope and Knapsack2D execute the following tasks:
* 1) to find all extreme points of Polytope 
* Conv{(x1, x2): a*x1 + b*x2 <= c, where a, b, c > 0 and x1, x2 must be non-negative integers}.
* The task is solved for polynomial time based on group minimization task
* 2) to solve two-dimensional Knapsack Problem
* max ( f1 * x1 + f2 * x2), for a*x1 + b*x2 <= c, where a,b,c > 0 and x1,x2 must be non-negative integers.
* The task is solved for polynomial time based on results of the previous task.
*/

#ifndef KNAPSACK2D_HPP
#define KNAPSACK2D_HPP

#include <array>
#include <set>
#include <time.h>
#include <iostream>

using namespace std;

#include "Error.hpp"
#include "Algorithms.hpp"
#include "AuxiliaryTypes.hpp"
#include "AuxiliaryFuncs.hpp"

/*! \class Knapsack2DPolytope
* An instance of the class Knapsack2DPolytope executes the following task:
* to find all extreme points of Polytope simplified by
* Conv{(x1, x2): a * x1 + b * x2 <= c, where a,b,c > 0 and x1,x2 must be non-negative integers}.
* The task is solved using polynomial algorithm based on group minimization task.
*/

template<class T>
class Knapsack2DPolytope{
public:
	Knapsack2DPolytope (T a, T b, T c);
	Knapsack2DPolytope (const Knapsack2DPolytope<T> & knapsack2DPolytope);
	~Knapsack2DPolytope ();
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
	Knapsack2DPolytope<T> & operator = (const Knapsack2DPolytope<T> & opKnapsack2DPolytope);
private:
	Error_t IterativeAlgorithm ();
private:
	set<array<T, 2>> extremePoints_;
	T a_, b_, c_;
	Status_t status_;
	double elapsedTime_;
	UINT8 elapsedCpuClocks_;
	Error_t initializationError_;
};

/*! \class Knapsack2D
* An instance of the class Knapsack2D executes the following task:
* to solve two-dimensional Knapsack Problem
* max ( f1 * x1 + f2 * x2), for a*x1 + b*x2 <= c, where a,b,c > 0 and x1,x2 must be non-negative integers.
* The task is solved for polynomial time based on results of the previous task. 
*/

template<class T>
class Knapsack2D{
public:
	Knapsack2D (T a, T b, T c, T f1, T f2);
	Knapsack2D (const Knapsack2D & knapsack2D);
	~Knapsack2D ();
	Error_t reset ();
	Error_t solve (Algorithm_t algorithm);
	Error_t getElapsedTime (double &elapsedTime) const;
	Error_t getElapsedCpuClocks (UINT8 &elapsedCpuClocks) const;
	Status_t getStatus () const;
	Error_t getInitializationError () const;
	Error_t getInfo () const;
	Error_t getOptimalPoints (set<array<T, 2>> &optimalPoints) const;
	Error_t getOptValue (T &optValue) const;
	Error_t getKnapsack2DPolytope (Knapsack2DPolytope<T> & knapsack2DPolytope) const;
	T getParamF1 () const;
	T getParamF2 () const;
private:
	Knapsack2DPolytope<T> *ptrKnapsack2DPolytope_;
	set<array<T, 2>> optimalPoints_;
	T f1_, f2_, optValue_;
	Status_t status_;
	double elapsedTime_;
	UINT8 elapsedCpuClocks_;
	Error_t initializationError_;
};

#endif // KNAPSACK2D_HPP
