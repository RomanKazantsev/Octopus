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

#include "Algorithms.hpp"

using namespace std;

/* Compute modulo residue a(mod), where mod > 0 */
template<class T>
Error_t computeResidue(T a, T mod, T &residue)
{
	if( mod <= 0) {
		cout << "incorrect args in compute residue" << endl;
		return INCORRECTARG;
	}

	residue = a % mod;

	if(residue < 0)
		residue = mod + residue;

	return SUCCESSFUL;
}
template Error_t computeResidue(INT4 a, INT4 mod, INT4 &residue);
template Error_t computeResidue(INT8 a, INT8 mod, INT8 &residue);

/* Compute modulo reduced residue a(mod), where mod > 0, so that is in (-mod/2; mod/2] */
template<class T>
Error_t computeReducedResidue(T a, T mod, T &reducedResidue)
{
	Error_t retVal;

	if( mod <= 0 ) {
		cout << "compute reduced residue" << endl;
		return INCORRECTARG;
	}

	retVal = computeResidue(a, mod, reducedResidue);

	if( retVal != SUCCESSFUL )
		return retVal;

	if(reducedResidue > mod / 2)
		reducedResidue = reducedResidue - mod;

	return SUCCESSFUL;
}
template Error_t computeReducedResidue(INT4 a, INT4 b, INT4 &res);
template Error_t computeReducedResidue(INT8 a, INT8 b, INT8 &res);

/* Compute all extreme points for Polytope of two-dimensional GMT (group minimization task) defined by
t1 + alpha * t2 = gamma (mod delta), where t1, t2 non-negative integers */
template<class T>
Error_t gmt2D(T alpha, T gamma, T delta, set<array<T, 2>> &extremePoints)
{
	Error_t retVal;

	array<T, 2> tmpPoint;
	set<array<T, 2>> tmpsetExtremePoints;

	retVal = computeReducedResidue(alpha, delta, alpha);
	if ( retVal != SUCCESSFUL)
		return retVal;

	retVal = computeResidue(gamma, delta, gamma);
	if ( retVal != SUCCESSFUL)
		return retVal;

	if ( alpha == 0) {
		tmpPoint[0] = gamma;
		tmpPoint[1] = 0;
		extremePoints.insert(tmpPoint);
	}
	else if ( alpha == 1 ) {
		tmpPoint[0] = gamma;
		tmpPoint[1] = 0;
		extremePoints.insert(tmpPoint);

		tmpPoint[0] = 0;
		tmpPoint[1] = gamma;
		extremePoints.insert(tmpPoint);
	}
	else if(alpha == -1) {
		tmpPoint[0] = gamma;
		tmpPoint[1] = 0;
		extremePoints.insert(tmpPoint);

		tmpPoint[0] = 0;
		tmpPoint[1] = delta - gamma;
		extremePoints.insert(tmpPoint);
	}
	else if(alpha > 1 && alpha <= delta / 2) {
		retVal = gmt2D(-delta, gamma, alpha, tmpsetExtremePoints);
		if ( retVal != SUCCESSFUL )
			return retVal;

		for(set<array<T, 2>>::const_iterator it = tmpsetExtremePoints.begin(); it != tmpsetExtremePoints.end(); it++) {
			tmpPoint[0] = (*it)[0];
			tmpPoint[1] = (gamma + delta * (*it)[1] - (*it)[0]) / alpha;
			extremePoints.insert(tmpPoint);
		}
		
	}
	else if(alpha < -1 && 2*alpha > -delta) {
		tmpPoint[0] = gamma;
		tmpPoint[1] = 0;
		extremePoints.insert(tmpPoint);

		retVal = gmt2D(delta, gamma - delta, -alpha, tmpsetExtremePoints);
		if ( retVal != SUCCESSFUL)
			return retVal;

		for(set<array<T, 2>>::const_iterator it = tmpsetExtremePoints.begin(); it != tmpsetExtremePoints.end(); it++) {
			tmpPoint[0] = (*it)[0];
			tmpPoint[1] = (gamma - delta - delta * (*it)[1] - (*it)[0]) / alpha;
			extremePoints.insert(tmpPoint);
		}

	}
	else {
		cout << "Incorrect execution flow" << endl;
		return INCORRECTEXECFLOW;
	}

	tmpPoint[0] = gamma;
	tmpPoint[1] = 0;
	extremePoints.insert(tmpPoint);

	return SUCCESSFUL; 
}
template Error_t gmt2D(INT4 alpha, INT4 gamma, INT4 delta, set<array<INT4, 2>> &extremePoints);
template Error_t gmt2D(INT8 alpha, INT8 gamma, INT8 delta, set<array<INT8, 2>> &extremePoints);

/*Solve alpha * y = gamma (mod delta), where y is minimum non-negative integer */
template<class T>
Error_t solveCongruent (T alpha, T gamma, T delta, T &y)
{
	T gcd, t1, t2;
	Error_t retVal;

	if ( delta <= 0 )
		return INCORRECTARG;

	retVal = computeResidue<T> ( alpha, delta, alpha);
	if ( retVal != SUCCESSFUL )
		return retVal;

	retVal = computeResidue<T> ( gamma, delta, gamma);
	if ( retVal != SUCCESSFUL )
		return retVal;

	retVal = ExtendedEuclidAlg<T> ( alpha, delta, t1, t2, gcd);
	if ( retVal != SUCCESSFUL )
		return retVal;

	if (gamma % gcd != 0)
		return NOSOLUTION;

	y = t1 * gamma / gcd;

	retVal = computeResidue ( y, delta, y);
	if ( retVal != SUCCESSFUL )
		return retVal;

	return SUCCESSFUL;
}
template Error_t solveCongruent (INT4 alpha, INT4 gamma, INT4 delta, INT4 &y);
template Error_t solveCongruent (INT8 alpha, INT8 gamma, INT8 delta, INT8 &y);
