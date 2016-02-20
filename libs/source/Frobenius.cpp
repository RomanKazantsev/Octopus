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

#include "Frobenius.hpp"

template<class T>
Frobenius<T>::Frobenius(T a, T b, T c)
{
    status_ = UNSOLVED;

	a_ = a;
	b_ = b;
	c_ = c;

	initializationError_ = SUCCESSFUL;
	elapsedTime_ = 0.0f;
	elapsedCpuClocks_ = 0;

}

template<class T>
Frobenius<T>::~Frobenius(){}

template<class T>
Frobenius<T>::Frobenius(const Frobenius & frobenius)
{
	a_ = frobenius.getParamA();
	b_ = frobenius.getParamB();
	c_ = frobenius.getParamC();

	initializationError_ = frobenius.getInitializationError();
	status_ = frobenius.getStatus();

	if (status_ == SOLVED) {
		frobenius.getElapsedTime(elapsedTime_);
		frobenius.getElapsedCpuClocks(elapsedCpuClocks_);
	}

}

template<class T>
Error_t Frobenius<T>::getInitializationError() const { return initializationError_; }

template<class T>
Status_t Frobenius<T>::getStatus() const { return status_; }

template<class T>
Error_t Frobenius<T>::getInfo() const
{	
	if ( initializationError_ != SUCCESSFUL ) {
		cout << "Frobenius task is invalid" << endl;
		return SUCCESSFUL;
	}

	cout << "Frobenius problem is defined for:" << endl;
	cout << "\t" << a_ << "\t" << b_ << "\t" << c_ << endl;

	if ( status_ == UNSOLVED ) {
		cout << "The Frobenius problem is unsolved. Please solve it" << endl;
		return SUCCESSFUL;
	} else if ( status_ == SOLVED ) {
		cout << "Frob = " << frobenius_ <<endl;
		cout << "The time spent for solving = " << elapsedTime_ << endl;
		cout << "CPU clocks consumed for solving = " << elapsedCpuClocks_ << endl;
	}

	return SUCCESSFUL;
}

template<class T>
Error_t Frobenius<T>::reset()
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	elapsedTime_ = 0.0f;
	elapsedCpuClocks_ = 0;
	status_ = UNSOLVED;
	return SUCCESSFUL;
}

template<class T>
Error_t Frobenius<T>::getElapsedTime(double &elapsedTime) const
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ != SOLVED)
		return UNSCRIBEDPOLYTOPE;

	elapsedTime = elapsedTime_;
	return SUCCESSFUL;
}

template<class T>
Error_t Frobenius<T>::getElapsedCpuClocks (UINT8 &elapsedCpuClocks) const
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ != SOLVED)
		return UNSCRIBEDPOLYTOPE;

	elapsedCpuClocks = elapsedCpuClocks_;
	return SUCCESSFUL;
}

template<class T>
Error_t Frobenius<T>::getFrobenius (T &frobenius) const
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ != SOLVED)
		return UNSCRIBEDPOLYTOPE;

	frobenius = frobenius_;
	return SUCCESSFUL;
}

template<class T>
T Frobenius<T>::getParamA() const { return a_; }

template<class T>
T Frobenius<T>::getParamB() const { return b_; }

template<class T>
T Frobenius<T>::getParamC() const { return c_; }


// t1+alpha*t2=0(mod delta), (t1,t2) != 0 and t1, t2 are non-negative integers
template<class T>
Error_t Frobenius<T>::calcNonZeroLaticeExtremePoints (T alpha, T delta, set<array<T, 2>> &optimalPoints)
{
	set<array<T, 2>> tmpExtremePoints;
	array<T, 2> tmpPoint;
	Error_t retVal;

	optimalPoints.clear();

	tmpPoint[0] = delta;
	tmpPoint[1] = 0;

	optimalPoints.insert(tmpPoint);

	if ( alpha == 0 ) {
		tmpPoint[0] = 0;
		tmpPoint[1] = 1;
		optimalPoints.insert(tmpPoint);
		return SUCCESSFUL;
	}

	retVal = gmt2D<T>(-delta, delta, alpha, tmpExtremePoints);

	if ( retVal != SUCCESSFUL )
		return retVal;

	for(set<array<T, 2>>::const_iterator it = tmpExtremePoints.begin(); it != tmpExtremePoints.end(); it++) {
			tmpPoint[0] = (*it)[0];
			tmpPoint[1] = ( delta + delta * (*it)[1] - tmpPoint[0] ) / alpha;
			if( tmpPoint[1] >= 0) {
				optimalPoints.insert(tmpPoint);
			}
	}

	return SUCCESSFUL;
}

template<class T>
Error_t Frobenius<T>::findQ1Q2LatticeVectors(array<T, 2> q0, array<T, 2> y, array<T, 2> &q1, array<T, 2> &q2 )
{
	array<T, 3> Q0, Y;
	T k;

	Q0[0] = b_ * q0[0] + c_ * q0[1];
	Q0[1] = q0[0];
	Q0[2] = q0[1];

	Y[0] = b_ * y[0] + c_ * y[1];
	Y[1] = y[0];
	Y[2] = y[1];

//	cout << " Q0 = " << Q0[0] << "\t" << Q0[1] << "\t" << Q0[2] << endl; 
//	cout << " Y  = " << Y[0] << "\t" << Y[1] << "\t" << Y[2] << endl; 

	if ( Q0[0] < Y[0] ) {
		k = Y[0] / Q0[0];
	} else if ( Q0[0] == Y[0] ) {
		k = 1;
	} else {
		cout << "findQ1Q2: TRACE1" << endl;
		return INCORRECTEXECFLOW;
	}

//	q1 = k * q0 - y;
//	q2 = y - (k + 1) * q0;

	q1[0] = k * q0[0] - y[0];
	q1[1] = k * q0[1] - y[1];

	q2[0] = y[0] - ( k + 1 ) * q0[0];
	q2[1] = y[1] - ( k + 1 ) * q0[1];
	
	return SUCCESSFUL;
}

template<class T>
Error_t Frobenius<T>::finalPhase(array<T, 2> q0, array<T, 2> q1, array<T, 2> q2)
{
	array<T, 2> p1, p2;
	T f1 ,f2;

	if ( q0[0] != 0 && q0[1] != 0 ) { // find p1 and p2
		if ( q1[1] < 0 ) {
			p1[0] = q0[0] - 1;
			p1[1] = -q1[1] - 1;
		} else if (q2[1] < 0) {
			p1[0] = q0[0] - 1;
			p1[1] = -q2[1] - 1;
		} else {
			return INCORRECTEXECFLOW;
		}

		if ( q1[0] < 0 ) {
			p2[1] = q0[1] - 1;
			p2[0] = -q1[0] - 1;
		} else if (q2[0] < 0) {
			p2[1] = q0[1] - 1;
			p2[0] = -q2[0] - 1;
		} else {
			return INCORRECTEXECFLOW;
		}

//		cout << " p1 = " << p1[0] << "\t" << p1[1] << endl;
//		cout << " p2 = " << p2[0] << "\t" << p2[1] << endl;

		f1 = b_ * p1[0] + c_ * p1[1];
		f2 = b_ * p2[0] + c_ * p2[1];

		if ( f1 <= f2 ) {
			frobenius_ = f2 - a_;
		} else {
			frobenius_ = f1 - a_;
		}
	} else if ( q0[0] != 0 ) { // find only p1
		if ( q1[1] < 0 ) {
			p1[0] = q0[0] - 1;
			p1[1] = -q1[1] - 1;
		} else if (q2[1] < 0) {
			p1[0] = q0[0] - 1;
			p1[1] = -q2[1] - 1;
		} else {
			return INCORRECTEXECFLOW;
		}

//		cout << " p1 = " << p1[0] << "\t" << p1[1] << endl;

		f1 = b_ * p1[0] + c_ * p1[1];
		frobenius_ = f1 - a_;
	} else if ( q0[1] != 0 ) { // find only p2
		if ( q1[0] < 0 ) {
			p2[1] = q0[1] - 1;
			p2[0] = -q1[0] - 1;
		} else if (q2[0] < 0) {
			p2[1] = q0[1] - 1;
			p2[0] = -q2[0] - 1;
		} else {
			return INCORRECTEXECFLOW;
		}

//		cout << " p2 = " << p2[0] << "\t" << p2[1] << endl;

		f2 = b_ * p2[0] + c_ * p2[1];
		frobenius_ = f2 - a_;
	} else {
		return INCORRECTEXECFLOW;
	}

//	cout << "Frobenius = " << frobenius_ << endl;

	return SUCCESSFUL;
}


template<class T>
Error_t Frobenius<T>::IterativeAlgorithm()
{
	set<array<T, 2>> tmpExtremePoints, extremePoints;
	T tmpGcd, a, b, c, t1, t2, tmp;
	array<T, 2> tmpPoint, f, q0, y, q1, q2;
	Error_t retVal;

	a = a_;
	b = b_;
	c = c_;

//	cout << "a = " << a << " b = " << b << " c = " << c << endl;

	//b * x1 + c * x2 = (mod a)
	retVal = EuclidAlg<T>(a, b, tmpGcd);
	if ( retVal != SUCCESSFUL ) {
//		cout << "TRACE1" << endl;
		return retVal;
	}

//	cout << " tmpGCD = " << tmpGcd;

	a = a / tmpGcd;
	b = b / tmpGcd;
	//b' * x1 + c * x2' = (mod a'), where a'=a/gdc, b'=b/gcd, x2=x2'*gcd
	
	retVal = ExtendedEuclidAlg (b, a, t1, t2, tmp);
	if ( retVal != SUCCESSFUL ) {
//		cout << "TRACE2" << endl;
		return retVal;
	}

//	cout << " t1 = " << t1 << endl;

//	cout << "compute residue 1" << endl; 

	retVal = computeResidue( b * t1, a, b);
	if ( retVal != SUCCESSFUL ) {
//		cout << "TRACE3" << endl;
		return retVal;
	}

//	cout << "compute residue 2" << endl; 

	retVal = computeResidue ( c * t1, a, c);
	if ( retVal != SUCCESSFUL )
		return retVal;

	if ( b != 1 ) {
		cout << "Incorrect execution flow" << endl;
		return INCORRECTEXECFLOW;
	}

//	cout << "calcNonZeroLaticeExtremePoints x1 + c * x2 = 0 (mod a)" << endl;
//	cout << " a = " << a << " b = " << b << " c = " << c << endl;
	//x1 + c' * x2' = (mod a'), where c' = c * t1 (mod a')
	retVal = calcNonZeroLaticeExtremePoints( c, a, tmpExtremePoints);
	if( retVal != SUCCESSFUL ) {
//		cout << "TRACE4" << endl;
		return retVal;
	}

	for(set<array<T, 2>>::const_iterator it = tmpExtremePoints.begin(); it != tmpExtremePoints.end(); it++) {
		tmpPoint[0] = (*it)[0];
		tmpPoint[1] = (*it)[1] * tmpGcd;
		extremePoints.insert(tmpPoint);
	}

//	cout << "Extreme points:" << endl;
/*
	for(set<array<T, 2>>::const_iterator it = extremePoints.begin(); it != extremePoints.end(); it++) {
			cout << (*it)[0] << "\t" << (*it)[1] << endl;
	}
*/
	f[0] = b_;
	f[1] = c_;

	retVal = findMinimumLatticeVector ( extremePoints, f, q0 );
	if ( retVal != SUCCESSFUL ) {
//		cout << "TRACE5" << endl;
		return retVal;
	}

	cout << "q0 = " << q0[0] << "\t" << q0[1] << endl;

	retVal = findLatticeBasisPairForQ0 ( q0, y );

//	cout << "y = " << y[0] << "\t" << y[1] << endl;

//	cout << " findQ1Q2LatticeVectors " << endl;

	retVal = findQ1Q2LatticeVectors ( q0, y, q1, q2);
	if ( retVal != SUCCESSFUL ) {
//		cout << "TRACE6" << endl;
		return retVal;
	}

//	cout << " q1 = " << q1[0] << "\t" << q1[1] << endl;
//	cout << " q2 = " << q2[0] << "\t" << q2[1] << endl;

	retVal = finalPhase ( q0, q1, q2 );
	if ( retVal != SUCCESSFUL ) {
//		cout << "TRACE7" << endl;
		return retVal;
	}

	return SUCCESSFUL;
}

template<class T>
Error_t Frobenius<T>::solve()
{
	double start, finish;
	UINT8 clock1, clock2;
	Error_t retVal;

	start = clock();
	clock1 = rdtsc();

	retVal = IterativeAlgorithm();

	clock2 = rdtsc();
	finish = clock();

	if ( retVal == SUCCESSFUL ) {
		elapsedTime_ = (double)(finish - start) / CLOCKS_PER_SEC;
		elapsedCpuClocks_ = clock2 - clock1;
		status_ = SOLVED;
	}

	return SUCCESSFUL;
}

template<class T>
Error_t Frobenius<T>::findMinimumLatticeVector(set<array<T, 2>> extremePoints, array<T, 2> f, array<T, 2> &q0)
{
	set<array<T, 2>>::const_iterator it = extremePoints.begin();

	q0[0] = (*it)[0];
	q0[1] = (*it)[1];

	if (extremePoints.size() <= 0 )
		return EMPTYSET;

	for(it = extremePoints.begin(); it != extremePoints.end(); it++) {
		if ( ( (*it)[0] * f[0] + (*it)[1] * f[1] ) < ( q0[0] * f[0] + q0[1] * f[1] ) ) {
			q0[0] = (*it)[0];
			q0[1] = (*it)[1];
		}
		else if ( ( (*it)[0] * f[0] + (*it)[1] * f[1] ) == ( q0[0] * f[0] + q0[1] * f[1] ) ) {
			if ( (*it)[0] < q0[0] ) {
				q0[0] = (*it)[0];
				q0[1] = (*it)[1];
			}
			else if ( (*it)[0] == q0[0] ) {
				if ( (*it)[1] < q0[1] ) {
					q0[0] = (*it)[0];
					q0[1] = (*it)[1];
				}
			}
		}
	}

	return SUCCESSFUL;
}

template<class T>
Error_t Frobenius<T>::findLatticeBasisPairForQ0(array<T, 2> q0, array<T, 2> &y)
{
	T t1, t2, gcd;
	Error_t retVal;
// | q00 * y1 - q01 * y0 | = a

	if ( q0[0] == 0 && q0[1] == 0 )
		return INCORRECTEXECFLOW;

	if ( q0[0] == 0 ) {
		y[0] = a_ / q0[1];

		retVal = solveCongruent<T> (c_, - ( b_ * y[0] ), a_, y[1]);
		if ( retVal != SUCCESSFUL )
			return retVal;

		return SUCCESSFUL;
	}

	if ( q0[1] == 0 ) {
		y[1] = a_ / q0[0];

		retVal = solveCongruent<T> (b_, - ( c_ * y[1] ), a_, y[0]);
		if ( retVal != SUCCESSFUL )
			return retVal;

		return SUCCESSFUL;
	}

	retVal = ExtendedEuclidAlg<T> ( q0[0], q0[1], t1, t2, gcd);

	y[0] = t2 * a_ / gcd;
	y[1] = t1 * a_ / gcd;

	if ( y[0] < 0 )
		y[0] = -y[0];
	if ( y[1] < 0 )
		y[1] = -y[1];

	return SUCCESSFUL;
}

template class Frobenius<INT4>; //explicit inst. of INT4 type knapsack problem
template class Frobenius<INT8>; //explicit inst. of INT8 type knapsack problem
