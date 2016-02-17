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

#include "InverseKnapsack2D.hpp"

template<class T>
InverseKnapsack2DPolyhedrone<T>::InverseKnapsack2DPolyhedrone(T a, T b, T c)
{
    status_ = UNSOLVED;

	if(a <= 0 || b <= 0 || c <= 0) {
		initializationError_ = INVALIDPOLYTOPE;
		return;
	}

	a_ = a;
	b_ = b;
	c_ = c;

	initializationError_ = SUCCESSFUL;
	elapsedTime_ = 0.0f;
	elapsedCpuClocks_ = 0;

}

template<class T>
InverseKnapsack2DPolyhedrone<T>::InverseKnapsack2DPolyhedrone(const InverseKnapsack2DPolyhedrone & inverseKnapsack2DPolyhedrone)
{
	a_ = inverseKnapsack2DPolyhedrone.getParamA();
	b_ = inverseKnapsack2DPolyhedrone.getParamB();
	c_ = inverseKnapsack2DPolyhedrone.getParamC();

	initializationError_ = inverseKnapsack2DPolyhedrone.getInitializationError();
	status_ = inverseKnapsack2DPolyhedrone.getStatus();

	if (status_ == SOLVED) {
		inverseKnapsack2DPolyhedrone.getElapsedTime(elapsedTime_);
		inverseKnapsack2DPolyhedrone.getElapsedCpuClocks(elapsedCpuClocks_);
		inverseKnapsack2DPolyhedrone.getExtremePoints(extremePoints_);
	}
}

template<class T>
InverseKnapsack2DPolyhedrone<T>::~InverseKnapsack2DPolyhedrone(){}

template<class T>
Error_t InverseKnapsack2DPolyhedrone<T>::reset()
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	extremePoints_.clear();
	elapsedTime_ = 0.0f;
	elapsedCpuClocks_ = 0;
	status_ = UNSOLVED;
	return SUCCESSFUL;
}

template<class T>
Error_t InverseKnapsack2DPolyhedrone<T>::getInitializationError() const { return initializationError_; }

template<class T>
Status_t InverseKnapsack2DPolyhedrone<T>::getStatus() const { return status_; }

template<class T>
Error_t InverseKnapsack2DPolyhedrone<T>::getElapsedTime(double &elapsedTime) const
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ != SOLVED)
		return UNSCRIBEDPOLYTOPE;

	elapsedTime = elapsedTime_;
	return SUCCESSFUL;
}

template<class T>
Error_t InverseKnapsack2DPolyhedrone<T>::getElapsedCpuClocks (UINT8 &elapsedCpuClocks) const
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ != SOLVED)
		return UNSCRIBEDPOLYTOPE;

	elapsedCpuClocks = elapsedCpuClocks_;
	return SUCCESSFUL;
}

template<class T>
Error_t InverseKnapsack2DPolyhedrone<T>::getExtremePoints( set<array<T, 2>> &extremePoints ) const
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ != SOLVED)
		return UNSCRIBEDPOLYTOPE;

	extremePoints = extremePoints_;

	return SUCCESSFUL;
}

template<class T>
T InverseKnapsack2DPolyhedrone<T>::getParamA() const { return a_; }

template<class T>
T InverseKnapsack2DPolyhedrone<T>::getParamB() const { return b_; }

template<class T>
T InverseKnapsack2DPolyhedrone<T>::getParamC() const { return c_; }

template<class T>
Error_t InverseKnapsack2DPolyhedrone<T>::scribe(Algorithm_t algorithm)
{
	double start, finish;
	UINT8 clock1, clock2;
	Error_t retVal;

	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ == SOLVED)
		return SCRIBEDPOLYTOPE;

	if(algorithm == ITERATIVE) {
		start = clock();
		clock1 = rdtsc();

		retVal = IterativeAlgorithm();

		clock2 = rdtsc();
		finish = clock();
	}
	else {
		return INCORRECTARG;
	}

	if(retVal != SUCCESSFUL)
		return retVal;

	elapsedTime_ = (double)(finish - start) / CLOCKS_PER_SEC;
	elapsedCpuClocks_ = clock2 - clock1;

	status_ = SOLVED;
	
	return SUCCESSFUL;
}

template<class T>
Error_t InverseKnapsack2DPolyhedrone<T>::IterativeAlgorithm()
{
	array<T, 2> tmpPoint;
	set<array<T, 2>> gmt2DsetExtremePoints1, gmt2DsetExtremePoints2;
	Error_t retVal;

	//x0 - a*x1 = -c(mod b)
	retVal = gmt2D( -a_, -c_, b_, gmt2DsetExtremePoints1 );
	if ( retVal != SUCCESSFUL )
		return retVal;

	for(set<array<T, 2>>::const_iterator it = gmt2DsetExtremePoints1.begin(); it != gmt2DsetExtremePoints1.end(); it++) {
			tmpPoint[0] = (*it)[1];
			tmpPoint[1] = ( c_ - a_ * tmpPoint[0] + (*it)[0] ) / b_;
			if( tmpPoint[1] >= 0) {
				extremePoints_.insert(tmpPoint);
			}
	}

	//x0 - b * x2 = -c(mod a)
	retVal = gmt2D( -b_, -c_, a_, gmt2DsetExtremePoints2 );
	if ( retVal != SUCCESSFUL )
		return retVal;

	for(set<array<T, 2>>::const_iterator it = gmt2DsetExtremePoints2.begin(); it != gmt2DsetExtremePoints2.end(); it++) {
			tmpPoint[1] = (*it)[1];
			tmpPoint[0] = ( c_ - b_ * tmpPoint[1] + (*it)[0] ) / a_;
			if( tmpPoint[0] >= 0) {
				extremePoints_.insert(tmpPoint);
			}
	}

	CleanNonExtremePoints();

	return SUCCESSFUL;
}

template<class T>
Error_t InverseKnapsack2DPolyhedrone<T>::CleanNonExtremePoints()
{
	set<array<T, 2>> newExtremePoints_;
	
	T **extPoint;
	unsigned int ind = 0;

	array<T, 2> tmpPoint;

	if ( extremePoints_.size() == 0 ) {
		return SUCCESSFUL;
	}

	extPoint = new T *[2];

	extPoint[0] = new T[extremePoints_.size()];
	extPoint[1] = new T[extremePoints_.size()];

	for(set<array<T, 2>>::const_iterator it = extremePoints_.begin(); it != extremePoints_.end(); it++) {
		extPoint[0][ind] = (*it)[0];
		extPoint[1][ind] = (*it)[1];
		ind++;
	}

	for(ind = 0; ind < extremePoints_.size(); ind++) {
		if ( ind == 0 || ind == extremePoints_.size() - 1 ) {
			tmpPoint[0] = extPoint[0][ind];
			tmpPoint[1] = extPoint[1][ind];
			newExtremePoints_.insert(tmpPoint);
		}
		else if ( ( extPoint[0][ind] - extPoint[0][ind - 1] ) * ( extPoint[1][ind + 1] - extPoint[1][ind] ) !=
			( extPoint[1][ind] - extPoint[1][ind - 1] ) * ( extPoint[0][ind + 1] - extPoint[0][ind] ) ) {
				tmpPoint[0] = extPoint[0][ind];
				tmpPoint[1] = extPoint[1][ind];
				newExtremePoints_.insert( tmpPoint);
		}
	}

	extremePoints_ = newExtremePoints_;

	delete extPoint[0];
	delete extPoint[1];
	delete extPoint;

	return SUCCESSFUL;
}

template<class T>
Error_t InverseKnapsack2DPolyhedrone<T>::getInfo() const
{
	if ( initializationError_ != SUCCESSFUL ) {
		cout << "Inverse Knapsack2D Polyhedrone is invalid" << endl;
		return SUCCESSFUL;
	}

	cout << "The Inverse Knapsack2D Polyhedrone:" << endl;
	cout << "Conv { (x1, x2) :" << a_ << " * x1 + " << b_ << " * x2 >= " << c_ << endl;
	cout << "x1,x2 are non-negative integer }" << endl;

	if ( status_ == UNSOLVED ) {
		cout << "The polyhedrone is unscribed. Please scribe it" << endl;
		return SUCCESSFUL;
	} else if ( status_ == SOLVED ) {
		cout << "Decision:" << endl;
		for(set<array<T,2>>::const_iterator it = extremePoints_.begin(); it != extremePoints_.end(); it++)
			cout << (*it)[0] << "\t" << (*it)[1] << endl;

		cout << "The number of edge points = " << extremePoints_.size() << endl;
		cout << "The time spent for scribing the polyhedrone = " << elapsedTime_ << endl;
		cout << "CPU clocks consumed for scribing the polyhedrone = " << elapsedCpuClocks_ << endl;
	}

	return SUCCESSFUL;
}

template<class T>
InverseKnapsack2DPolyhedrone<T>& InverseKnapsack2DPolyhedrone<T>::operator = (const InverseKnapsack2DPolyhedrone<T> & opInverseKnapsack2DPolyhedrone)
{
	this->a_ = opInverseKnapsack2DPolyhedrone.getParamA();
	this->b_ = opInverseKnapsack2DPolyhedrone.getParamB();
	this->c_ = opInverseKnapsack2DPolyhedrone.getParamC();

	this->initializationError_ = opInverseKnapsack2DPolyhedrone.getInitializationError();
	this->status_ = opInverseKnapsack2DPolyhedrone.getStatus();

	if( this->status_ == SOLVED) {
		opInverseKnapsack2DPolyhedrone.getExtremePoints(this->extremePoints_);
		opInverseKnapsack2DPolyhedrone.getElapsedTime(this->elapsedTime_);
		opInverseKnapsack2DPolyhedrone.getElapsedCpuClocks(this->elapsedCpuClocks_);
	}

	return *this;
}

template class InverseKnapsack2DPolyhedrone<INT4>; //explicit inst. of INT4 type knapsack problem
template class InverseKnapsack2DPolyhedrone<INT8>; //explicit inst. of INT8 type knapsack problem
