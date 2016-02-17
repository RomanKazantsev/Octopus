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

#include "Knapsack2D.hpp"
#include "big_int.hpp"

using namespace Arageli;

template<class T>
Knapsack2DPolytope<T>::Knapsack2DPolytope(T a, T b, T c)
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
Knapsack2DPolytope<T>::Knapsack2DPolytope(const Knapsack2DPolytope & knapsack2DPolytope)
{
	a_ = knapsack2DPolytope.getParamA();
	b_ = knapsack2DPolytope.getParamB();
	c_ = knapsack2DPolytope.getParamC();

	initializationError_ = knapsack2DPolytope.getInitializationError();
	status_ = knapsack2DPolytope.getStatus();

	if (status_ == SOLVED) {
		knapsack2DPolytope.getElapsedTime(elapsedTime_);
		knapsack2DPolytope.getElapsedCpuClocks(elapsedCpuClocks_);
		knapsack2DPolytope.getExtremePoints(extremePoints_);
	}

}

template<class T>
Knapsack2DPolytope<T>::~Knapsack2DPolytope(){}

template<class T>
Error_t Knapsack2DPolytope<T>::reset()
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
Error_t Knapsack2DPolytope<T>::getInitializationError() const { return initializationError_; }

template<class T>
Status_t Knapsack2DPolytope<T>::getStatus() const { return status_; }

template<class T>
Error_t Knapsack2DPolytope<T>::getElapsedTime(double &elapsedTime) const
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ != SOLVED)
		return UNSCRIBEDPOLYTOPE;

	elapsedTime = elapsedTime_;
	return SUCCESSFUL;
}

template<class T>
Error_t Knapsack2DPolytope<T>::getElapsedCpuClocks (UINT8 &elapsedCpuClocks) const
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ != SOLVED)
		return UNSCRIBEDPOLYTOPE;

	elapsedCpuClocks = elapsedCpuClocks_;
	return SUCCESSFUL;
}

template<class T>
Error_t Knapsack2DPolytope<T>::getExtremePoints( set<array<T, 2>> &extremePoints ) const
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ != SOLVED)
		return UNSCRIBEDPOLYTOPE;

	extremePoints = extremePoints_;

	return SUCCESSFUL;
}

template<class T>
T Knapsack2DPolytope<T>::getParamA() const { return a_; }

template<class T>
T Knapsack2DPolytope<T>::getParamB() const { return b_; }

template<class T>
T Knapsack2DPolytope<T>::getParamC() const { return c_; }

template<class T>
Error_t Knapsack2DPolytope<T>::scribe(Algorithm_t algorithm)
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
Error_t Knapsack2DPolytope<T>::IterativeAlgorithm()
{
	array<T, 2> tmpPoint;
	set<array<T, 2>> gmt2DsetExtremePoints1, gmt2DsetExtremePoints2;
	Error_t retVal;

	tmpPoint[0] = 0;
	tmpPoint[1] = 0;

	extremePoints_.insert( tmpPoint );

	//x0 + a1*x1 = b(mod a2)
	retVal = gmt2D( a_, c_, b_, gmt2DsetExtremePoints1 );
	if ( retVal != SUCCESSFUL )
		return retVal;

	for(set<array<T, 2>>::const_iterator it = gmt2DsetExtremePoints1.begin(); it != gmt2DsetExtremePoints1.end(); it++) {
			tmpPoint[0] = (*it)[1];
			tmpPoint[1] = ( c_ - a_ * tmpPoint[0] - (*it)[0] ) / b_;
			if( tmpPoint[1] >= 0) {
				extremePoints_.insert(tmpPoint);
			}
	}

	//x0 + a2 * x2 = b(mod a1)
	retVal = gmt2D( b_, c_, a_, gmt2DsetExtremePoints2 );
	if ( retVal != SUCCESSFUL )
		return retVal;

	for(set<array<T, 2>>::const_iterator it = gmt2DsetExtremePoints2.begin(); it != gmt2DsetExtremePoints2.end(); it++) {
			tmpPoint[1] = (*it)[1];
			tmpPoint[0] = ( c_ - b_ * tmpPoint[1] - (*it)[0] ) / a_;
			if( tmpPoint[0] >= 0) {
				extremePoints_.insert(tmpPoint);
			}
	}

	return SUCCESSFUL;
}

template<class T>
Error_t Knapsack2DPolytope<T>::getInfo() const
{
	if ( initializationError_ != SUCCESSFUL ) {
		cout << "Knapsack2D Polytope is invalid" << endl;
		return SUCCESSFUL;
	}

	cout << "The Knapsack2D Polytope:" << endl;
	cout << "Conv { (x1, x2) :" << a_ << " * x1 + " << b_ << " * x2 <= " << c_ << endl;
	cout << "x1,x2 are non-negative integer }" << endl;

	if ( status_ == UNSOLVED ) {
		cout << "The polytope is unscribed. Please scribe it" << endl;
		return SUCCESSFUL;
	} else if ( status_ == SOLVED ) {
		cout << "Decision:" << endl;
		for(set<array<T,2>>::const_iterator it = extremePoints_.begin(); it != extremePoints_.end(); it++)
			cout << (*it)[0] << "\t" << (*it)[1] << endl;

		cout << "The number of edge points = " << extremePoints_.size() << endl;
		cout << "The time spent for scribing the polytope = " << elapsedTime_ << endl;
		cout << "CPU clocks consumed for scribing the polytope = " << elapsedCpuClocks_ << endl;
	}

	return SUCCESSFUL;
}

template<class T>
Knapsack2DPolytope<T>& Knapsack2DPolytope<T>::operator = (const Knapsack2DPolytope<T> & opKnapsack2DPolytope)
{
	this->a_ = opKnapsack2DPolytope.getParamA();
	this->b_ = opKnapsack2DPolytope.getParamB();
	this->c_ = opKnapsack2DPolytope.getParamC();

	this->initializationError_ = opKnapsack2DPolytope.getInitializationError();
	this->status_ = opKnapsack2DPolytope.getStatus();

	if( this->status_ == SOLVED) {
		opKnapsack2DPolytope.getExtremePoints(this->extremePoints_);
		opKnapsack2DPolytope.getElapsedTime(this->elapsedTime_);
		opKnapsack2DPolytope.getElapsedCpuClocks(this->elapsedCpuClocks_);
	}

	return *this;
}


template class Knapsack2DPolytope<INT4>; //explicit inst. of INT4 type knapsack problem
template class Knapsack2DPolytope<INT8>; //explicit inst. of INT8 type knapsack problem
template class Knapsack2DPolytope<big_int>; //explicit inst. of big_int type knapsack problem


template<class T>
Knapsack2D<T>::Knapsack2D (T a, T b, T c, T f1, T f2)
{
	f1_ = f1;
	f2_ = f2;

	ptrKnapsack2DPolytope_ = new Knapsack2DPolytope<T>(a, b, c);
	initializationError_ = ptrKnapsack2DPolytope_->getInitializationError();

	status_ = UNSOLVED;
	optValue_ = 0;
}

template<class T>
Knapsack2D<T>::Knapsack2D (const Knapsack2D & knapsack2D)
{
	f1_ = knapsack2D.getParamF1();
	f2_ = knapsack2D.getParamF2();
	knapsack2D.getKnapsack2DPolytope(*ptrKnapsack2DPolytope_);
	status_ = knapsack2D.getStatus();
	initializationError_ = knapsack2D.getInitializationError();

	if (status_ == SOLVED) {
		knapsack2D.getOptimalPoints(optimalPoints_);
		knapsack2D.getOptValue (optValue_);

		knapsack2D.getElapsedTime(elapsedTime_);
		knapsack2D.getElapsedCpuClocks(elapsedCpuClocks_);
	}

}

template<class T>
Knapsack2D<T>::~Knapsack2D ()
{
	if (ptrKnapsack2DPolytope_ != NULL)
		delete ptrKnapsack2DPolytope_;
}

template<class T>
Error_t Knapsack2D<T>::reset ()
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	ptrKnapsack2DPolytope_->reset();
	optimalPoints_.clear();

	elapsedTime_ = 0.0f;
	elapsedCpuClocks_ = 0;

	status_ = UNSOLVED;

	return SUCCESSFUL;
}

template<class T>
Error_t Knapsack2D<T>::solve (Algorithm_t algorithm)
{
	double start, finish;
	double polytopeElapsedTime;
	UINT8 clock1, clock2;
	UINT8 polytopeCpuClocks;
	set<array<T,2>> extremePoints;
	array<T,2> tmpPoint;
	Error_t retVal;

	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ == SOLVED)
		return SOLVEDKNAPSACK;

	if(algorithm == ITERATIVE) {
		retVal = ptrKnapsack2DPolytope_->scribe(ITERATIVE);
		if ( retVal != SUCCESSFUL) {
			status_ = UNSOLVED;
			return retVal;
		}

		start = clock();
		clock1 = rdtsc();
		retVal = ptrKnapsack2DPolytope_->getExtremePoints(extremePoints);
		if ( retVal != SUCCESSFUL) {
			status_ = UNSOLVED;
			return retVal;
		}

		optimalPoints_.clear();

		set<array<T,2>>::const_iterator it = extremePoints.begin();
		optValue_ = (*it)[0] * f1_ + (*it)[1] * f2_;

		for( it = extremePoints.begin(); it != extremePoints.end(); it++) {
			optValue_ = ( optValue_ < (*it)[0] * f1_ + (*it)[1] * f2_ ) ? ( (*it)[0] * f1_ + (*it)[1] * f2_ ) : optValue_; 
		}

		for( it = extremePoints.begin(); it != extremePoints.end(); it++) {
			if ( optValue_ == (*it)[0] * f1_ + (*it)[1] * f2_ ) {
				tmpPoint[0] = (*it)[0];
				tmpPoint[1] = (*it)[1];
				optimalPoints_.insert(tmpPoint);
			}
		}
		clock2 = rdtsc();
		finish = clock();
	}
	else {
		return INCORRECTARG;
	}

	if(retVal != SUCCESSFUL)
		return retVal;

	ptrKnapsack2DPolytope_->getElapsedTime ( polytopeElapsedTime );
	ptrKnapsack2DPolytope_->getElapsedCpuClocks ( polytopeCpuClocks );
	elapsedTime_ = polytopeElapsedTime + (double)(finish - start) / CLOCKS_PER_SEC;
	elapsedCpuClocks_ = polytopeCpuClocks + clock2 - clock1;

	status_ = SOLVED;
	
	return SUCCESSFUL;

}

template<class T>
Error_t Knapsack2D<T>::getInitializationError() const { return initializationError_; }

template<class T>
Status_t Knapsack2D<T>::getStatus() const { return status_; }

template<class T>
Error_t Knapsack2D<T>::getElapsedTime(double &elapsedTime) const
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ != SOLVED)
		return UNSOLVEDKNAPSACK;

	elapsedTime = elapsedTime_;
	return SUCCESSFUL;
}

template<class T>
Error_t Knapsack2D<T>::getElapsedCpuClocks (UINT8 &elapsedCpuClocks) const
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ != SOLVED)
		return UNSOLVEDKNAPSACK;

	elapsedCpuClocks = elapsedCpuClocks_;
	return SUCCESSFUL;
}

template<class T>
T Knapsack2D<T>::getParamF1() const { return f1_; }

template<class T>
T Knapsack2D<T>::getParamF2() const { return f2_; }

template<class T>
Error_t Knapsack2D<T>::getInfo() const
{
	set<array<T,2>>::const_iterator it;

	if ( initializationError_ != SUCCESSFUL ) {
		cout << "Knapsack2D task is invalid" << endl;
		return SUCCESSFUL;
	}

	cout << "The Knapsack2D Task:" << endl;
	cout << " max " << f1_ << " * x1 + " << f2_ << " * x2 " << endl;
	cout << "Conv { (x1, x2) :" << ptrKnapsack2DPolytope_->getParamA() << " * x1 + " << ptrKnapsack2DPolytope_->getParamB() << " * x2 <= " << ptrKnapsack2DPolytope_->getParamC() << "; x1, x2 are non-negative integer }" << endl;

	if ( status_ == UNSOLVED ) {
		cout << "The Knapsack2d is unsolved. Please solve it" << endl;
		return SUCCESSFUL;
	} else if ( status_ == SOLVED ) {
		
		if( optimalPoints_.size() == 1) {
			it = optimalPoints_.begin();
			cout << "The optimal point:" << endl << (*it)[0] << "\t" << (*it)[1] << endl;
		} 
		else {
			cout << "The infinite number of optimal points beetween:" << endl;
			for(it = optimalPoints_.begin(); it != optimalPoints_.end(); it++)
				cout << (*it)[0] << "\t" << (*it)[1] << endl;
		}

		cout << endl;
		cout << "The optimal value = " << optValue_ << endl;

		cout << "The time spent for solving task [msec.] = " << elapsedTime_ << endl;
		cout << "CPU clocks consumed for solving task = " << elapsedCpuClocks_ << endl;
	}

	return SUCCESSFUL;
}

template<class T>
Error_t Knapsack2D<T>::getOptimalPoints (set<array<T, 2>> &optimalPoints) const
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ != SOLVED)
		return UNSOLVEDKNAPSACK;

	optimalPoints = optimalPoints_;

	return SUCCESSFUL;
}

template<class T>
Error_t Knapsack2D<T>::getOptValue (T &optValue) const
{
	if(initializationError_ != SUCCESSFUL)
		return initializationError_;

	if(status_ != SOLVED)
		return UNSOLVEDKNAPSACK;

	optValue = optValue_;

	return SUCCESSFUL;
}

template<class T>
Error_t Knapsack2D<T>::getKnapsack2DPolytope (Knapsack2DPolytope<T> & knapsack2DPolytope) const
{
	knapsack2DPolytope = *ptrKnapsack2DPolytope_;

	return SUCCESSFUL;
}

template class Knapsack2D<INT4>; //explicit inst. of INT4 type knapsack problem
template class Knapsack2D<INT8>; //explicit inst. of INT8 type knapsack problem
template class Knapsack2D<big_int>; //explicit inst. of big_int type knapsack problem
