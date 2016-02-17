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

#include "AuxiliaryFuncs.hpp"
#include "big_int.hpp"

using namespace Arageli;

UINT8 rdtsc() {
    return (UINT8)__rdtsc();
}

template<class Type>
Error_t EuclidAlg(Type a, Type b, Type &GCD)
{
	Type r, tmp;

	if ( a == 0 && b == 0)
		return INCORRECTARG;

	if( a < 0 )
		a = -a;
	if( b < 0 )
		b = -b;

	if( b == 0 ) {
		tmp = a;
		a = b;
		b = tmp;
	}

	r = a % b;

	while(r != 0) {
		a = b;
		b = r;
		r = a % b;
	}

	GCD = b;

	return SUCCESSFUL;
}
template Error_t EuclidAlg(INT4 a, INT4 b, INT4 &GCD);
template Error_t EuclidAlg(INT8 a, INT8 b, INT8 &GCD);
template Error_t EuclidAlg(big_int a, big_int b, big_int &GCD);

//extended euclid algorithm. a and b should be positive > 0
//---------------- a*t1 + b*t2 = GCD(a,b), where t1 and t2 are integers 
template<class T>
Error_t ExtendedEuclidAlg(T a, T b, T &t1, T &t2, T &GCD)
{
	T x1 = 1,x2 = 0;
	T y1 = 0, y2 = 1;
	T tmp;

	T r,q;

	if ( a == 0 && b == 0 )
		return INCORRECTARG;

	do {
		q = a / b;
		r = a % b;
		a = b;
		b = r;

		tmp = x2;
		x2 = x1 - q * x2;
		x1 = tmp;

		tmp = y2;
		y2 = y1 - q * y2;
		y1 = tmp;
	}while(r != 0);

	GCD = a;
	t1 = x1;
	t2 = y1;

	return SUCCESSFUL;
}
template Error_t ExtendedEuclidAlg(INT4 a, INT4 b, INT4 &t1, INT4 &t2, INT4 &GCD);
template Error_t ExtendedEuclidAlg(INT8 a, INT8 b, INT8 &t1, INT8 &t2, INT8 &GCD);
template Error_t ExtendedEuclidAlg(big_int a, big_int b, big_int &t1, big_int &t2, big_int &GCD);
