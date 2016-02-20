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

using namespace std;

#include "AuxiliaryTypes.hpp"
#include "Knapsack2D.hpp"

void Input();
void Solve();

INT8 f1, f2, a, b, c;

Knapsack2D<INT8> *task = NULL;

int main(int argc, char* argv[])
{	
	Input();

	Solve();

 	return 0;
}

void Input() {
	cout << "For the following Knapsack Problem: " << endl;
	cout << "\t max ( f1 * x1 + f2 * x2 )" << endl;
	cout << "\t a * x1 + b * x2 <= c " << endl;
	cout << "\t x1, x2 - non-negative integers " << endl;
	cout << "Please specify parameters f1, f2, a, b, c." << endl;

	cout << " f1 = "; cin >> f1;
	cout << " f2 = "; cin >> f2;
	cout << " a = "; cin >> a;
	cout << " b = "; cin >> b;
	cout << " c = "; cin >> c;

	cout << endl << endl << endl;
}

void Solve() {
	task = new Knapsack2D<INT8>( a, b, c, f1, f2);

	task->solve(ITERATIVE);

	task->getInfo();

	if ( task != NULL)
		delete task;
}
