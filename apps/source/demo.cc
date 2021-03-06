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
#include <memory>

#include "libs/headers/types.h"
#include "libs/headers/knapsack_2d_polytop.h"
#include "libs/headers/inverse_knapsack_2d_polyhedron.h"

void RunSolver(Task *task_ptr) {
  task_ptr->Solve(Task::OctopusAlgorithmType::kOctopusIterativeAlg);

  task_ptr->Write(std::cout);
}

int main(int argc, char *argv[]) {
  OctopusInt8 a, b, c;

  std::cout << "For the following Knapsack Problem Polytop: " << std::endl;
  std::cout << "\t a * x1 + b * x2 <= c " << std::endl;
  std::cout << "\t x1, x2 - non-negative integers " << std::endl;
  std::cout << "Please specify parameters a, b, c." << std::endl;

  std::cout << " a = ";
  std::cin >> a;
  std::cout << " b = ";
  std::cin >> b;
  std::cout << " c = ";
  std::cin >> c;

  std::cout << std::endl;

  try {
    std::unique_ptr<Knapsack2dPolytope<OctopusInt8>>
        knapsack_2d_polytop_task_ptr(
            new Knapsack2dPolytope<OctopusInt8>(a, b, c));

    RunSolver(knapsack_2d_polytop_task_ptr.get());

    std::unique_ptr<InverseKnapsack2dPolyhedron<OctopusInt8>>
        inverse_knapsack_2d_polyhedron_task_ptr(
            new InverseKnapsack2dPolyhedron<OctopusInt8>(a, b, c));

    RunSolver(inverse_knapsack_2d_polyhedron_task_ptr.get());
  } catch (std::exception const &ex) {
    std::cout << ex.what() << std::endl;
  }

  return 0;
}
