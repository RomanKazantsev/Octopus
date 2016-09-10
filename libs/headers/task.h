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

/*!
* \file
* \brief Abstract class for task.
*/
#ifndef LIBS_HEADERS_TASK_H_
#define LIBS_HEADERS_TASK_H_

#include <iostream>

/*!
Abstract class for task.
*/
class Task {
 public:
  /// task status
  enum class OctopusTaskStatus { kOctopusUnsolved, kOctopusSolved };
  /// algorithm type: iterative or parallel
  enum class OctopusAlgorithmType { kOctopusIterativeAlg, kOctopusParallelAlg };

  /// constructor
  Task()
      : status_(OctopusTaskStatus::kOctopusUnsolved)
      , alg_type_(OctopusAlgorithmType::kOctopusIterativeAlg) {}
  /// copy constructor
  Task(Task const& other)
      : status_(other.status_), alg_type_(other.alg_type_) {}
  /// reset a task
  virtual void Reset() = 0;
  /// solve a task
  virtual void Solve(OctopusAlgorithmType alg_type) = 0;
  /// write information about polytop to a file
  virtual void Write(std::ostream& s) {
    switch (status_) {
      case OctopusTaskStatus::kOctopusSolved:
        s << "Status: solved" << std::endl;
        break;
      case OctopusTaskStatus::kOctopusUnsolved:
        s << "Status: unsolved" << std::endl;
        break;
      default:
        throw std::runtime_error("Unknown task status.");
    }

    switch (alg_type_) {
      case Task::OctopusAlgorithmType::kOctopusIterativeAlg:
        s << "Algorithm: iterative" << std::endl;
        break;
      default:
        throw std::runtime_error("Unknown or unsupported algorithm.");
    }
  }
  /// get task status
  virtual OctopusTaskStatus GetTaskStatus() const final { return status_; }
  /// get algorithm type
  virtual OctopusAlgorithmType GetAlgorithmType() const final {
    return alg_type_;
  }
  /// assignment operator
  Task& operator=(Task const& other) {
    status_ = other.status_;
    alg_type_ = other.alg_type_;
    return *this;
  }
  /// destructor
  virtual ~Task() {}

 protected:
  OctopusTaskStatus status_;
  OctopusAlgorithmType alg_type_;
};

#endif  // LIBS_HEADERS_TASK_H_
