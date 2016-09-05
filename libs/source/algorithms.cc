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

#include <stdexcept>

#include "libs/headers/algorithms.h"
#include "libs/headers/types.h"

template <class T> void ComputeResidue(T const &a, T const &b, T *res_ptr) {
  if (res_ptr == nullptr) {
    throw std::invalid_argument("Specified null pointer to output value.");
  }

  if (b <= 0) {
    throw std::invalid_argument("Specified incorrect modulus value");
  }

  *res_ptr = a % b;
  if (*res_ptr < 0) {
    *res_ptr += b;
  }
}
template void ComputeResidue(OctopusInt4 const &a, OctopusInt4 const &b,
                             OctopusInt4 *res_ptr);
template void ComputeResidue(OctopusInt8 const &a, OctopusInt8 const &b,
                             OctopusInt8 *res_ptr);

template <class T>
void ComputeReducedResidue(T const &a, T const &b, T *reduced_res_ptr) {
  if (reduced_res_ptr == nullptr) {
    throw std::invalid_argument("Specified null pointer to output value.");
  }

  if (b <= 0) {
    throw std::invalid_argument("Specified incorrect modulus value");
  }

  ComputeResidue(a, b, reduced_res_ptr);

  if (*reduced_res_ptr > b / 2)
    *reduced_res_ptr -= b;
}
template void ComputeReducedResidue(OctopusInt4 const &a, OctopusInt4 const &b,
                                    OctopusInt4 *reduced_res_ptr);
template void ComputeReducedResidue(OctopusInt8 const &a, OctopusInt8 const &b,
                                    OctopusInt8 *reduced_res_ptr);

template <class T> void ComputeGcd(T const &a, T const &b, T *gcd_ptr) {
  if (gcd_ptr == nullptr) {
    throw std::invalid_argument(
        "Specified null pointer to store output values.");
  }

  if (a == 0 && b == 0) {
    throw std::invalid_argument(
        "Unable to compute GCD for both zero parameters.");
  }
  T aa = a, bb = b;

  if (aa < 0) {
    aa = -aa;
  }
  if (bb < 0) {
    bb = -bb;
  }

  if (bb == 0) {
    aa = aa ^ bb;
    bb = aa ^ bb;
    aa = aa ^ bb;
  }

  T r = aa % bb;

  while (r != 0) {
    aa = bb;
    bb = r;
    r = aa % bb;
  }

  *gcd_ptr = bb;
}

template void ComputeGcd(OctopusInt4 const &a, OctopusInt4 const &b,
                         OctopusInt4 *gcd);
template void ComputeGcd(OctopusInt8 const &a, OctopusInt8 const &b,
                         OctopusInt8 *gcd);

template <class T>
void ComputeGcdAndBezoutCoeffs(T const &a, T const &b, T *t1_ptr, T *t2_ptr,
                               T *gcd_ptr) {
  if (t1_ptr == nullptr || t2_ptr == nullptr || gcd_ptr == nullptr) {
    throw std::invalid_argument(
        "Specified null pointer to store output values.");
  }

  if (a == 0 && b == 0) {
    throw std::invalid_argument(
        "Unable to compute GCD for both zero parameters.");
  }

  T aa = a, bb = b;
  T x1 = 1, x2 = 0;
  T y1 = 0, y2 = 1;
  T r = aa % bb;

  do {
    T q = aa / bb;
    r = aa % bb;
    aa = bb;
    bb = r;

    T tmp = x2;
    x2 = x1 - q * x2;
    x1 = tmp;

    tmp = y2;
    y2 = y1 - q * y2;
    y1 = tmp;
  } while (r != 0);

  *gcd_ptr = aa;
  *t1_ptr = x1;
  *t2_ptr = y1;
}
template void ComputeGcdAndBezoutCoeffs(OctopusInt4 const &a,
                                        OctopusInt4 const &b,
                                        OctopusInt4 *t1_ptr,
                                        OctopusInt4 *t2_ptr,
                                        OctopusInt4 *gdc_ptr);
template void ComputeGcdAndBezoutCoeffs(OctopusInt8 const &a,
                                        OctopusInt8 const &b,
                                        OctopusInt8 *t1_ptr,
                                        OctopusInt8 *t2_ptr,
                                        OctopusInt8 *gdc_ptr);

template <class T>
void FindExtremePointsAt2dGmtPolytop(
    T const &alpha, T const &gamma, T const &delta,
    std::set<std::array<T, 2>> *extreme_points_ptr) {
  // t1 + alpha * t2 = gamma (mod delta), where t1, t2 non-negative integers
  if (extreme_points_ptr == nullptr) {
    throw std::invalid_argument("Specified null pointer to output value.");
  }

  if (delta <= 0) {
    throw std::invalid_argument(
        "Specified incorrect value for delta (modulus).");
  }

  // clean up a set
  extreme_points_ptr->clear();

  T red_res_alpha{};
  T res_gamma{};

  ComputeReducedResidue(alpha, delta, &red_res_alpha);
  ComputeResidue(gamma, delta, &res_gamma);

  if (red_res_alpha == 0) {
    extreme_points_ptr->insert({{res_gamma, 0}});
  } else if (red_res_alpha == 1) {
    extreme_points_ptr->insert({{res_gamma, 0}});
    extreme_points_ptr->insert({{0, res_gamma}});
  } else if (red_res_alpha == -1) {
    extreme_points_ptr->insert({{res_gamma, 0}});
    extreme_points_ptr->insert({{0, delta - res_gamma}});
  } else if ((red_res_alpha > 1) && (red_res_alpha <= delta / 2)) {
    std::set<std::array<T, 2>> tmp_extreme_points;
    T new_alpha = -delta;
    T new_delta = red_res_alpha;
    FindExtremePointsAt2dGmtPolytop(new_alpha, res_gamma, new_delta,
                                    &tmp_extreme_points);

    for (const auto &point : tmp_extreme_points) {
      extreme_points_ptr->insert(
          {{point[0], (gamma + delta * point[1] - point[0]) / alpha}});
    }

  } else if ((red_res_alpha < -1) && (2 * red_res_alpha > -delta)) {
    extreme_points_ptr->insert({{res_gamma, 0}});
    std::set<std::array<T, 2>> tmp_extreme_points;
    T new_alpha = delta;
    T new_gamma = res_gamma - delta;
    T new_delta = -red_res_alpha;

    FindExtremePointsAt2dGmtPolytop(new_alpha, new_gamma, new_delta,
                                    &tmp_extreme_points);

    for (const auto &point : tmp_extreme_points) {
      extreme_points_ptr->insert(
          {{point[0], (gamma - delta - delta * point[1] - point[0]) / alpha}});
    }

  } else {
    throw std::logic_error("Internal algorithm error.");
  }

  extreme_points_ptr->insert({{gamma, 0}});
}

template void FindExtremePointsAt2dGmtPolytop(
    OctopusInt4 const &alpha, OctopusInt4 const &gamma,
    OctopusInt4 const &delta,
    std::set<std::array<OctopusInt4, 2>> *extreme_points_ptr);
template void FindExtremePointsAt2dGmtPolytop(
    OctopusInt8 const &alpha, OctopusInt8 const &gamma,
    OctopusInt8 const &delta,
    std::set<std::array<OctopusInt8, 2>> *extreme_points_ptr);

template <class T>
void SolveLinearCongruenceEquation(T const &a, T const &b, T const &d,
                                   T *x_ptr) {
  if (x_ptr == nullptr) {
    throw std::invalid_argument("Specified null pointer to output value.");
  }

  if (d <= 0) {
    throw std::invalid_argument(
        "Specified incorrect value for delta (modulus).");
  }

  T res_alpha{}, res_gamma{};
  T gcd{}, t1{}, t2{};

  ComputeResidue(a, d, &res_alpha);
  ComputeResidue(b, d, &res_gamma);

  ComputeGcdAndBezoutCoeffs(res_alpha, d, &t1, &t2, &gcd);

  if (res_gamma % gcd != 0) {
    throw std::logic_error("No solutions are found.");
  }

  T x = t1 * res_gamma / gcd;

  ComputeResidue(x, d, x_ptr);
}

template void SolveLinearCongruenceEquation(OctopusInt4 const &a,
                                            OctopusInt4 const &b,
                                            OctopusInt4 const &d,
                                            OctopusInt4 *x_ptr);
template void SolveLinearCongruenceEquation(OctopusInt8 const &a,
                                            OctopusInt8 const &b,
                                            OctopusInt8 const &d,
                                            OctopusInt8 *x_ptr);
