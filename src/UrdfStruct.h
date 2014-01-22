// This file is part of RBDynUrdf.
//
// RBDynUrdf is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RBDynUrdf is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with RBDynUrdf.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

// include
//std
#include <map>

// RBDyn
#include <RBDyn/MultiBodyGraph.h>

namespace rbdyn_urdf
{

/**
 * @brief Robot limits
 */
struct Limits
{
  Limits() {}
  Limits(const Limits& limit);
  Limits(Limits&& limit);

  Limits& operator=(const Limits& limit);

  /// joints position lower and upper bounds by joint id.
  std::map<int, double> ql, qu;
  /// joints velocity lower and upper bounds by joint id.
  std::map<int, double> vl, vu;
  /// joints torque lower and upper bounds by joint id.
  std::map<int, double> tl, tu;
};



/**
 * @brief Reader return value.
 */
struct Urdf
{
  Urdf() {}
  Urdf(const Urdf& urdf);
  Urdf(Urdf&& urdf);

  Urdf& operator=(const Urdf& urdf);

  rbd::MultiBodyGraph mbg;
  Limits limits;
};

} // rbdyn_urdf
