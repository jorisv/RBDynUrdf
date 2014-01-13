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
// std
#include <string>

// RBDyn
#include <RBDyn/MultiBodyGraph.h>

namespace rbdyn_urdf
{

/**
 * @brief Main urdf data.
 */
struct Urdf
{
  Urdf() {}
  Urdf(Urdf&& urdf);

  rbd::MultiBodyGraph mbg;
  /// joints position lower and upper bounds by joint id.
  std::map<int, double> ql, qu;
  /// joints velocity lower and upper bounds by joint id.
  std::map<int, double> vl, vu;
  /// joints torque lower and upper bounds by joint id.
  std::map<int, double> tl, tu;
};


/**
 * @brief Create a MultiBodyGraph an his limits from a urdf string.
 * @param urdf urdf string.
 * @return MultiBodyGraph and his limits.
 * @throw runtime_error if the urdf file is not well formated.
 */
Urdf readUrdf(const std::string& urdf);

/**
 * @brief Create a MultiBodyGraph an his limits from a urdf file path.
 * @param urdf urdf file path.
 * @return MultiBodyGraph and his limits.
 * @throw runtime_error if the urdf file is not well formated or if the file
 * is not readable.
 */
Urdf readUrdfFile(const std::string& fileName);


} // rbdyn_urdf
