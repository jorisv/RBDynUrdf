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

// forward declaration
namespace rbd
{
  class MultiBody;
}

namespace rbdyn_urdf
{
// forward declaration
class Limits;

/**
 * @brief write the MultiBody mb as an urdf file.
 * @param filename File where to write the urdf.
 * @param robotName Name of the robot in the urdf file.
 * @param mb MultiBody to write in the urdf.
 * @throw runtime_error if the file is not writable or if joints type are wrong.
 */
void writeUrdf(const std::string& filename, const std::string& robotName,
               const rbd::MultiBody& mb);

/**
 * @brief write the MultiBody mb as an urdf file.
 * @param filename File where to write the urdf.
 * @param robotName Name of the robot in the urdf file.
 * @param mb MultiBody to write in the urdf.
 * @param limits MultiBody joint limits.
 * @throw runtime_error if the file is not writable or if joints type are wrong.
 */
void writeUrdf(const std::string& filename, const std::string& robotName,
               const rbd::MultiBody& mb, const Limits& limits);

} // rbdyn_urdf
