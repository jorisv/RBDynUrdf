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

// associated header
#include "UrdfStruct.h"

namespace rbdyn_urdf
{

/*
 *                       Limits
 */

Limits::Limits(const Limits& limit)
  : ql(limit.ql)
  , qu(limit.qu)
  , vl(limit.vl)
  , vu(limit.vu)
  , tl(limit.tl)
  , tu(limit.tu)
{}


Limits::Limits(Limits&& limit)
{
  std::swap(ql, limit.ql);
  std::swap(qu, limit.qu);
  std::swap(vl, limit.vl);
  std::swap(vu, limit.vu);
  std::swap(tl, limit.tl);
  std::swap(tu, limit.tu);
}


Limits& Limits::operator=(const Limits& limit)
{
  if(&limit != this)
  {
    ql = limit.ql;
    qu = limit.qu;
    vl = limit.vl;
    vu = limit.vu;
    tl = limit.tl;
    tu = limit.tu;
  }
  return *this;
}

/*
 *                       Urdf
 */

Urdf::Urdf(const Urdf& urdf)
  : mbg(urdf.mbg)
  , limits(urdf.limits)
{}


Urdf::Urdf(Urdf&& urdf)
{
  std::swap(mbg, urdf.mbg);
  std::swap(limits, urdf.limits);
}


Urdf& Urdf::operator=(const Urdf& urdf)
{
  if(&urdf != this)
  {
    mbg = urdf.mbg;
    limits = urdf.limits;
  }
  return *this;
}

} // rbdyn_urdf
