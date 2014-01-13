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

// includes
// std
#include <iostream>

// boost
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE XYZSarm
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// RBDyn
#include <RBDyn/MultiBody.h>

// RBDynUrdf
#include "Reader.h"

// arm
#include "XYZSarm.h"


const double TOL = 1e-6;


BOOST_AUTO_TEST_CASE(loadTest)
{
  rbdyn_urdf::Urdf urdfFromLoader(rbdyn_urdf::readUrdf(XYZSarmUrdf));
  rbdyn_urdf::Urdf urdfFromCpp(makeXYZSarm());

  rbd::MultiBody mbLoader(urdfFromLoader.mbg.makeMultiBody(0, true));
  rbd::MultiBody mbCpp(urdfFromCpp.mbg.makeMultiBody(0, true));

  // basic check
  BOOST_CHECK_EQUAL(mbLoader.nrBodies(), mbCpp.nrBodies());
  BOOST_CHECK_EQUAL(mbLoader.nrJoints(), mbCpp.nrJoints());
  BOOST_CHECK_EQUAL(mbLoader.nrParams(), mbCpp.nrParams());
  BOOST_CHECK_EQUAL(mbLoader.nrDof(), mbCpp.nrDof());

  // mb structure check
  BOOST_CHECK(std::equal(mbLoader.predecessors().begin(),
                         mbLoader.predecessors().end(),
                         mbCpp.predecessors().begin()));
  BOOST_CHECK(std::equal(mbLoader.successors().begin(),
                         mbLoader.successors().end(),
                         mbCpp.successors().begin()));
  BOOST_CHECK(std::equal(mbLoader.parents().begin(),
                         mbLoader.parents().end(),
                         mbCpp.parents().begin()));
  BOOST_CHECK(std::equal(mbLoader.transforms().begin(),
                         mbLoader.transforms().end(),
                         mbCpp.transforms().begin()));

  // check limits
  // position
  BOOST_CHECK(std::equal(urdfFromLoader.ql.begin(), urdfFromLoader.ql.end(),
                         urdfFromCpp.ql.begin()));
  BOOST_CHECK(std::equal(urdfFromLoader.qu.begin(), urdfFromLoader.qu.end(),
                         urdfFromCpp.qu.begin()));

  // velocity
  BOOST_CHECK(std::equal(urdfFromLoader.vl.begin(), urdfFromLoader.vl.end(),
                         urdfFromCpp.vl.begin()));
  BOOST_CHECK(std::equal(urdfFromLoader.vu.begin(), urdfFromLoader.vu.end(),
                         urdfFromCpp.vu.begin()));

  // torque
  BOOST_CHECK(std::equal(urdfFromLoader.tl.begin(), urdfFromLoader.tl.end(),
                         urdfFromCpp.tl.begin()));
  BOOST_CHECK(std::equal(urdfFromLoader.tu.begin(), urdfFromLoader.tu.end(),
                         urdfFromCpp.tu.begin()));

  // check bodies
  for(int i = 0; i < mbLoader.nrBodies(); ++i)
  {
    const rbd::Body& bLoader = mbLoader.body(i);
    const rbd::Body& bCpp = mbCpp.body(i);

    BOOST_CHECK_EQUAL(bLoader.id(), bCpp.id());
    BOOST_CHECK_EQUAL(bLoader.name(), bCpp.name());

    BOOST_CHECK_EQUAL(bLoader.inertia().mass(), bCpp.inertia().mass());
    BOOST_CHECK_EQUAL(bLoader.inertia().momentum(), bCpp.inertia().momentum());
    BOOST_CHECK_SMALL((bLoader.inertia().inertia() - bCpp.inertia().inertia()).norm(),
                      TOL);
  }

  // check joints
  for(int i = 0; i < mbLoader.nrJoints(); ++i)
  {
    const rbd::Joint& jLoader = mbLoader.joint(i);
    const rbd::Joint& jCpp = mbCpp.joint(i);

    BOOST_CHECK_EQUAL(jLoader.id(), jCpp.id());
    BOOST_CHECK_EQUAL(jLoader.name(), jCpp.name());
    BOOST_CHECK_EQUAL(jLoader.type(), jCpp.type());
    BOOST_CHECK_EQUAL(jLoader.direction(), jCpp.direction());
    BOOST_CHECK_EQUAL(jLoader.motionSubspace(), jCpp.motionSubspace());
  }
}
