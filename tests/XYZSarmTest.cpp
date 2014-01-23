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
#include "Writer.h"

// arm
#include "XYZSarm.h"


const double TOL = 1e-6;


void checkModel(rbdyn_urdf::Urdf urdf1, rbdyn_urdf::Urdf urdf2)
{
  rbd::MultiBody mb1(urdf1.mbg.makeMultiBody(0, true));
  rbd::MultiBody mb2(urdf2.mbg.makeMultiBody(0, true));

  // basic check
  BOOST_CHECK_EQUAL(mb1.nrBodies(), mb2.nrBodies());
  BOOST_CHECK_EQUAL(mb1.nrJoints(), mb2.nrJoints());
  BOOST_CHECK_EQUAL(mb1.nrParams(), mb2.nrParams());
  BOOST_CHECK_EQUAL(mb1.nrDof(), mb2.nrDof());

  // mb structure check
  BOOST_CHECK(std::equal(mb1.predecessors().begin(),
                         mb1.predecessors().end(),
                         mb2.predecessors().begin()));
  BOOST_CHECK(std::equal(mb1.successors().begin(),
                         mb1.successors().end(),
                         mb2.successors().begin()));
  BOOST_CHECK(std::equal(mb1.parents().begin(),
                         mb1.parents().end(),
                         mb2.parents().begin()));
  BOOST_CHECK(std::equal(mb1.transforms().begin(),
                         mb1.transforms().end(),
                         mb2.transforms().begin()));

  // check limits
  // position
  BOOST_CHECK(std::equal(urdf1.limits.ql.begin(),
                         urdf1.limits.ql.end(),
                         urdf2.limits.ql.begin()));
  BOOST_CHECK(std::equal(urdf1.limits.qu.begin(),
                         urdf1.limits.qu.end(),
                         urdf2.limits.qu.begin()));

  // velocity
  BOOST_CHECK(std::equal(urdf1.limits.vl.begin(),
                         urdf1.limits.vl.end(),
                         urdf2.limits.vl.begin()));
  BOOST_CHECK(std::equal(urdf1.limits.vu.begin(),
                         urdf1.limits.vu.end(),
                         urdf2.limits.vu.begin()));

  // torque
  BOOST_CHECK(std::equal(urdf1.limits.tl.begin(),
                         urdf1.limits.tl.end(),
                         urdf2.limits.tl.begin()));
  BOOST_CHECK(std::equal(urdf1.limits.tu.begin(),
                         urdf1.limits.tu.end(),
                         urdf2.limits.tu.begin()));

  // check bodies
  for(int i = 0; i < mb1.nrBodies(); ++i)
  {
    const rbd::Body& b1 = mb1.body(i);
    const rbd::Body& b2 = mb2.body(i);

    BOOST_CHECK_EQUAL(b1.id(), b2.id());
    BOOST_CHECK_EQUAL(b1.name(), b2.name());

    BOOST_CHECK_EQUAL(b1.inertia().mass(), b2.inertia().mass());
    BOOST_CHECK_EQUAL(b1.inertia().momentum(), b2.inertia().momentum());
    BOOST_CHECK_SMALL((b1.inertia().inertia() - b2.inertia().inertia()).norm(),
                      TOL);
  }

  // check joints
  for(int i = 0; i < mb1.nrJoints(); ++i)
  {
    const rbd::Joint& j1 = mb1.joint(i);
    const rbd::Joint& j2 = mb2.joint(i);

    BOOST_CHECK_EQUAL(j1.id(), j2.id());
    BOOST_CHECK_EQUAL(j1.name(), j2.name());
    BOOST_CHECK_EQUAL(j1.type(), j2.type());
    BOOST_CHECK_EQUAL(j1.direction(), j2.direction());
    BOOST_CHECK_EQUAL(j1.motionSubspace(), j2.motionSubspace());
  }
}


BOOST_AUTO_TEST_CASE(loadTest)
{
  rbdyn_urdf::Urdf urdfFromLoader(rbdyn_urdf::readUrdf(XYZSarmUrdf));
  rbdyn_urdf::Urdf urdfFromCpp(makeXYZSarm());

  checkModel(urdfFromLoader, urdfFromCpp);
}

BOOST_AUTO_TEST_CASE(saveTest)
{
  rbdyn_urdf::Urdf urdfFromCpp(makeXYZSarm());
  std::string filename("XYZSArm.urdf");
  rbdyn_urdf::writeUrdf(filename, "XYZSarm",
                        urdfFromCpp.mbg.makeMultiBody(0, true),
                        urdfFromCpp.limits);
  rbdyn_urdf::Urdf urdfFromLoader(rbdyn_urdf::readUrdfFile(filename));

  checkModel(urdfFromCpp, urdfFromLoader);
}
