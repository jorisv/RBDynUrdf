// This file is part of RBDyn.
//
// RBDyn is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RBDyn is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with RBDyn.  If not, see <http://www.gnu.org/licenses/>.

#pragma once

// includes
// std
#include <string>

// RBDyn
#include <RBDyn/Body.h>
#include <RBDyn/Joint.h>
#include <RBDyn/MultiBodyGraph.h>

// RBDynUrdf
#include "Reader.h"

// XYZSarm Robot

//                b4
//             j3 | RevX
//  Root     j0   |   j1     j2
//  ---- b0 ---- b1 ---- b2 ----b3
//  Fixed    RevX   RevY    RevZ

//  X
//  ^
//  |
//   -- > Y

std::string XYZSarmUrdf(
R"(
  <robot name="XYZSarm">
    <link name="b0">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0 0" />
        <mass value="1" />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
    </link>
    <link name="b1">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0.5 0" />
        <mass value="5." />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
    </link>
    <link name="b2">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0.5 0" />
        <mass value="2." />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
    </link>
    <link name="b3">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0.5 0" />
        <mass value="1.5" />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
    </link>
    <link name="b4">
      <inertial>
        <origin rpy="0 0 0" xyz="0.5 0 0" />
        <mass value="1" />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
    </link>


    <joint name="j0" type="revolute">
      <parent link="b0" />
      <child link="b1" />
      <origin rpy="0 0 0" xyz="0 1 0" />
      <axis xyz="1 0 0" />
      <limit lower="-1" upper="1" velocity="10" effort="50" />
    </joint>
    <joint name="j1" type="revolute">
      <parent link="b1" />
      <child link="b2" />
      <origin rpy="0 0 0" xyz="0 1 0" />
      <axis xyz="0 1 0" />
      <limit lower="-1" upper="1" velocity="10" effort="50" />
    </joint>
    <joint name="j2" type="revolute">
      <parent link="b2" />
      <child link="b3" />
      <origin rpy="0 0 0" xyz="0 1 0" />
      <axis xyz="0 0 1" />
      <limit lower="-1" upper="1" velocity="10" effort="50" />
    </joint>
    <joint name="j3" type="continuous">
      <parent link="b1" />
      <child link="b4" />
      <origin rpy="1. 0 0" xyz="1 0 0" />
      <axis xyz="1 0 0" />
    </joint>
  </robot>
)"
);



/// @return An simple XYZ spherical arm with Y as up axis.
rbdyn_urdf::Urdf makeXYZSarm()
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  rbdyn_urdf::Urdf urdf;

  Matrix3d I0, I1, I2, I3, I4;

  // computed with rbdyn_urdf python
  I0 << 0.1 , 0.0 , 0.0,
        0.0 , 0.05 , 0.0,
        0.0 , 0.0 , 0.001;

  I1 << 1.35 , 0.0 , 0.0,
        0.0 , 0.05 , 0.0,
        0.0 , 0.0 , 1.251;

  I2 << 0.6 , 0.0 , 0.0,
        0.0 , 0.05 , 0.0,
        0.0 , 0.0 , 0.501;

  I3 << 0.475 , 0.0 , 0.0,
        0.0 , 0.05 , 0.0,
        0.0 , 0.0 , 0.376;

  I4 << 0.1 , 0.0 , 0.0,
        0.0 , 0.3 , 0.0,
        0.0 , 0.0 , 0.251;

  Body b0(1., Vector3d::Zero(), I0, 0, "b0");
  Body b1(5., Vector3d(0., 0.5, 0.), I1, 1, "b1");
  Body b2(2., Vector3d(0., 0.5, 0.), I2, 2, "b2");
  Body b3(1.5, Vector3d(0., 0.5, 0.), I3, 3, "b3");
  Body b4(1., Vector3d(0.5, 0., 0.), I4, 4, "b4");

  urdf.mbg.addBody(b0);
  urdf.mbg.addBody(b1);
  urdf.mbg.addBody(b2);
  urdf.mbg.addBody(b3);
  urdf.mbg.addBody(b4);

  Joint j0(Joint::RevX, true, 0, "j0");
  Joint j1(Joint::RevY, true, 1, "j1");
  Joint j2(Joint::RevZ, true, 2, "j2");
  Joint j3(Joint::RevX, true, 3, "j3");

  urdf.mbg.addJoint(j0);
  urdf.mbg.addJoint(j1);
  urdf.mbg.addJoint(j2);
  urdf.mbg.addJoint(j3);


  PTransformd to(Vector3d(0., 1., 0.));
  PTransformd from(PTransformd::Identity());

  urdf.mbg.linkBodies(0, to, 1, from, 0);
  urdf.mbg.linkBodies(1, to, 2, from, 1);
  urdf.mbg.linkBodies(2, to, 3, from, 2);
  urdf.mbg.linkBodies(1, PTransformd(sva::RotX(1.), Vector3d(1., 0., 0.)),
                      4, from, 3);

  // fill limits
  urdf.limits.ql[0] = -1.;
  urdf.limits.ql[1] = -1.;
  urdf.limits.ql[2] = -1.;

  urdf.limits.qu[0] = 1.;
  urdf.limits.qu[1] = 1.;
  urdf.limits.qu[2] = 1.;

  urdf.limits.vl[0] = -10.;
  urdf.limits.vl[1] = -10.;
  urdf.limits.vl[2] = -10.;

  urdf.limits.vu[0] = 10.;
  urdf.limits.vu[1] = 10.;
  urdf.limits.vu[2] = 10.;

  urdf.limits.tl[0] = -50.;
  urdf.limits.tl[1] = -50.;
  urdf.limits.tl[2] = -50.;

  urdf.limits.tu[0] = 50.;
  urdf.limits.tu[1] = 50.;
  urdf.limits.tu[2] = 50.;

  return std::move(urdf);
}
