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
#include "Writer.h"

// include

// boost
#include <boost/make_shared.hpp>

// urdfdom
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

// RBDyn
#include <RBDyn/MultiBody.h>

// RBDynUrdf
#include "UrdfStruct.h"

namespace rbdyn_urdf
{

urdf::Vector3 fromEigen(const Eigen::Vector3d& vec)
{
  return urdf::Vector3(vec.x(), vec.y(), vec.z());
}


urdf::Rotation fromEigen(const Eigen::Quaterniond& quat)
{
  return urdf::Rotation(quat.x(), quat.y(), quat.z(), quat.w());
}


urdf::Rotation fromEigen(const Eigen::Matrix3d& rot)
{
  return fromEigen(Eigen::Quaterniond(rot));
}


urdf::Pose transformToPose(const sva::PTransformd& X)
{
  urdf::Pose pose;
  pose.position = fromEigen(X.translation());
  pose.rotation = fromEigen(Eigen::Matrix3d(X.rotation().inverse()));
  return pose;
}


template <typename Map>
bool exist(const Map& map, const typename Map::key_type& key)
{
  return map.find(key) != map.end();
}


void fillUrdfJoint(const rbd::Joint& rbdJ, const Limits& limits, urdf::Joint& urdfJ)
{
  switch(rbdJ.type())
  {
  case rbd::Joint::Fixed:
    urdfJ.type = urdf::Joint::FIXED;
    break;
  case rbd::Joint::Rev:
    if(exist(limits.ql, rbdJ.id()) && exist(limits.qu, rbdJ.id()))
    {
        urdfJ.type = urdf::Joint::REVOLUTE;
    }
    else
    {
        urdfJ.type = urdf::Joint::CONTINUOUS;
    }
    urdfJ.axis = fromEigen(Eigen::Vector3d(rbdJ.motionSubspace().block<3,1>(0,0)));
    break;
  case rbd::Joint::Prism:
    urdfJ.type = urdf::Joint::PRISMATIC;
    urdfJ.axis = fromEigen(Eigen::Vector3d(rbdJ.motionSubspace().block<3,1>(3,0)));
    break;
  case rbd::Joint::Spherical:
    throw std::runtime_error("Impossible to export spherical joint to urdf");
  case rbd::Joint::Free:
    throw std::runtime_error("Impossible to export free joint to urdf");
  default:
    throw std::runtime_error("Unknow RBDyn joint type");
  }
}


void writeUrdf(const std::string& filename, const urdf::ModelInterface& model)
{
  TiXmlDocument* doc = urdf::exportURDF(model);
  if(!doc->SaveFile(filename))
  {
    throw std::runtime_error(doc->ErrorDesc());
  }
}


void writeUrdf(const std::string& filename, const std::string& robotName,
               const rbd::MultiBody& mb)
{
  Limits limits;
  writeUrdf(filename, robotName, mb, limits);
}


void writeUrdf(const std::string& filename, const std::string& robotName,
               const rbd::MultiBody& mb, const Limits& limits)
{
  urdf::ModelInterface model;
  model.name_ = robotName;

  for(const rbd::Body& b: mb.bodies())
  {
    urdf::Link link;
    link.name = b.name();
    if(b.inertia().mass() != 0.)
    {
      urdf::Inertial inertial;
      Eigen::Vector3d com = b.inertia().momentum()/b.inertia().mass();
      Eigen::Matrix3d inertiaInCoM = sva::inertiaToOrigin<double>(
        b.inertia().inertia(), -b.inertia().mass(), com,
        Eigen::Matrix3d::Identity());

      inertial.ixx = inertiaInCoM(0,0);
      inertial.ixy = inertiaInCoM(0,1);
      inertial.ixz = inertiaInCoM(0,2);
      inertial.iyy = inertiaInCoM(1,1);
      inertial.iyz = inertiaInCoM(1,2);
      inertial.izz = inertiaInCoM(2,2);

      inertial.origin = transformToPose(sva::PTransformd(com));
      inertial.mass = b.inertia().mass();
      link.inertial = boost::make_shared<urdf::Inertial>(inertial);
    }
    model.links_[b.name()] = boost::make_shared<urdf::Link>(link);
  }

  // don't read the first joint (that a virtual joint add by MultiBodyGraph)
  for(int i = 1; i < mb.nrJoints(); ++i)
  {
    const rbd::Joint& j = mb.joint(i);
    urdf::Joint joint;

    joint.name = j.name();
    joint.child_link_name = mb.body(mb.successor(i)).name();
    joint.parent_link_name = mb.body(mb.predecessor(i)).name();
    joint.parent_to_joint_origin_transform = transformToPose(mb.transform(i));
    fillUrdfJoint(j, limits, joint);

    // only export limits if there is position limits or
    // velocity and effort limits
    if((exist(limits.ql, j.id()) && exist(limits.qu, j.id())) ||
       ((exist(limits.vl, j.id()) && exist(limits.vu, j.id())) &&
        (exist(limits.tl, j.id()) && exist(limits.tu, j.id()))))
    {
      urdf::JointLimits urdfLimits;

      if(exist(limits.ql, j.id()) && exist(limits.qu, j.id()))
      {
        urdfLimits.lower = limits.ql.at(j.id());
        urdfLimits.upper = limits.qu.at(j.id());
      }

      if(exist(limits.vl, j.id()) && exist(limits.vu, j.id()))
      {
        urdfLimits.velocity = std::min(std::abs(limits.vl.at(j.id())),
                                       limits.vu.at(j.id()));
      }

      if(exist(limits.tl, j.id()) && exist(limits.tu, j.id()))
      {
        urdfLimits.effort = std::min(std::abs(limits.tl.at(j.id())),
                                     limits.tu.at(j.id()));
      }
      joint.limits = boost::make_shared<urdf::JointLimits>(urdfLimits);
    }
    model.joints_[j.name()] = boost::make_shared<urdf::Joint>(joint);
  }

  model.root_link_ = model.links_[mb.body(0).name()];

  writeUrdf(filename, model);
}

} // rbdyn_urdf
