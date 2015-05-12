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
#include "Reader.h"

// include
// std
#include <fstream>
#include <unordered_map>

// urdfdom
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace rbdyn_urdf
{


Eigen::Vector3d toEigen(const urdf::Vector3& vec)
{
  return Eigen::Vector3d(vec.x, vec.y, vec.z);
}


sva::PTransformd poseToPTransform(const urdf::Pose& pose)
{
  Eigen::Quaterniond rot;
  pose.rotation.getQuaternion(rot.x(), rot.y(), rot.z(), rot.w());
  // inverse the rotation since rbdyn use anti trig rotation
  return sva::PTransformd(rot.inverse(), toEigen(pose.position));
}


rbd::Joint::Type urdfJointTypeToRbd(int type)
{
  switch(type)
  {
  case urdf::Joint::REVOLUTE:
  case urdf::Joint::CONTINUOUS:
    return rbd::Joint::Rev;
  case urdf::Joint::FIXED:
    return rbd::Joint::Fixed;
  case urdf::Joint::PRISMATIC:
    return rbd::Joint::Prism;
  case urdf::Joint::FLOATING:
    return rbd::Joint::Free;
  case urdf::Joint::PLANAR:
    return rbd::Joint::Planar;

  // unknown joint type
  case urdf::Joint::UNKNOWN:
  default:
    throw std::runtime_error("Unknow joint type");
  }
}


Urdf mbgFromModel(const boost::shared_ptr<urdf::ModelInterface>& model)
{
  if(!model)
  {
    throw std::runtime_error("Urdf file is not valid");
  }

  Urdf urdf;

  std::unordered_map<std::string, int> linkNameToId;

  int linkId = 0;
  for(const auto& it: model->links_)
  {
    sva::RBInertiad inertia(0., Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero());

    if(it.second->inertial)
    {
      const urdf::Inertial& inertiaUrdf = *it.second->inertial;

      // mass
      double mass = inertiaUrdf.mass;

      // com position
      sva::PTransformd com(poseToPTransform(inertiaUrdf.origin));

      // inertia
      Eigen::Matrix3d inertiaMat;
      inertiaMat << inertiaUrdf.ixx, inertiaUrdf.ixy, inertiaUrdf.ixz,
                    inertiaUrdf.ixy, inertiaUrdf.iyy, inertiaUrdf.iyz,
                    inertiaUrdf.ixz, inertiaUrdf.iyz, inertiaUrdf.izz;

      // set inertia in body origin frame (not com frame)
      inertiaMat = sva::inertiaToOrigin(inertiaMat, mass,
                                        com.translation(), com.rotation());
      inertia = sva::RBInertiad(mass, com.translation()*mass, inertiaMat);
    }

    rbd::Body body(inertia, linkId, it.second->name);
    urdf.mbg.addBody(body);
    // store name to id map
    linkNameToId[it.second->name] = linkId;

    ++linkId;
  }

  int jointId = 0;
  for(const auto& it: model->joints_)
  {
    int jointParentId = linkNameToId[it.second->parent_link_name];
    int jointChildId = linkNameToId[it.second->child_link_name];

    sva::PTransformd staticTf(poseToPTransform(it.second->parent_to_joint_origin_transform));
    rbd::Joint::Type jType = urdfJointTypeToRbd(it.second->type);

    // create rbd joint type from urdf joint
    rbd::Joint joint(jType, toEigen(it.second->axis), true, jointId, it.second->name);
    urdf.mbg.addJoint(joint);
    urdf.mbg.linkBodies(jointParentId, staticTf,
                   jointChildId, sva::PTransformd::Identity(), jointId);


    // fill joint limits
    if(it.second->limits)
    {
      urdf.limits.ql[jointId] = it.second->limits->lower;
      urdf.limits.qu[jointId] = it.second->limits->upper;
      urdf.limits.vl[jointId] = -it.second->limits->velocity;
      urdf.limits.vu[jointId] = it.second->limits->velocity;
      urdf.limits.tl[jointId] = -it.second->limits->effort;
      urdf.limits.tu[jointId] = it.second->limits->effort;
    }

    ++jointId;
  }

  return std::move(urdf);
}


Urdf readUrdf(const std::string& urdf)
{
  return mbgFromModel(urdf::parseURDF(urdf));
}


Urdf readUrdfFile(const std::string& filename)
{
  std::ifstream file(filename);
  if(!file)
  {
    std::ostringstream ss;
    ss << "open failed: " << filename;
    throw std::runtime_error(ss.str());
  }

  // get length of file:
  file.seekg(0, file.end);
  std::size_t length = file.tellg();
  file.seekg(0, file.beg);

  std::string urdf;
  urdf.resize(length, ' '); // reserve space
  char* begin = &*urdf.begin();

  file.read(begin, length);
  file.close();

  return mbgFromModel(urdf::parseURDF(urdf));
}


} // rbdyn_urdf
