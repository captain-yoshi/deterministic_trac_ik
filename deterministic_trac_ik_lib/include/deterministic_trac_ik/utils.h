#ifndef DETERMINISTIC_TRAC_IK_UTILS_H
#define DETERMINISTIC_TRAC_IK_UTILS_H

// standard includes
#include <stdlib.h>
#include <ostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <urdf/model.h>

namespace Deterministic_TRAC_IK {

bool LoadModelOverride(
    const ros::NodeHandle& nh,
    const std::string& robot_description,
    urdf::Model& model);

bool InitKDLChain(
    const urdf::Model& model,
    const std::string& base_name,
    const std::string& tip_name,
    KDL::Chain& chain,
    std::vector<std::string>& link_names,
    std::vector<std::string>& joint_names,
    KDL::JntArray& joint_min,
    KDL::JntArray& joint_max);

} // namespace Deterministic_TRAC_IK

namespace KDL {

std::ostream& operator<<(std::ostream& o, const JntArray& q);
std::ostream& operator<<(std::ostream& o, const Vector& v);
std::ostream& operator<<(std::ostream& o, const Rotation& R);
std::ostream& operator<<(std::ostream& o, const Frame& f);

} // namespace KDl

#endif
