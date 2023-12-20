/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2023,
 *  TU Dortmund University, Institute of Control Theory and System Engineering
 *  All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
*  Authors: Maximilian Krämer
*  Maintainer(s)/Modifier(s): Heiko Renz
 *********************************************************************/

#include <mhp_robot/robot_dynamic/robot_dynamic.h>
#include <kdl_parser/kdl_parser.hpp>

namespace mhp_robot {
namespace robot_dynamic {

RobotDynamic::RobotDynamic(RobotUtility::UPtr robot_utility)
    : _robot_utility(std::move(robot_utility)), _dynamic_kdl(getDynamicKDL(_robot_utility->getRobotDescription()))

{
    initialize();
}

RobotDynamic::RobotDynamic(const std::string& urdf_param, const std::string& first_frame, const std::string& last_frame)
    : _robot_utility(std::make_unique<RobotUtility>(urdf_param, first_frame, last_frame)),
      _dynamic_kdl(getDynamicKDL(_robot_utility->getRobotDescription()))

{
    initialize();
}

RobotDynamic::UPtr RobotDynamic::createUniqueInstance() const
{
    return std::make_unique<RobotDynamic>(_robot_utility->getRobotDescription().getURDFParam(), _robot_utility->getRobotDescription().getFirstFrame(),
                                          _robot_utility->getRobotDescription().getLastFrame());
}

RobotDynamic::Ptr RobotDynamic::createSharedInstance() const
{
    return std::make_shared<RobotDynamic>(_robot_utility->getRobotDescription().getURDFParam(), _robot_utility->getRobotDescription().getFirstFrame(),
                                          _robot_utility->getRobotDescription().getLastFrame());
}

const robot_misc::RobotUtility& RobotDynamic::getRobotUtility() const { return *_robot_utility; }

void RobotDynamic::setJointPosition(const Eigen::Ref<const Eigen::VectorXd>& joint_position) { _position_kdl.data = joint_position; }

void RobotDynamic::setJointVelocity(const Eigen::Ref<const Eigen::VectorXd>& joint_velocity) { _velocity_kdl.data = joint_velocity; }

void RobotDynamic::setJointAcceleration(const Eigen::Ref<const Eigen::VectorXd>& joint_acceleration) { _joint_acceleration = joint_acceleration; }

void RobotDynamic::setJointState(const Eigen::Ref<const Eigen::VectorXd>& joint_position, const Eigen::Ref<const Eigen::VectorXd>& joint_velocity,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_acceleration)
{
    setJointPosition(joint_position);
    setJointVelocity(joint_velocity);
    setJointAcceleration(joint_acceleration);
}

const Eigen::Ref<const Eigen::MatrixXd> RobotDynamic::getInertiaMatrix()
{
    if (!_position_kdl.data.isApprox(_joint_position_inertia_matrix))
    {
        calculateInertiaMatrix();
        _joint_position_inertia_matrix = _position_kdl.data;
    }
    return _inertia_kdl.data;
}

const Eigen::Ref<const Eigen::MatrixXd> RobotDynamic::getInertiaMatrix(const Eigen::Ref<const Eigen::VectorXd>& joint_position)
{
    setJointPosition(joint_position);
    return getInertiaMatrix();
}

const Eigen::Ref<const Eigen::VectorXd> RobotDynamic::getInertiaTerm()
{
    if (!_position_kdl.data.isApprox(_joint_position_inertia_term) || !_joint_acceleration.isApprox(_joint_acceleration_inertia_term))
    {
        calculateInertiaTerm();
        _joint_position_inertia_term     = _position_kdl.data;
        _joint_acceleration_inertia_term = _joint_acceleration;
    }
    return _inertia_term;
}

const Eigen::Ref<const Eigen::VectorXd> RobotDynamic::getInertiaTerm(const Eigen::Ref<const Eigen::VectorXd>& joint_position,
                                                                     const Eigen::Ref<const Eigen::VectorXd>& joint_acceleration)
{
    setJointPosition(joint_position);
    setJointAcceleration(joint_acceleration);
    return getInertiaTerm();
}

const Eigen::Ref<const Eigen::VectorXd> RobotDynamic::getVelocityProductTerm()
{
    if (!_position_kdl.data.isApprox(_joint_position_velocity_product) || !_velocity_kdl.data.isApprox(_joint_velocity_velocity_product))
    {
        calculateVelocityProductTerm();
        _joint_position_velocity_product = _position_kdl.data;
        _joint_velocity_velocity_product = _velocity_kdl.data;
    }
    return _velocity_product_kdl.data;
}

const Eigen::Ref<const Eigen::VectorXd> RobotDynamic::getVelocityProductTerm(const Eigen::Ref<const Eigen::VectorXd>& joint_position,
                                                                             const Eigen::Ref<const Eigen::VectorXd>& joint_velocity)
{
    setJointPosition(joint_position);
    setJointVelocity(joint_velocity);
    return getVelocityProductTerm();
}

const Eigen::Ref<const Eigen::VectorXd> RobotDynamic::getGravityTerm()
{
    if (!_position_kdl.data.isApprox(_joint_position_gravity))
    {
        calculateGravityTerm();
        _joint_position_gravity = _position_kdl.data;
    }
    return _gravity_kdl.data;
}

const Eigen::Ref<const Eigen::VectorXd> RobotDynamic::getGravityTerm(const Eigen::Ref<const Eigen::VectorXd>& joint_position)
{
    setJointPosition(joint_position);
    return getGravityTerm();
}

void RobotDynamic::calculateInertiaMatrix() { _dynamic_kdl.JntToMass(_position_kdl, _inertia_kdl); }

void RobotDynamic::calculateInertiaTerm() { _inertia_term.noalias() = getInertiaMatrix() * _joint_acceleration; }

void RobotDynamic::calculateVelocityProductTerm() { _dynamic_kdl.JntToCoriolis(_position_kdl, _velocity_kdl, _velocity_product_kdl); }

void RobotDynamic::calculateGravityTerm() { _dynamic_kdl.JntToGravity(_position_kdl, _gravity_kdl); }

KDL::ChainDynParam RobotDynamic::getDynamicKDL(const robot_description::RobotDescription& robot_description) const
{
    KDL::Tree tree;
    KDL::Chain chain_kdl;

    urdf::Model model = robot_description.getURDFModel();
    kdl_parser::treeFromUrdfModel(model, tree);
    tree.getChain(robot_description.getFirstFrame(), robot_description.getLastFrame(), chain_kdl);

    return KDL::ChainDynParam(chain_kdl, KDL::Vector(0, 0, -9.80665));
}

void RobotDynamic::applyInfinity(Eigen::Ref<Eigen::VectorXd> vector) const
{
    for (int i = 0; i < vector.size(); i++) vector[i] = std::numeric_limits<double>::infinity();
}

void RobotDynamic::initialize()
{
    _joint_count                     = _robot_utility->getJointsCount();
    _position_kdl                    = KDL::JntArray(_joint_count);
    _velocity_kdl                    = KDL::JntArray(_joint_count);
    _joint_acceleration              = Eigen::VectorXd(_joint_count);
    _inertia_term                    = Eigen::VectorXd(_joint_count);
    _inertia_kdl                     = KDL::JntSpaceInertiaMatrix(_joint_count);
    _velocity_product_kdl            = KDL::JntArray(_joint_count);
    _gravity_kdl                     = KDL::JntArray(_joint_count);
    _joint_position_inertia_matrix   = Eigen::VectorXd(_joint_count);
    _joint_position_inertia_term     = Eigen::VectorXd(_joint_count);
    _joint_acceleration_inertia_term = Eigen::VectorXd(_joint_count);
    _joint_position_velocity_product = Eigen::VectorXd(_joint_count);
    _joint_velocity_velocity_product = Eigen::VectorXd(_joint_count);
    _joint_position_gravity          = Eigen::VectorXd(_joint_count);

    applyInfinity(_joint_position_inertia_matrix);
    applyInfinity(_joint_position_inertia_term);
    applyInfinity(_joint_acceleration_inertia_term);
    applyInfinity(_joint_position_velocity_product);
    applyInfinity(_joint_velocity_velocity_product);
    applyInfinity(_joint_position_gravity);
}

}  // namespace robot_dynamic
}  // namespace mhp_robot
