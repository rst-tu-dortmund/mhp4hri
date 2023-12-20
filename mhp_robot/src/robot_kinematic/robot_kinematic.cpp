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

#include <assert.h>
#include <math.h>
#include <mhp_robot/robot_kinematic/robot_kinematic.h>
#include <ros/io.h>
#include <iostream>

namespace mhp_robot {
namespace robot_kinematic {

RobotKinematic::RobotKinematic(RobotUtility::UPtr robot_utility) : _robot_utility(std::move(robot_utility)) { initialize(); }

RobotKinematic::RobotKinematic(const std::string& urdf_param, const std::string& first_frame, const std::string& last_frame)
    : _robot_utility(std::make_unique<RobotUtility>(urdf_param, first_frame, last_frame))
{
    initialize();
}

RobotKinematic::UPtr RobotKinematic::createUniqueInstance() const { return std::make_unique<RobotKinematic>(_robot_utility->createUniqueInstance()); }

RobotKinematic::Ptr RobotKinematic::createSharedInstance() const { return std::make_shared<RobotKinematic>(_robot_utility->createUniqueInstance()); }

const robot_misc::RobotUtility& RobotKinematic::getRobotUtility() const { return *_robot_utility; }

void RobotKinematic::setJointState(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& q)
{
    assert(_initialized);
    assert(_joint_state.size() == q.size());

    if (_joint_state.isApprox(q)) return;

    if (_joint_state.rows() != q.rows() || _joint_state.cols() != q.cols())
    {
        ROS_ERROR("RobotKinematic: Wrong joint dimension.");
        return;
    }

    _joint_state = q;
}

const std::vector<Eigen::Matrix<double, 4, 4>>& RobotKinematic::getForwardKinematicChain()
{
    calculateKinematicChain(_chained_transformations);
    return _chained_transformations;
}

const std::vector<Eigen::Matrix<double, 4, 4>>& RobotKinematic::getForwardKinematicChain(
    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& q)
{
    setJointState(q);
    return getForwardKinematicChain();
}

const std::vector<Eigen::Matrix<double, 4, 4>>& RobotKinematic::getForwardKinematicTransformations()
{
    calculateKinematicTransformations(_transformations);
    return _transformations;
}

const std::vector<Eigen::Matrix<double, 4, 4>>& RobotKinematic::getForwardKinematicTransformations(
    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& q)
{
    setJointState(q);
    return getForwardKinematicTransformations();
}

const Eigen::Ref<const Eigen::Matrix<double, 4, 4>> RobotKinematic::getEndEffectorMatrix()
{
    calculateEndEffectorMatrix(_ee_transformation);
    return _ee_transformation;
}

const Eigen::Ref<const Eigen::Matrix<double, 4, 4>> RobotKinematic::getEndEffectorMatrix(
    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& q)
{
    setJointState(q);
    return getEndEffectorMatrix();
}

const Eigen::Ref<const Eigen::Matrix<double, 6, Eigen::Dynamic>> RobotKinematic::getGeometricJacobian()
{
    calculateGeometricJacobian(_jacobian);
    return _jacobian;
}

const Eigen::Ref<const Eigen::Matrix<double, 6, Eigen::Dynamic>> RobotKinematic::getGeometricJacobian(
    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& q)
{
    setJointState(q);
    return getGeometricJacobian();
}

const Eigen::Ref<const Eigen::Matrix<double, 6, Eigen::Dynamic>> RobotKinematic::getPartialGeometricJacobian(
    const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& q, int skip_range, int end_id,
    const Eigen::Ref<const Eigen::Vector3d>& end_position)
{
    setJointState(q);
    return getPartialGeometricJacobian(skip_range, end_id, end_position);
}

const Eigen::Ref<const Eigen::Matrix<double, 6, Eigen::Dynamic>> RobotKinematic::getPartialGeometricJacobian(
    int skip_range, int end_id, const Eigen::Ref<const Eigen::Vector3d>& end_position)
{
    calculatePartialGeometricJacobian(skip_range, end_id, end_position, _partial_jacobian);
    return _partial_jacobian;
}

void RobotKinematic::calculateKinematicTransformations(std::vector<Eigen::Matrix<double, 4, 4>>& transformations)
{
    if (_joint_state.isApprox(_joint_state_transformations)) return;

    int k = 0;
    for (int i = 0; i < (int)transformations.size(); ++i)
    {
        const robot_misc::Joint& joint = _segment_structs[i].joint;

        if (joint.fixed)
        {
            continue;
        }
        else
        {
            // only update rotations as positions do not change for revolute joints
            if (joint.axis[0])
            {
                robot_misc::Common::rotxyz(joint.Rx + joint.offset + _joint_state(k), joint.Ry, joint.Rz, transformations[i].block<3, 3>(0, 0));
            }
            else if (joint.axis[1])
            {
                robot_misc::Common::rotxyz(joint.Rx, joint.Ry + joint.offset + _joint_state(k), joint.Rz, transformations[i].block<3, 3>(0, 0));
            }
            else if (joint.axis[2])
            {
                robot_misc::Common::rotxyz(joint.Rx, joint.Ry, joint.Rz + joint.offset + _joint_state(k), transformations[i].block<3, 3>(0, 0));
            }
            ++k;
        }
    }

    _joint_state_transformations = _joint_state;
}

void RobotKinematic::initKinematicTransformations(std::vector<Eigen::Matrix<double, 4, 4>>& transformations)
{
    int k = 0;
    for (int i = 0; i < (int)transformations.size(); ++i)
    {
        const robot_misc::Joint& joint = _segment_structs[i].joint;

        if (joint.fixed)
        {
            robot_misc::Common::rotxyz(joint.Rx, joint.Ry, joint.Rz, transformations[i].block<3, 3>(0, 0));
            transformations[i].block<3, 1>(0, 3) << joint.tx, joint.ty, joint.tz;
        }
        else
        {
            if (joint.axis[0])
            {
                robot_misc::Common::rotxyz(joint.Rx + joint.offset + _joint_state(k), joint.Ry, joint.Rz, transformations[i].block<3, 3>(0, 0));
            }
            else if (joint.axis[1])
            {
                robot_misc::Common::rotxyz(joint.Rx, joint.Ry + joint.offset + _joint_state(k), joint.Rz, transformations[i].block<3, 3>(0, 0));
            }
            else if (joint.axis[2])
            {
                robot_misc::Common::rotxyz(joint.Rx, joint.Ry, joint.Rz + joint.offset + _joint_state(k), transformations[i].block<3, 3>(0, 0));
            }

            transformations[i].block<3, 1>(0, 3) << joint.tx, joint.ty, joint.tz;
            ++k;
        }
    }
}

void RobotKinematic::initKinematicChain(std::vector<Eigen::Matrix<double, 4, 4>>& chain)
{
    chain[0] = _transformations[0];
    for (int i = 1; i < (int)_transformations.size(); ++i)
    {
        chain[i].noalias() = chain[i - 1] * _transformations[i];
    }

    _ee_transformation = chain.back();
}

void RobotKinematic::initGeometricJacobian(Eigen::Ref<Eigen::Matrix<double, 6, Eigen::Dynamic>> jacobian)
{
    if (!_joint_state.isApprox(_joint_state_chain)) calculateKinematicChain(_chained_transformations);

    Eigen::Vector3d ee_position, axis;
    ee_position = _chained_transformations.back().block<3, 1>(0, 3);

    int k = 0;
    for (int i = 0; i < (int)_transformations.size(); ++i)
    {
        const robot_misc::Joint& joint = _segment_structs[i].joint;

        if (!joint.fixed)
        {

            axis.noalias() = _chained_transformations[i].block<3, 1>(0, 0) * joint.axis[0] +
                             _chained_transformations[i].block<3, 1>(0, 1) * joint.axis[1] +
                             _chained_transformations[i].block<3, 1>(0, 2) * joint.axis[2];

            jacobian.block<3, 1>(0, k) = axis.cross(ee_position - _chained_transformations[i].block<3, 1>(0, 3));  // J_Li
            jacobian.block<3, 1>(3, k) = axis;                                                                     // J_Ai
            ++k;
        }
    }
}

void RobotKinematic::initialize()
{
    // get kinematic structure
    _segment_structs = _robot_utility->getRobotDescription().getKinematicStructure();

    _n_dynamic_joints            = _robot_utility->getJointsCount();
    _joint_state                 = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(_n_dynamic_joints, 1);
    _joint_state_chain           = _joint_state;
    _joint_state_transformations = _joint_state;
    _joint_state_ee              = _joint_state;
    _joint_state_jacobian        = _joint_state;
    _jacobian                    = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, _n_dynamic_joints);
    _partial_jacobian            = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, _n_dynamic_joints);
    _ee_transformation           = Eigen::Matrix<double, 4, 4>::Identity();

    // initialize forward kinematics
    _chained_transformations = std::vector<Eigen::Matrix<double, 4, 4>>(_segment_structs.size(), Eigen::Matrix<double, 4, 4>::Identity());
    _transformations         = std::vector<Eigen::Matrix<double, 4, 4>>(_segment_structs.size(), Eigen::Matrix<double, 4, 4>::Identity());

    // initialize for default q
    initKinematicTransformations(_transformations);
    initKinematicChain(_chained_transformations);
    initGeometricJacobian(_jacobian);

    _initialized = true;
}

void RobotKinematic::calculateKinematicChain(std::vector<Eigen::Matrix<double, 4, 4>>& chain)
{
    if (_joint_state.isApprox(_joint_state_chain)) return;

    if (!_joint_state.isApprox(_joint_state_transformations)) calculateKinematicTransformations(_transformations);

    chain[0] = _transformations[0];
    for (int i = 1; i < (int)_transformations.size(); ++i)
    {
        chain[i].noalias() = chain[i - 1] * _transformations[i];
    }

    _joint_state_chain = _joint_state;
}

void RobotKinematic::calculateGeometricJacobian(Eigen::Ref<Eigen::Matrix<double, 6, Eigen::Dynamic>> jacobian)
{
    if (_joint_state.isApprox(_joint_state_jacobian)) return;

    if (!_joint_state.isApprox(_joint_state_chain)) calculateKinematicChain(_chained_transformations);

    int k = 0;
    jacobian.setZero();

    Eigen::Vector3d ee_position, axis;
    ee_position = _chained_transformations.back().block<3, 1>(0, 3);  // End effector position

    for (int i = 0; i < (int)_transformations.size(); ++i)
    {
        const robot_misc::Joint& joint = _segment_structs[i].joint;

        if (!joint.fixed)
        {
            axis.noalias() = _chained_transformations[i].block<3, 1>(0, 0) * joint.axis[0] +
                             _chained_transformations[i].block<3, 1>(0, 1) * joint.axis[1] +
                             _chained_transformations[i].block<3, 1>(0, 2) * joint.axis[2];

            jacobian.block<3, 1>(0, k) = axis.cross(ee_position - _chained_transformations[i].block<3, 1>(0, 3));  // J_Li
            jacobian.block<3, 1>(3, k) = axis;                                                                     // J_Ai
            ++k;
        }
    }

    _joint_state_jacobian = _joint_state;
}

void RobotKinematic::calculatePartialGeometricJacobian(int skip_range, int end_id, const Eigen::Ref<const Eigen::Vector3d>& end_position,
                                                       Eigen::Ref<Eigen::Matrix<double, 6, Eigen::Dynamic>> jacobian)
{
    if (skip_range >= _n_dynamic_joints || skip_range >= end_id) return;

    if (!_joint_state.isApprox(_joint_state_chain)) calculateKinematicChain(_chained_transformations);

    int k            = 0;
    int skip_counter = 0;

    Eigen::Vector3d axis;
    jacobian.setZero();

    for (int i = 0; i < (int)_transformations.size() && i <= end_id; ++i)
    {
        const robot_misc::Joint& joint = _segment_structs[i].joint;

        if (!joint.fixed)
        {
            if (skip_counter < skip_range)
            {
                ++skip_counter;
            }
            else
            {
                axis.noalias() = _chained_transformations[i].block<3, 1>(0, 0) * joint.axis[0] +
                                 _chained_transformations[i].block<3, 1>(0, 1) * joint.axis[1] +
                                 _chained_transformations[i].block<3, 1>(0, 2) * joint.axis[2];

                jacobian.block<3, 1>(0, k) = axis.cross(end_position - _chained_transformations[i].block<3, 1>(0, 3));  // J_Li
                jacobian.block<3, 1>(3, k) = axis;                                                                      // J_Ai
            }
            ++k;
        }
    }
}

void RobotKinematic::calculateEndEffectorMatrix(Eigen::Ref<Eigen::Matrix<double, 4, 4>> ee)
{
    calculateKinematicChain(_chained_transformations);
    ee = _chained_transformations.back();
}

}  // namespace robot_kinematic
}  // namespace mhp_robot
