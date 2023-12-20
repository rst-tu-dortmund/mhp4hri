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

#ifndef ROBOT_KINEMATIC_H
#define ROBOT_KINEMATIC_H

#include <mhp_robot/robot_description/robot_description.h>
#include <mhp_robot/robot_misc/common.h>
#include <mhp_robot/robot_misc/robot_utility.h>

namespace mhp_robot {
namespace robot_kinematic {

class RobotKinematic
{
 public:
    using Ptr  = std::shared_ptr<RobotKinematic>;
    using UPtr = std::unique_ptr<RobotKinematic>;

    RobotKinematic(robot_misc::RobotUtility::UPtr robot_utility);
    RobotKinematic(const std::string& urdf_param, const std::string& first_frame, const std::string& last_frame);

    RobotKinematic(const RobotKinematic&)            = delete;
    RobotKinematic(RobotKinematic&&)                 = default;
    RobotKinematic& operator=(const RobotKinematic&) = delete;
    RobotKinematic& operator=(RobotKinematic&&)      = default;
    virtual ~RobotKinematic() {}

    virtual UPtr createUniqueInstance() const;
    virtual Ptr createSharedInstance() const;

    const robot_misc::RobotUtility& getRobotUtility() const;

    void setJointState(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& q);

    const std::vector<Eigen::Matrix<double, 4, 4>>& getForwardKinematicChain();
    const std::vector<Eigen::Matrix<double, 4, 4>>& getForwardKinematicChain(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& q);

    const std::vector<Eigen::Matrix<double, 4, 4>>& getForwardKinematicTransformations();
    const std::vector<Eigen::Matrix<double, 4, 4>>& getForwardKinematicTransformations(
        const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& q);

    const Eigen::Ref<const Eigen::Matrix<double, 4, 4>> getEndEffectorMatrix();
    const Eigen::Ref<const Eigen::Matrix<double, 4, 4>> getEndEffectorMatrix(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& q);

    const Eigen::Ref<const Eigen::Matrix<double, 6, Eigen::Dynamic>> getGeometricJacobian();
    const Eigen::Ref<const Eigen::Matrix<double, 6, Eigen::Dynamic>> getGeometricJacobian(
        const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& q);

    const Eigen::Ref<const Eigen::Matrix<double, 6, Eigen::Dynamic>> getPartialGeometricJacobian(
        int skip_range, int end_id, const Eigen::Ref<const Eigen::Vector3d>& end_position);
    const Eigen::Ref<const Eigen::Matrix<double, 6, Eigen::Dynamic>> getPartialGeometricJacobian(
        const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& q, int skip_range, int end_id,
        const Eigen::Ref<const Eigen::Vector3d>& end_position);

 protected:
    using RobotUtility = robot_misc::RobotUtility;

    RobotUtility::UPtr _robot_utility;

    std::vector<robot_misc::Segment> _segment_structs;
    std::vector<Eigen::Matrix<double, 4, 4>> _chained_transformations;
    std::vector<Eigen::Matrix<double, 4, 4>> _transformations;
    Eigen::Matrix<double, 6, Eigen::Dynamic> _jacobian;
    Eigen::Matrix<double, 6, Eigen::Dynamic> _partial_jacobian;
    Eigen::Matrix<double, 4, 4> _ee_transformation;

    Eigen::Matrix<double, Eigen::Dynamic, 1> _joint_state;
    Eigen::Matrix<double, Eigen::Dynamic, 1> _joint_state_ee;
    Eigen::Matrix<double, Eigen::Dynamic, 1> _joint_state_transformations;
    Eigen::Matrix<double, Eigen::Dynamic, 1> _joint_state_chain;
    Eigen::Matrix<double, Eigen::Dynamic, 1> _joint_state_jacobian;

    virtual void calculateKinematicTransformations(std::vector<Eigen::Matrix<double, 4, 4>>& transformations);
    virtual void calculateKinematicChain(std::vector<Eigen::Matrix<double, 4, 4>>& chain);
    virtual void calculateGeometricJacobian(Eigen::Ref<Eigen::Matrix<double, 6, Eigen::Dynamic>> jacobian);
    virtual void calculatePartialGeometricJacobian(int skip_range, int end_id, const Eigen::Ref<const Eigen::Vector3d>& end_position,
                                                   Eigen::Ref<Eigen::Matrix<double, 6, Eigen::Dynamic>> jacobian);
    virtual void calculateEndEffectorMatrix(Eigen::Ref<Eigen::Matrix<double, 4, 4>> ee);

 private:
    bool _initialized     = false;
    int _n_dynamic_joints = 0;

    void initKinematicTransformations(std::vector<Eigen::Matrix<double, 4, 4>>& transformations);
    void initKinematicChain(std::vector<Eigen::Matrix<double, 4, 4>>& chain);
    void initGeometricJacobian(Eigen::Ref<Eigen::Matrix<double, 6, Eigen::Dynamic>> jacobian);
    void initialize();
};

}  // namespace robot_kinematic
}  // namespace mhp_robot

#endif  // ROBOT_KINEMATIC_H
