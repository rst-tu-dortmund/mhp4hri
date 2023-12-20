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

#ifndef ROBOT_DYNAMIC_H
#define ROBOT_DYNAMIC_H

#include <mhp_robot/robot_misc/robot_utility.h>
#include <kdl/chaindynparam.hpp>
#include <kdl/tree.hpp>

namespace mhp_robot {
namespace robot_dynamic {

class RobotDynamic
{
 public:
    using Ptr  = std::shared_ptr<RobotDynamic>;
    using UPtr = std::unique_ptr<RobotDynamic>;

    RobotDynamic(robot_misc::RobotUtility::UPtr robot_utility);
    RobotDynamic(const std::string& urdf_param, const std::string& first_frame, const std::string& last_frame);

    RobotDynamic(const RobotDynamic&)            = delete;
    RobotDynamic(RobotDynamic&&)                 = default;
    RobotDynamic& operator=(const RobotDynamic&) = delete;
    RobotDynamic& operator=(RobotDynamic&&)      = delete;
    virtual ~RobotDynamic() {}

    virtual UPtr createUniqueInstance() const;
    virtual Ptr createSharedInstance() const;

    const robot_misc::RobotUtility& getRobotUtility() const;

    void setJointPosition(const Eigen::Ref<const Eigen::VectorXd>& joint_position);
    void setJointVelocity(const Eigen::Ref<const Eigen::VectorXd>& joint_velocity);
    void setJointAcceleration(const Eigen::Ref<const Eigen::VectorXd>& joint_acceleration);
    void setJointState(const Eigen::Ref<const Eigen::VectorXd>& joint_position, const Eigen::Ref<const Eigen::VectorXd>& joint_velocity,
                       const Eigen::Ref<const Eigen::VectorXd>& joint_acceleration);

    const Eigen::Ref<const Eigen::MatrixXd> getInertiaMatrix();
    const Eigen::Ref<const Eigen::MatrixXd> getInertiaMatrix(const Eigen::Ref<const Eigen::VectorXd>& joint_position);

    const Eigen::Ref<const Eigen::VectorXd> getInertiaTerm();
    const Eigen::Ref<const Eigen::VectorXd> getInertiaTerm(const Eigen::Ref<const Eigen::VectorXd>& joint_position,
                                                           const Eigen::Ref<const Eigen::VectorXd>& joint_acceleration);

    const Eigen::Ref<const Eigen::VectorXd> getVelocityProductTerm();
    const Eigen::Ref<const Eigen::VectorXd> getVelocityProductTerm(const Eigen::Ref<const Eigen::VectorXd>& joint_position,
                                                                   const Eigen::Ref<const Eigen::VectorXd>& joint_velocity);

    const Eigen::Ref<const Eigen::VectorXd> getGravityTerm();
    const Eigen::Ref<const Eigen::VectorXd> getGravityTerm(const Eigen::Ref<const Eigen::VectorXd>& joint_position);

 protected:
    using RobotUtility = robot_misc::RobotUtility;

    RobotUtility::UPtr _robot_utility;

    int _joint_count = 0;

    KDL::JntArray _position_kdl;
    KDL::JntArray _velocity_kdl;
    Eigen::VectorXd _joint_acceleration;
    Eigen::VectorXd _inertia_term;
    KDL::JntSpaceInertiaMatrix _inertia_kdl;
    KDL::JntArray _velocity_product_kdl;
    KDL::JntArray _gravity_kdl;

    virtual void calculateInertiaMatrix();
    virtual void calculateInertiaTerm();
    virtual void calculateVelocityProductTerm();
    virtual void calculateGravityTerm();

 private:
    KDL::ChainDynParam _dynamic_kdl;

    Eigen::VectorXd _joint_position_inertia_matrix;
    Eigen::VectorXd _joint_position_inertia_term;
    Eigen::VectorXd _joint_acceleration_inertia_term;
    Eigen::VectorXd _joint_position_velocity_product;
    Eigen::VectorXd _joint_velocity_velocity_product;
    Eigen::VectorXd _joint_position_gravity;

    KDL::ChainDynParam getDynamicKDL(const robot_description::RobotDescription& robot_description) const;
    void applyInfinity(Eigen::Ref<Eigen::VectorXd> vector) const;
    void initialize();
};

}  // namespace robot_dynamic
}  // namespace mhp_robot

#endif  // ROBOT_DYNAMIC_H
