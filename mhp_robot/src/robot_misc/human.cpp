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

#include <mhp_robot/robot_misc/common.h>
#include <mhp_robot/robot_misc/human.h>

#include <tf/transform_listener.h>

#include <math.h>

namespace mhp_robot {
namespace robot_misc {

constexpr const int Human::_body_part_count;

Human::Human(int ID, std::string HumanName, std::vector<double> lengthLinks, std::vector<double> radiusLinks, std::vector<double> angles,
             Eigen::Vector3d foot_print)
{
    _id   = ID;
    _name = HumanName;
    std::vector<std::string> body_part_names{"Hip", "Neck", "Head", "LUArm", "LFArm", "LHand", "RUArm", "RFArm", "RHand", "Leg"};
    unsigned long cnt = 0;

    for (std::size_t part_it = 0; part_it < body_part_names.size(); ++part_it)
    {
        if (_length.find(body_part_names[part_it] + "_length") == _length.end())
        {
            _length.find(body_part_names[part_it] + "_length")->second = 0.0;
        }
        else
        {
            _length.find(body_part_names[part_it] + "_length")->second = lengthLinks[cnt];
            cnt += 1;
        }
    }
    cnt = 0;
    for (std::size_t part_it = 0; part_it < body_part_names.size(); ++part_it)
    {
        if (_radius.find(body_part_names[part_it] + "_radius") == _radius.end())
        {
            _radius.find(body_part_names[part_it] + "_length")->second = 0.0;
        }
        else
        {
            _radius.find(body_part_names[part_it] + "_radius")->second = radiusLinks[cnt];
            cnt += 1;
        }
    }

    _joint_angles = angles;
    _foot_print   = foot_print;

    for (std::size_t part_it = 0; part_it < body_part_names.size(); ++part_it)
    {
        robot_misc::Obstacle bodyPartObs;

        bodyPartObs.id       = int(part_it);
        bodyPartObs.group_id = _id;
        bodyPartObs.name     = body_part_names[part_it];
        if (body_part_names[part_it] == "Hip")
        {
            bodyPartObs.bounding_box.length_x = std::max(_length.at("Hip_length"), 0.0);
            bodyPartObs.bounding_box.length_y = 0.0;
            //            bodyPartObs.bounding_box.radius   = std::max(_radius.at("Hip_radius"), 0.0);
            bodyPartObs.bounding_box.radius = 0.0;
            bodyPartObs.bounding_box.T      = Common::rotz(-M_PI / 2);
            //                bodyPartObs.state._poses[0].block<4, 1>(0, 3) = body_poses.at(body_part_names[part_it]).block<4, 1>(0, 3);
            //                bodyPartObs.state._poses[0].block<3, 3>(0, 0) = body_poses.at(body_part_names[part_it]).block<3, 3>(0, 0);
            bodyPartObs.bounding_box.type = BoundingBoxType::SPHERE;
        }
        if (body_part_names[part_it] == "Neck")
        {
            bodyPartObs.bounding_box.length_x = std::max(_length.at("Neck_length") - 1 * _radius.at("Neck_radius"), 0.0);
            bodyPartObs.bounding_box.length_y = 0.0;
            bodyPartObs.bounding_box.radius   = std::max(_radius.at("Neck_radius"), 0.0);
            bodyPartObs.bounding_box.T        = Common::rotz(-M_PI / 2);

            //                bodyPartObs.state._poses[0].block<4, 1>(0, 3) = body_poses.at(body_part_names[part_it]).block<4, 1>(0, 3);
            //                bodyPartObs.state._poses[0].block<3, 3>(0, 0) = body_poses.at(body_part_names[part_it]).block<3, 3>(0, 0);
            //                std::cout << "Neck pose in Constructor: " << bodyPartObs.state._poses[0].block<3, 3>(0, 0) << std::endl;
            bodyPartObs.bounding_box.type = BoundingBoxType::CYLINDER;
        }
        if (body_part_names[part_it] == "Head")
        {
            bodyPartObs.bounding_box.length_x = std::max(_length.at("Head_length"), 0.0);
            bodyPartObs.bounding_box.length_y = 0.0;
            bodyPartObs.bounding_box.radius   = std::max(_radius.at("Head_radius"), 0.0);
            Eigen::Matrix4d boxTranslation{Eigen::Matrix4d::Identity()};
            boxTranslation(1, 3)       = _radius.at("Hip_radius");
            bodyPartObs.bounding_box.T = boxTranslation;
            //                bodyPartObs.state._poses[0].block<4, 1>(0, 3) = body_poses.at(body_part_names[part_it]).block<4, 1>(0, 3);
            //                bodyPartObs.state._poses[0].block<3, 3>(0, 0) = body_poses.at(body_part_names[part_it]).block<3, 3>(0, 0);
            bodyPartObs.bounding_box.type = BoundingBoxType::SPHERE;
        }
        if (body_part_names[part_it] == "LUArm")
        {
            bodyPartObs.bounding_box.length_x = std::max(_length.at("LUArm_length") - 2 * _radius.at("LUArm_radius"), 0.0);
            bodyPartObs.bounding_box.length_y = 0.0;
            bodyPartObs.bounding_box.radius   = std::max(_radius.at("LUArm_radius"), 0.0);
            bodyPartObs.bounding_box.type     = BoundingBoxType::CYLINDER;
        }
        if (body_part_names[part_it] == "LFArm")
        {
            bodyPartObs.bounding_box.length_x = std::max(_length.at("LFArm_length") - 2 * _radius.at("LFArm_radius"), 0.0);
            bodyPartObs.bounding_box.length_y = 0.0;
            bodyPartObs.bounding_box.radius   = std::max(_radius.at("LFArm_radius"), 0.0);
            bodyPartObs.bounding_box.type     = BoundingBoxType::CYLINDER;
        }
        if (body_part_names[part_it] == "LHand")
        {
            //            std::cout << "Teste Lhand \n" << std::endl;
            bodyPartObs.bounding_box.length_x = 0.0;
            bodyPartObs.bounding_box.length_y = 0.0;
            bodyPartObs.bounding_box.radius   = 0.0;
            bodyPartObs.bounding_box.type     = BoundingBoxType::SPHERE;
        }
        if (body_part_names[part_it] == "RUArm")
        {
            bodyPartObs.bounding_box.length_x = std::max(_length.at("RUArm_length") - 2 * _radius.at("RUArm_radius"), 0.0);
            bodyPartObs.bounding_box.length_y = 0.0;
            bodyPartObs.bounding_box.radius   = std::max(_radius.at("RUArm_radius"), 0.0);
            bodyPartObs.bounding_box.T        = Common::roty(M_PI);

            bodyPartObs.bounding_box.type = BoundingBoxType::CYLINDER;
        }
        if (body_part_names[part_it] == "RFArm")
        {
            bodyPartObs.bounding_box.length_x = std::max(_length.at("RFArm_length") - 2 * _radius.at("RFArm_radius"), 0.0);
            bodyPartObs.bounding_box.length_y = 0.0;
            bodyPartObs.bounding_box.radius   = std::max(_radius.at("RFArm_radius"), 0.0);
            bodyPartObs.bounding_box.T        = Common::roty(M_PI);

            bodyPartObs.bounding_box.type = BoundingBoxType::CYLINDER;
        }
        if (body_part_names[part_it] == "RHand")
        {
            bodyPartObs.bounding_box.length_x = 0.0;
            bodyPartObs.bounding_box.length_y = 0.0;
            bodyPartObs.bounding_box.radius   = 0.0;
            bodyPartObs.bounding_box.type     = BoundingBoxType::SPHERE;
        }

        if (body_part_names[part_it] == "Leg")
        {
            bodyPartObs.bounding_box.length_x = std::max(_length.at("Leg_length") - 1.0 * _radius.at("Leg_radius"), 0.0);
            bodyPartObs.bounding_box.length_y = 0.0;
            bodyPartObs.bounding_box.radius   = std::max(_radius.at("Leg_radius"), 0.0);
            bodyPartObs.bounding_box.T        = Common::rotz(-M_PI / 2);
            bodyPartObs.bounding_box.type     = BoundingBoxType::CYLINDER;
        }
        _body_parts.insert({body_part_names[part_it], bodyPartObs});
    }

    _lower_angle_limit << -0.7, -0.5, -1.5708, 0, -3.1416, 0, -1.5708, 0, -3.1416, 0, 0;
    _upper_angle_limit << 1.8, 1.44, 3.1416, 3.1416, 3.1416, 2.6, 3.1416, 3.1416, 3.1416, 2.6, 0;

    updatePoses(_foot_print);
}

const Obstacle& Human::getBodyPart(const std::string& name) const
{
    auto it = _body_parts.find(name);
    return it->second;
}

void Human::inverseKinematic()
{
    // Get current pose data from the body parts
    Eigen::Matrix4d Hip = _body_parts.find("Hip")->second.state._poses[0];

    Eigen::Matrix4d Neck  = _body_parts.find("Neck")->second.state._poses[0];
    Eigen::Matrix4d Head  = _body_parts.find("Head")->second.state._poses[0];
    Eigen::Matrix4d RUArm = _body_parts.find("RUArm")->second.state._poses[0];
    Eigen::Matrix4d RFArm = _body_parts.find("RFArm")->second.state._poses[0];

    Eigen::Matrix4d RHand = _body_parts.find("RHand")->second.state._poses[0];
    Eigen::Matrix4d LUArm = _body_parts.find("LUArm")->second.state._poses[0];
    Eigen::Matrix4d LFArm = _body_parts.find("LFArm")->second.state._poses[0];
    Eigen::Matrix4d LHand = _body_parts.find("LHand")->second.state._poses[0];

    // Get single joint angles with corresponding helper function
    calculateHipAngle(Hip, Neck);

    calculateElbowAngle(LHand, LUArm, true);
    calculateElbowAngle(RHand, RUArm, false);

    calculateHeadAngle(Head, Neck);

    calculateShoulderAngle(LUArm, LFArm, LHand, true);
    calculateShoulderAngle(RUArm, RFArm, RHand, false);

    // Set leg Angle default to zero
    _joint_angles[10] = 0;
}

void Human::inverseKinematicTf()
{
    // get current body parts from the pose data
    // Difference to previous is the additional mocap rotation for all frames
    Eigen::Matrix4d Hip = _body_parts.find("Hip")->second.state._poses[0];

    Eigen::Matrix4d Neck  = _body_parts.find("Neck")->second.state._poses[0];
    Eigen::Matrix4d Head  = _body_parts.find("Head")->second.state._poses[0];
    Eigen::Matrix4d RUArm = _body_parts.find("RUArm")->second.state._poses[0];
    Eigen::Matrix4d RFArm = _body_parts.find("RFArm")->second.state._poses[0];

    Eigen::Matrix4d RHand = _body_parts.find("RHand")->second.state._poses[0];
    Eigen::Matrix4d LUArm = _body_parts.find("LUArm")->second.state._poses[0];
    Eigen::Matrix4d LFArm = _body_parts.find("LFArm")->second.state._poses[0];
    Eigen::Matrix4d LHand = _body_parts.find("LHand")->second.state._poses[0];

    // Call Helper functions for joint angles
    calculateHipAngleTf(Hip, Neck);

    calculateElbowAngleTf(LHand, LUArm, true);
    calculateElbowAngleTf(RHand, RUArm, false);

    calculateHeadAngleTf(Head, Neck);

    calculateShoulderAngleTf(LUArm, LFArm, LHand, true);
    calculateShoulderAngleTf(RUArm, RFArm, RHand, false);

    // Set leg Angle default to zero
    _joint_angles[10] = 0;
}

void Human::initializeBodyLength()
{
    // calculate body part lengths
    // required at initilization and recursive if we use real data due to
    //  little shift that lead to numerical instability in the IK then
    Eigen::Matrix4d Hip      = _body_parts.find("Hip")->second.state._poses[0];
    _length.at("Hip_length") = Hip(2, 3);

    Eigen::Matrix4d Neck      = _body_parts.find("Neck")->second.state._poses[0];
    _length.at("Neck_length") = (Hip.block<3, 1>(0, 3) - Neck.block<3, 1>(0, 3)).norm();

    Eigen::Matrix4d LUArm      = _body_parts.find("LUArm")->second.state._poses[0];
    Eigen::Matrix4d LFArm      = _body_parts.find("LFArm")->second.state._poses[0];
    Eigen::Matrix4d LHand      = _body_parts.find("LHand")->second.state._poses[0];
    Eigen::Matrix4d RUArm      = _body_parts.find("RUArm")->second.state._poses[0];
    Eigen::Matrix4d RFArm      = _body_parts.find("RFArm")->second.state._poses[0];
    Eigen::Matrix4d RHand      = _body_parts.find("RHand")->second.state._poses[0];
    _length.at("RUArm_length") = (RUArm.block<3, 1>(0, 3) - RFArm.block<3, 1>(0, 3)).norm();
    _length.at("RFArm_length") = (RFArm.block<3, 1>(0, 3) - RHand.block<3, 1>(0, 3)).norm();
    _length.at("LUArm_length") = (LUArm.block<3, 1>(0, 3) - LFArm.block<3, 1>(0, 3)).norm();
    _length.at("LFArm_length") = (LFArm.block<3, 1>(0, 3) - LHand.block<3, 1>(0, 3)).norm();

    Eigen::Matrix4d Head      = _body_parts.find("Head")->second.state._poses[0];
    _radius.at("Head_radius") = (Neck.block<3, 1>(0, 3) - Head.block<3, 1>(0, 3)).norm();
}

void Human::updatePoses(const Eigen::Ref<const Eigen::Vector3d>& foot_print)
{
    // function for updating the current poses saved inside the human class elements
    _foot_print                           = foot_print;
    _body_parts.at("Hip").state._poses[0] = calculateHipPose(foot_print);
    _body_parts.at("Leg").state._poses[0] = _body_parts.at("Hip").state._poses[0];

    _body_parts.at("Neck").state._poses[0] = calculateNeckpose(_body_parts.at("Hip").state._poses[0], _joint_angles[0]);

    _body_parts.at("Head").state._poses[0] = calculateHeadPose(_body_parts.at("Neck").state._poses[0], _joint_angles[1]);

    _body_parts.at("LUArm").state._poses[0] = calculateLeftShoulderPose(
        _body_parts.at("Hip").state._poses[0], Eigen::Vector3d{_joint_angles[2], _joint_angles[3], _joint_angles[4]}, _joint_angles[0]);

    _body_parts.at("LFArm").state._poses[0] = calculateLeftElbowPose(_body_parts.at("LUArm").state._poses[0], _joint_angles[5]);

    _body_parts.at("LHand").state._poses[0] = calculateLeftHandPose(_body_parts.at("LFArm").state._poses[0]);

    _body_parts.at("RUArm").state._poses[0] = calculateRightShoulderPose(
        _body_parts.at("Hip").state._poses[0], Eigen::Vector3d{_joint_angles[6], _joint_angles[7], _joint_angles[8]}, _joint_angles[0]);

    _body_parts.at("RFArm").state._poses[0] = calculateRightElbowPose(_body_parts.at("RUArm").state._poses[0], _joint_angles[9]);

    _body_parts.at("RHand").state._poses[0] = calculateRightHandPose(_body_parts.at("RFArm").state._poses[0]);
}

std::vector<Eigen::Matrix4d> Human::returnPoses(const Eigen::Ref<const Eigen::Vector3d>& foot_print, Eigen::VectorXd angles)
{
    // can return a vector of homogenous transformation matrices with the poses of all body parts
    std::vector<Eigen::Matrix4d> allPoses(9, Eigen::Matrix4d::Zero());
    allPoses[0] = calculateHipPose(foot_print);

    allPoses[1] = calculateNeckpose(allPoses[0], angles[0]);

    allPoses[2] = calculateHeadPose(allPoses[1], angles[1]);

    allPoses[3] = calculateLeftShoulderPose(allPoses[0], Eigen::Vector3d{angles[2], angles[3], angles[4]}, angles[0]);

    allPoses[4] = calculateLeftElbowPose(allPoses[3], angles[5]);

    allPoses[5] = calculateLeftHandPose(allPoses[4]);

    allPoses[6] = calculateRightShoulderPose(allPoses[0], Eigen::Vector3d{angles[6], angles[7], angles[8]}, angles[0]);

    allPoses[7] = calculateRightElbowPose(allPoses[6], angles[9]);

    allPoses[8] = calculateRightHandPose(allPoses[7]);

    return allPoses;
}

void Human::updateFutureHipPoses(const int futureStep, const int _N)
{
    size_t i = futureStep;
    // Use predicted foot prints to calculate future Hip poses
    if (_future_foot_prints.size() >= i)
    {
        if (_body_parts.at("Hip").state._poses.size() < _N)  // if currently is no hip pose available for all N extrapolation steps
        {
            _body_parts.at("Hip").state._poses.push_back(calculateHipPose(_future_foot_prints[i - 1]));

            _body_parts.at("Leg").state._poses.push_back(*_body_parts.at("Hip").state._poses.end());

            if (_body_parts.at("Hip").state._times.size() < _N)
            {
                _body_parts.at("Hip").state._times.push_back(i * _dt);
                _body_parts.at("Leg").state._times.push_back(i * _dt);
            }
        }
        else  // if already a pose is available we overwrite it
        {
            _body_parts.at("Hip").state._poses[i] = calculateHipPose(_future_foot_prints[i - 1]);
            _body_parts.at("Hip").state._times[i] = i * _dt;

            _body_parts.at("Leg").state._poses[i] = *_body_parts.at("Hip").state._poses.end();
            _body_parts.at("Leg").state._times[i] = i * _dt;
        }
    }
}

void Human::updateFuturePoses(const Eigen::Ref<Eigen::Vector<double, 11>> angles, int futureStep, int _N)
{
    size_t i = futureStep;
    // Hip Pose is not estiamted, because not possible from joint angles but if available use predicted foot prints
    if (_future_foot_prints.size() >= i)
    {
        //        std::cout << "Future foot prints available: \n"
        //        << calculateHipPose(_future_foot_prints[i - 1]) << "\nwith footprint: " << _future_foot_prints[i - 1] << "at i:" << i << std::endl;
        if (_body_parts.at("Hip").state._poses.size() < _N)
        {
            // std::cout << "Hip poses size: " << _body_parts.at("Hip").state._poses.size() << "and N: " << _N << "\n" << std::endl;
            _body_parts.at("Hip").state._poses.push_back(calculateHipPose(_future_foot_prints[i - 1]));
            _body_parts.at("Hip").state._times.push_back(i * _dt);

            _body_parts.at("Leg").state._poses.push_back(*_body_parts.at("Hip").state._poses.end());
            _body_parts.at("Leg").state._times.push_back(i * _dt);
        }
        else
        {
            //        std::cout << "Overwrite \n" << std::endl;
            _body_parts.at("Hip").state._poses[i] = calculateHipPose(_future_foot_prints[i - 1]);
            _body_parts.at("Hip").state._times[i] = i * _dt;

            _body_parts.at("Leg").state._poses[i] = _body_parts.at("Hip").state._poses[i];
            _body_parts.at("Leg").state._times[i] = i * _dt;
        }
    }
    else  // without future foot prints
    {
        std::cout << "Future foot prints not available at \n" << i << std::endl;
        if (_body_parts.at("Hip").state._poses.size() < _N)
        {
            _body_parts.at("Hip").state._poses.push_back(_body_parts.at("Hip").state._poses.back());
            _body_parts.at("Hip").state._times.push_back(i * _dt);
        }
        else
        {
            _body_parts.at("Hip").state._poses[i] = _body_parts.at("Hip").state._poses.back();
            _body_parts.at("Hip").state._times[i] = i * _dt;
        }
        if (_body_parts.at("Leg").state._poses.size() < _N)
        {
            _body_parts.at("Leg").state._poses.push_back(_body_parts.at("Leg").state._poses.back());
            _body_parts.at("Leg").state._times.push_back(i * _dt);
        }
        else
        {
            _body_parts.at("Leg").state._poses[i] = _body_parts.at("Leg").state._poses.back();
            _body_parts.at("Leg").state._times[i] = i * _dt;
        }
    }
    // Update future body part poses considering the given angles (function input)
    if (_body_parts.at("Neck").state._poses.size() < _N)
    {
        _body_parts.at("Neck").state._poses.push_back(calculateNeckpose(_body_parts.at("Hip").state._poses[i], angles[0]));
        _body_parts.at("Neck").state._times.push_back(i * _dt);
    }
    else
    {
        _body_parts.at("Neck").state._poses[i] = calculateNeckpose(_body_parts.at("Hip").state._poses[i], angles[0]);
        _body_parts.at("Neck").state._times[i] = i * _dt;
    }

    if (_body_parts.at("Head").state._poses.size() < _N)
    {
        _body_parts.at("Head").state._poses.push_back(calculateHeadPose(_body_parts.at("Neck").state._poses[i], angles[1]));
        _body_parts.at("Head").state._times.push_back(i * _dt);
    }
    else
    {
        _body_parts.at("Head").state._poses[i] = calculateHeadPose(_body_parts.at("Neck").state._poses[i], angles[1]);
        _body_parts.at("Head").state._times[i] = i * _dt;
    }

    Eigen::Vector3d leftShoulderAng(angles[2], angles[3], angles[4]);
    if (_body_parts.at("LUArm").state._poses.size() < _N)
    {
        _body_parts.at("LUArm").state._poses.push_back(calculateLeftShoulderPose(_body_parts.at("Hip").state._poses[i], leftShoulderAng, angles[0]));
        _body_parts.at("LUArm").state._times.push_back(i * _dt);
    }
    else
    {
        _body_parts.at("LUArm").state._poses[i] = calculateLeftShoulderPose(_body_parts.at("Hip").state._poses[i], leftShoulderAng, angles[0]);
        _body_parts.at("LUArm").state._times[i] = i * _dt;
    }

    if (_body_parts.at("LFArm").state._poses.size() < _N)
    {
        _body_parts.at("LFArm").state._poses.push_back(calculateLeftElbowPose(_body_parts.at("LUArm").state._poses[i], angles[5]));
        _body_parts.at("LFArm").state._times.push_back(i * _dt);
    }
    else
    {
        _body_parts.at("LFArm").state._poses[i] = calculateLeftElbowPose(_body_parts.at("LUArm").state._poses[i], angles[5]);
        _body_parts.at("LFArm").state._times[i] = i * _dt;
    }

    if (_body_parts.at("LHand").state._poses.size() < _N)
    {
        _body_parts.at("LHand").state._poses.push_back(calculateLeftHandPose(_body_parts.at("LFArm").state._poses[i]));
        _body_parts.at("LHand").state._times.push_back(i * _dt);
    }
    else
    {
        _body_parts.at("LHand").state._poses[i] = calculateLeftHandPose(_body_parts.at("LFArm").state._poses[i]);
        _body_parts.at("LHand").state._times[i] = i * _dt;
    }

    Eigen::Vector3d rightShoulderAng(angles[6], angles[7], angles[8]);
    if (_body_parts.at("RUArm").state._poses.size() < _N)
    {
        _body_parts.at("RUArm").state._poses.push_back(
            calculateRightShoulderPose(_body_parts.at("Hip").state._poses[i], rightShoulderAng, angles[0]));
        _body_parts.at("RUArm").state._times.push_back(i * _dt);
    }
    else
    {
        _body_parts.at("RUArm").state._poses[i] = calculateRightShoulderPose(_body_parts.at("Hip").state._poses[i], rightShoulderAng, angles[0]);
        _body_parts.at("RUArm").state._times[i] = i * _dt;
    }

    if (_body_parts.at("RFArm").state._poses.size() < _N)
    {
        _body_parts.at("RFArm").state._poses.push_back(calculateRightElbowPose(_body_parts.at("RUArm").state._poses[i], angles[9]));
        _body_parts.at("RFArm").state._times.push_back(i * _dt);
    }
    else
    {
        _body_parts.at("RFArm").state._poses[i] = calculateRightElbowPose(_body_parts.at("RUArm").state._poses[i], angles[9]);
        _body_parts.at("RFArm").state._times[i] = i * _dt;
    }

    if (_body_parts.at("RHand").state._poses.size() < _N)
    {
        _body_parts.at("RHand").state._poses.push_back(calculateRightHandPose(_body_parts.at("RFArm").state._poses[i]));
        _body_parts.at("RHand").state._times.push_back(i * _dt);
    }
    else
    {
        _body_parts.at("RHand").state._poses[i] = calculateRightHandPose(_body_parts.at("RFArm").state._poses[i]);
        _body_parts.at("RHand").state._times[i] = i * _dt;
    }
}

void Human::updatePosesFromTf(tf::TransformListener& listener)
{
    for (auto& bdy : _body_parts)
    {
        // Wait until transfomrations are avaialable
        std::string frame_name = "Heiko_upper_" + bdy.second.name + "_" + std::to_string(_id);
        listener.waitForTransform("world", frame_name, ros::Time(0), ros::Duration(10.0));

        if (listener.frameExists(frame_name))
        {
            // Update Poses of current body part with Transformation
            Eigen::Matrix4d T = Common::getTransformation("world", frame_name, listener);

            if (bdy.second.bounding_box.type == mhp_robot::robot_misc::BoundingBoxType::EBOX)
            {
                Eigen::Quaterniond quat(T.block<3, 3>(0, 0));
                Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);

                // Only update position and roation around z
                bdy.second.state._poses[0].block<3, 1>(0, 3) = T.block<3, 1>(0, 3);
                bdy.second.state._poses[0].block<3, 3>(0, 0) = Common::rotz(euler(2)).block<3, 3>(0, 0);
            }
            else
            {
                // Update full pose
                bdy.second.state._poses[0] = T;
            }
        }
        if (bdy.second.name == "Leg")
        {
            bdy.second.state._poses[0] = _body_parts.at("Hip").state._poses[0];
        }
    }
}

void Human::updateObstaclePose(Obstacle& obs, const std::string bodyPart, const Eigen::Ref<const Eigen::Vector<double, 11>> angles,
                               const size_t timeStep)
{
    if ("Neck" == bodyPart)
    {
        obs.state._poses[timeStep] = calculateNeckpose(calculateHipPose(_future_foot_prints.at(timeStep)), angles[0]);
    }
    else if ("Head" == bodyPart)
    {
        obs.state._poses[timeStep] = calculateHeadPose(calculateNeckpose(calculateHipPose(_future_foot_prints.at(timeStep)), angles[0]), angles[1]);
    }
    else if ("LUArm" == bodyPart)
    {
        obs.state._poses[timeStep] = calculateLeftShoulderPose(calculateHipPose(_future_foot_prints.at(timeStep)), angles.segment(2, 3), angles[0]);
    }
    else if ("LFArm" == bodyPart)
    {
        obs.state._poses[timeStep] = calculateLeftElbowPose(
            calculateLeftShoulderPose(calculateHipPose(_future_foot_prints.at(timeStep)), angles.segment(2, 3), angles[0]), angles[5]);
    }
    else if ("LHand" == bodyPart)
    {
        obs.state._poses[timeStep] = calculateLeftHandPose(calculateLeftElbowPose(
            calculateLeftShoulderPose(calculateHipPose(_future_foot_prints.at(timeStep)), angles.segment(2, 3), angles[0]), angles[5]));
    }
    else if ("RUArm" == bodyPart)
    {
        obs.state._poses[timeStep] = calculateRightShoulderPose(calculateHipPose(_future_foot_prints.at(timeStep)), angles.segment(6, 3), angles[0]);
    }
    else if ("RFArm" == bodyPart)
    {
        obs.state._poses[timeStep] = calculateRightElbowPose(
            calculateRightShoulderPose(calculateHipPose(_future_foot_prints.at(timeStep)), angles.segment(6, 3), angles[0]), angles[9]);
    }
    else if ("RHand" == bodyPart)
    {
        obs.state._poses[timeStep] = calculateRightHandPose(calculateRightElbowPose(
            calculateRightShoulderPose(calculateHipPose(_future_foot_prints.at(timeStep)), angles.segment(6, 3), angles[0]), angles[9]));
    }
    else if ("Leg" == bodyPart)
    {
        ROS_ERROR("LEG IDS");
        obs.state._poses[timeStep] = calculateHipPose(_future_foot_prints.at(timeStep));
    }
    else
    {
        ROS_ERROR("human.cpp: UpdateObstaclePose: Unknown Body Part Name!");
    }
}

void Human::addUncertaintyObstacles(std::vector<Eigen::MatrixXd>& angles, int timeStep, std::vector<int>& splitIdx)
{

    if (timeStep == 1)
    {
        // Clearing previous run
        _id_count = _body_part_count;
        _Neck_ids.clear();
        _Head_ids.clear();
        _LUArm_ids.clear();
        _LFArm_ids.clear();
        _LHand_ids.clear();
        _RUArm_ids.clear();
        _RFArm_ids.clear();
        _RHand_ids.clear();
        _Leg_ids.clear();

        _body_parts.find("Head")->second.uncertainty_states.clear();
        _body_parts.find("Neck")->second.uncertainty_states.clear();
        _body_parts.find("LUArm")->second.uncertainty_states.clear();
        _body_parts.find("LFArm")->second.uncertainty_states.clear();
        _body_parts.find("LHand")->second.uncertainty_states.clear();
        _body_parts.find("RUArm")->second.uncertainty_states.clear();
        _body_parts.find("RFArm")->second.uncertainty_states.clear();
        _body_parts.find("RHand")->second.uncertainty_states.clear();
        _body_parts.find("Leg")->second.uncertainty_states.clear();

        // Add Body parts
        int nbConfsL = 1;
        int nbConfsR = 1;

        _all_uncertainty_angles.clear();

        int variationCounter;

        int uncertaintyInstanceCounter = 1;
        for (int i = 0; i < angles.size(); ++i)
        {
            switch (i)
            {
                case 0:
                    for (int var = 1; var < angles.at(i).cols(); ++var)
                    {
                        _body_parts.find("Neck")->second.uncertainty_states.push_back(_body_parts.find("Neck")->second.state);

                        _Neck_ids.push_back({var, var});
                        if (std::none_of(_all_uncertainty_angles.begin(), _all_uncertainty_angles.end(),
                                         [&angles, &i, &var](std::pair<int, std::vector<double>> elem) {
                                             std::vector<double> tmp(
                                                 angles.at(i).col(var).data(),
                                                 angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols());
                                             return std::equal(elem.second.begin(), elem.second.end(), tmp.begin(),
                                                               [](double i, double j) { return (std::abs(i - j) < 1e-3); });
                                         }))
                        {
                            _all_uncertainty_angles.push_back(
                                {uncertaintyInstanceCounter,
                                 std::vector<double>(angles.at(i).col(var).data(),
                                                     angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols())});
                            uncertaintyInstanceCounter++;
                        }
                    }
                    break;
                case 1:
                    for (int var = 1; var < angles.at(i).cols(); ++var)
                    {
                        _body_parts.find("Head")->second.uncertainty_states.push_back(_body_parts.find("Head")->second.state);

                        _Head_ids.push_back({var, var});
                        if (std::none_of(_all_uncertainty_angles.begin(), _all_uncertainty_angles.end(),
                                         [&angles, &i, &var](std::pair<int, std::vector<double>> elem) {
                                             std::vector<double> tmp(
                                                 angles.at(i).col(var).data(),
                                                 angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols());
                                             return std::equal(elem.second.begin(), elem.second.end(), tmp.begin(),
                                                               [](double i, double j) { return (std::abs(i - j) < 1e-3); });
                                         }))
                        {
                            _all_uncertainty_angles.push_back(
                                {uncertaintyInstanceCounter,
                                 std::vector<double>(angles.at(i).col(var).data(),
                                                     angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols())});
                            uncertaintyInstanceCounter++;
                        }
                    }
                    break;
                case 2:
                    for (int var = nbConfsL; var < angles.at(i).cols(); ++var)
                    {
                        _body_parts.find("LUArm")->second.uncertainty_states.push_back(_body_parts.find("LUArm")->second.state);

                        if (_Head_ids.size() > 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _Head_ids.size();
                            _LUArm_ids.push_back({variationCounter, nbConfsL});
                        }
                        else
                        {
                            _LUArm_ids.push_back({var, nbConfsL});
                        }
                        nbConfsL++;
                        if (std::none_of(_all_uncertainty_angles.begin(), _all_uncertainty_angles.end(),
                                         [&angles, &i, &var](std::pair<int, std::vector<double>> elem) {
                                             std::vector<double> tmp(
                                                 angles.at(i).col(var).data(),
                                                 angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols());
                                             return std::equal(elem.second.begin(), elem.second.end(), tmp.begin(),
                                                               [](double i, double j) { return (std::abs(i - j) < 1e-3); });
                                         }))
                        {
                            _all_uncertainty_angles.push_back(
                                {uncertaintyInstanceCounter,
                                 std::vector<double>(angles.at(i).col(var).data(),
                                                     angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols())});
                            uncertaintyInstanceCounter++;
                        }
                    }
                    break;
                case 3:
                    for (int var = nbConfsL; var < angles.at(i).cols(); ++var)
                    {
                        _body_parts.find("LUArm")->second.uncertainty_states.push_back(_body_parts.find("LUArm")->second.state);

                        if (_Head_ids.size() > 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _Head_ids.size();
                            _LUArm_ids.push_back({variationCounter, nbConfsL});
                        }
                        else
                        {
                            _LUArm_ids.push_back({var, nbConfsL});
                        }
                        nbConfsL++;
                        if (std::none_of(_all_uncertainty_angles.begin(), _all_uncertainty_angles.end(),
                                         [&angles, &i, &var](std::pair<int, std::vector<double>> elem) {
                                             std::vector<double> tmp(
                                                 angles.at(i).col(var).data(),
                                                 angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols());
                                             return std::equal(elem.second.begin(), elem.second.end(), tmp.begin(),
                                                               [](double i, double j) { return (std::abs(i - j) < 1e-3); });
                                         }))
                        {
                            _all_uncertainty_angles.push_back(
                                {uncertaintyInstanceCounter,
                                 std::vector<double>(angles.at(i).col(var).data(),
                                                     angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols())});
                            uncertaintyInstanceCounter++;
                        }
                    }
                    break;
                case 4:
                    for (int var = nbConfsL; var < angles.at(i).cols(); ++var)
                    {
                        _body_parts.find("LUArm")->second.uncertainty_states.push_back(_body_parts.find("LUArm")->second.state);

                        if (_Head_ids.size() > 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _Head_ids.size();
                            _LUArm_ids.push_back({variationCounter, nbConfsL});
                        }
                        else
                        {
                            _LUArm_ids.push_back({var, nbConfsL});
                        }
                        nbConfsL++;
                        if (std::none_of(_all_uncertainty_angles.begin(), _all_uncertainty_angles.end(),
                                         [&angles, &i, &var](std::pair<int, std::vector<double>> elem) {
                                             std::vector<double> tmp(
                                                 angles.at(i).col(var).data(),
                                                 angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols());
                                             return std::equal(elem.second.begin(), elem.second.end(), tmp.begin(),
                                                               [](double i, double j) { return (std::abs(i - j) < 1e-3); });
                                         }))
                        {
                            _all_uncertainty_angles.push_back(
                                {uncertaintyInstanceCounter,
                                 std::vector<double>(angles.at(i).col(var).data(),
                                                     angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols())});
                            uncertaintyInstanceCounter++;
                        }
                    }
                    break;
                case 5:
                    for (int var = 1; var < angles.at(i).cols(); ++var)
                    {

                        _body_parts.find("LFArm")->second.uncertainty_states.push_back(_body_parts.find("LFArm")->second.state);
                        _body_parts.find("LHand")->second.uncertainty_states.push_back(_body_parts.find("LHand")->second.state);

                        if (_Head_ids.size() > 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _Head_ids.size();
                            _LFArm_ids.push_back({variationCounter, var});
                            _LHand_ids.push_back({variationCounter, var});
                        }
                        else
                        {
                            _LFArm_ids.push_back({var, var});
                            _LHand_ids.push_back({var, var});
                        }
                        nbConfsL++;
                        if (std::none_of(_all_uncertainty_angles.begin(), _all_uncertainty_angles.end(),
                                         [&angles, &i, &var](std::pair<int, std::vector<double>> elem) {
                                             std::vector<double> tmp(
                                                 angles.at(i).col(var).data(),
                                                 angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols());
                                             return std::equal(elem.second.begin(), elem.second.end(), tmp.begin(),
                                                               [](double i, double j) { return (std::abs(i - j) < 1e-3); });
                                         }))
                        {
                            _all_uncertainty_angles.push_back(
                                {uncertaintyInstanceCounter,
                                 std::vector<double>(angles.at(i).col(var).data(),
                                                     angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols())});
                            uncertaintyInstanceCounter++;
                        }
                    }
                    break;
                case 6:
                    for (int var = nbConfsR; var < angles.at(i).cols(); ++var)
                    {

                        _body_parts.find("RUArm")->second.uncertainty_states.push_back(_body_parts.find("RUArm")->second.state);

                        if (_Head_ids.size() > 0 && _LFArm_ids.size() == 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _Head_ids.size();
                            _RUArm_ids.push_back({variationCounter, nbConfsR});
                        }
                        else if (_Head_ids.size() == 0 && _LFArm_ids.size() > 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _LFArm_ids.size();
                            _RUArm_ids.push_back({variationCounter, nbConfsR});
                        }
                        else if (_Head_ids.size() > 0 && _LFArm_ids.size() > 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _Head_ids.size() + _LFArm_ids.size();

                            _RUArm_ids.push_back({variationCounter, nbConfsR});
                        }
                        else
                        {
                            _RUArm_ids.push_back({var, nbConfsR});
                        }

                        nbConfsR++;
                        if (std::none_of(_all_uncertainty_angles.begin(), _all_uncertainty_angles.end(),
                                         [&angles, &i, &var](std::pair<int, std::vector<double>> elem) {
                                             std::vector<double> tmp(
                                                 angles.at(i).col(var).data(),
                                                 angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols());
                                             return std::equal(elem.second.begin(), elem.second.end(), tmp.begin(),
                                                               [](double i, double j) { return (std::abs(i - j) < 1e-3); });
                                         }))
                        {
                            _all_uncertainty_angles.push_back(
                                {uncertaintyInstanceCounter,
                                 std::vector<double>(angles.at(i).col(var).data(),
                                                     angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols())});
                            uncertaintyInstanceCounter++;
                        }
                    }
                    break;
                case 7:
                    for (int var = nbConfsR; var < angles.at(i).cols(); ++var)
                    {

                        _body_parts.find("RUArm")->second.uncertainty_states.push_back(_body_parts.find("RUArm")->second.state);

                        if (_Head_ids.size() > 0 && _LFArm_ids.size() == 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _Head_ids.size();
                            _RUArm_ids.push_back({variationCounter, nbConfsR});
                        }
                        else if (_Head_ids.size() == 0 && _LFArm_ids.size() > 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _LFArm_ids.size();
                            _RUArm_ids.push_back({variationCounter, nbConfsR});
                        }
                        else if (_Head_ids.size() > 0 && _LFArm_ids.size() > 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _Head_ids.size() + _LFArm_ids.size();

                            _RUArm_ids.push_back({variationCounter, nbConfsR});
                        }
                        else
                        {
                            _RUArm_ids.push_back({var, nbConfsR});
                        }
                        nbConfsR++;
                        if (std::none_of(_all_uncertainty_angles.begin(), _all_uncertainty_angles.end(),
                                         [&angles, &i, &var](std::pair<int, std::vector<double>> elem) {
                                             std::vector<double> tmp(
                                                 angles.at(i).col(var).data(),
                                                 angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols());
                                             return std::equal(elem.second.begin(), elem.second.end(), tmp.begin(),
                                                               [](double i, double j) { return (std::abs(i - j) < 1e-3); });
                                         }))
                        {
                            _all_uncertainty_angles.push_back(
                                {uncertaintyInstanceCounter,
                                 std::vector<double>(angles.at(i).col(var).data(),
                                                     angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols())});
                            uncertaintyInstanceCounter++;
                        }
                    }
                    break;
                case 8:
                    for (int var = nbConfsR; var < angles.at(i).cols(); ++var)
                    {
                        _body_parts.find("RUArm")->second.uncertainty_states.push_back(_body_parts.find("RUArm")->second.state);

                        if (_Head_ids.size() > 0 && _LFArm_ids.size() == 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _Head_ids.size();
                            _RUArm_ids.push_back({variationCounter, nbConfsR});
                        }
                        else if (_Head_ids.size() == 0 && _LFArm_ids.size() > 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _LFArm_ids.size();
                            _RUArm_ids.push_back({variationCounter, nbConfsR});
                        }
                        else if (_Head_ids.size() > 0 && _LFArm_ids.size() > 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _Head_ids.size() + _LFArm_ids.size();

                            _RUArm_ids.push_back({variationCounter, nbConfsR});
                        }
                        else
                        {
                            _RUArm_ids.push_back({var, nbConfsR});
                        }
                        nbConfsR++;
                        if (std::none_of(_all_uncertainty_angles.begin(), _all_uncertainty_angles.end(),
                                         [&angles, &i, &var](std::pair<int, std::vector<double>> elem) {
                                             std::vector<double> tmp(
                                                 angles.at(i).col(var).data(),
                                                 angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols());
                                             return std::equal(elem.second.begin(), elem.second.end(), tmp.begin(),
                                                               [](double i, double j) { return (std::abs(i - j) < 1e-3); });
                                         }))
                        {
                            _all_uncertainty_angles.push_back(
                                {uncertaintyInstanceCounter,
                                 std::vector<double>(angles.at(i).col(var).data(),
                                                     angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols())});
                            uncertaintyInstanceCounter++;
                        }
                    }
                    break;
                case 9:
                    for (int var = 1; var < angles.at(i).cols(); ++var)
                    {
                        _body_parts.find("RFArm")->second.uncertainty_states.push_back(_body_parts.find("RFArm")->second.state);
                        _body_parts.find("RHand")->second.uncertainty_states.push_back(_body_parts.find("RHand")->second.state);

                        if (_Head_ids.size() > 0 && _LFArm_ids.size() == 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _Head_ids.size();
                            _RFArm_ids.push_back({variationCounter, var});
                            _RHand_ids.push_back({variationCounter, var});
                        }
                        else if (_Head_ids.size() == 0 && _LFArm_ids.size() > 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _LFArm_ids.size();
                            _RFArm_ids.push_back({variationCounter, var});
                            _RHand_ids.push_back({variationCounter, var});
                        }
                        else if (_Head_ids.size() > 0 && _LFArm_ids.size() > 0 && var > _Neck_ids.size())
                        {
                            variationCounter = var + _Head_ids.size() + _LFArm_ids.size();

                            _RFArm_ids.push_back({variationCounter, var});
                            _RHand_ids.push_back({variationCounter, var});
                        }
                        else
                        {
                            _RFArm_ids.push_back({var, var});
                            _RHand_ids.push_back({var, var});
                        }

                        nbConfsR++;
                        if (std::none_of(_all_uncertainty_angles.begin(), _all_uncertainty_angles.end(),
                                         [&angles, &i, &var](std::pair<int, std::vector<double>> elem) {
                                             std::vector<double> tmp(
                                                 angles.at(i).col(var).data(),
                                                 angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols());
                                             return std::equal(elem.second.begin(), elem.second.end(), tmp.begin(),
                                                               [](double i, double j) { return (std::abs(i - j) < 1e-3); });
                                         }))
                        {
                            _all_uncertainty_angles.push_back(
                                {uncertaintyInstanceCounter,
                                 std::vector<double>(angles.at(i).col(var).data(),
                                                     angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols())});
                            uncertaintyInstanceCounter++;
                        }
                    }
                    break;
                case 10:
                    for (int var = 1; var < angles.at(i).cols(); ++var)
                    {
                        _body_parts.find("Leg")->second.uncertainty_states.push_back(_body_parts.find("Leg")->second.state);

                        _Leg_ids.push_back({uncertaintyInstanceCounter, var});
                        if (std::none_of(_all_uncertainty_angles.begin(), _all_uncertainty_angles.end(),
                                         [&angles, &i, &var](std::pair<int, std::vector<double>> elem) {
                                             std::vector<double> tmp(
                                                 angles.at(i).col(var).data(),
                                                 angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols());
                                             return std::equal(elem.second.begin(), elem.second.end(), tmp.begin(),
                                                               [](double i, double j) { return (std::abs(i - j) < 1e-3); });
                                         }))
                        {
                            _all_uncertainty_angles.push_back(
                                {uncertaintyInstanceCounter,
                                 std::vector<double>(angles.at(i).col(var).data(),
                                                     angles.at(i).col(var).data() + angles.at(i).col(var).rows() * angles.at(i).col(var).cols())});
                            uncertaintyInstanceCounter++;
                        }
                    }
                    break;
                default:
                    ROS_ERROR("human.cpp: addUncertaintyObstacle: Unknown splitted Joint");
                    break;
            }
        }
    }
    // Update Body part poses for timesteps
    for (auto var : _Head_ids)
    {
        _body_parts.find("Head")->second.uncertainty_states.at(var.second - 1)._poses[timeStep] = calculateHeadPose(
            calculateNeckpose(calculateHipPose(_future_foot_prints.at(timeStep)), angles.at(1).col(var.second)[0]), angles.at(1).col(var.second)[1]);
        _body_parts.find("Head")->second.uncertainty_states.at(var.second - 1)._uncertainty_instance = var.first;
    }
    for (auto var : _Neck_ids)
    {
        _body_parts.find("Neck")->second.uncertainty_states.at(var.second - 1)._poses[timeStep] =
            calculateNeckpose(calculateHipPose(_future_foot_prints.at(timeStep)), angles.at(0).col(var.second)[0]);
        _body_parts.find("Neck")->second.uncertainty_states.at(var.second - 1)._uncertainty_instance = var.first;
    }
    for (auto var : _LUArm_ids)
    {
        _body_parts.find("LUArm")->second.uncertainty_states.at(var.second - 1)._poses[timeStep] = calculateLeftShoulderPose(
            calculateHipPose(_future_foot_prints.at(timeStep)), angles.at(4).col(var.second).segment(2, 3), angles.at(4).col(var.second)[0]);
        _body_parts.find("LUArm")->second.uncertainty_states.at(var.second - 1)._uncertainty_instance = var.first;
    }
    for (auto var : _LFArm_ids)
    {
        _body_parts.find("LFArm")->second.uncertainty_states.at(var.second - 1)._poses[timeStep] =
            calculateLeftElbowPose(calculateLeftShoulderPose(calculateHipPose(_future_foot_prints.at(timeStep)),
                                                             angles.at(5).col(var.second).segment(2, 3), angles.at(5).col(var.second)[0]),
                                   angles.at(5).col(var.second)[5]);
        _body_parts.find("LFArm")->second.uncertainty_states.at(var.second - 1)._uncertainty_instance = var.first;
    }
    for (auto var : _LHand_ids)
    {
        _body_parts.find("LHand")->second.uncertainty_states.at(var.second - 1)._poses[timeStep] = calculateLeftHandPose(
            calculateLeftElbowPose(calculateLeftShoulderPose(calculateHipPose(_future_foot_prints.at(timeStep)),
                                                             angles.at(5).col(var.second).segment(2, 3), angles.at(5).col(var.second)[0]),
                                   angles.at(5).col(var.second)[5]));
        _body_parts.find("LHand")->second.uncertainty_states.at(var.second - 1)._uncertainty_instance = var.first;
    }
    for (auto var : _RUArm_ids)
    {
        _body_parts.find("RUArm")->second.uncertainty_states.at(var.second - 1)._poses[timeStep] = calculateRightShoulderPose(
            calculateHipPose(_future_foot_prints.at(timeStep)), angles.at(8).col(var.second).segment(6, 3), angles.at(8).col(var.second)[0]);
        _body_parts.find("RUArm")->second.uncertainty_states.at(var.second - 1)._uncertainty_instance = var.first;
    }
    for (auto var : _RFArm_ids)
    {
        _body_parts.find("RFArm")->second.uncertainty_states.at(var.second - 1)._poses[timeStep] =
            calculateRightElbowPose(calculateRightShoulderPose(calculateHipPose(_future_foot_prints.at(timeStep)),
                                                               angles.at(9).col(var.second).segment(6, 3), angles.at(9).col(var.second)[0]),
                                    angles.at(9).col(var.second)[9]);
        _body_parts.find("RFArm")->second.uncertainty_states.at(var.second - 1)._uncertainty_instance = var.first;
    }
    for (auto var : _RHand_ids)
    {
        _body_parts.find("RHand")->second.uncertainty_states.at(var.second - 1)._poses[timeStep] = calculateRightHandPose(
            calculateRightElbowPose(calculateRightShoulderPose(calculateHipPose(_future_foot_prints.at(timeStep)),
                                                               angles.at(9).col(var.second).segment(6, 3), angles.at(9).col(var.second)[0]),
                                    angles.at(9).col(var.second)[9]));
        _body_parts.find("RHand")->second.uncertainty_states.at(var.second - 1)._uncertainty_instance = var.first;
    }
    for (auto var : _Leg_ids)
    {
        _body_parts.find("Leg")->second.uncertainty_states.at(var.second - 1)._poses[timeStep] = calculateHipPose(_future_foot_prints.at(timeStep));
        _body_parts.find("Leg")->second.uncertainty_states.at(var.second - 1)._uncertainty_instance = var.first;
    }
}

void Human::deleteUncertaintyObstacles()
{
    // Clearing prevoius run
    _id_count = _body_part_count;

    _body_parts.find("Head")->second.uncertainty_states.clear();
    _body_parts.find("Neck")->second.uncertainty_states.clear();
    _body_parts.find("LUArm")->second.uncertainty_states.clear();
    _body_parts.find("LFArm")->second.uncertainty_states.clear();
    _body_parts.find("LHand")->second.uncertainty_states.clear();
    _body_parts.find("RUArm")->second.uncertainty_states.clear();
    _body_parts.find("RFArm")->second.uncertainty_states.clear();
    _body_parts.find("RHand")->second.uncertainty_states.clear();
    _body_parts.find("Leg")->second.uncertainty_states.clear();

    _Neck_ids.clear();
    _Head_ids.clear();
    _LUArm_ids.clear();
    _LFArm_ids.clear();
    _LHand_ids.clear();
    _RUArm_ids.clear();
    _RFArm_ids.clear();
    _RHand_ids.clear();
    _Leg_ids.clear();
}

void Human::rewriteBoundingBoxes()
{
    // Only for visualization purposes
    for (auto bp = _body_parts.begin(); bp != _body_parts.end(); ++bp)
    {

        if (bp->second.name == "Hip")
        {
            bp->second.bounding_box.length_x = std::max(_length.at("Hip_length"), 0.0);
            bp->second.bounding_box.length_y = 0.0;
            bp->second.bounding_box.radius   = 0.0;
            bp->second.bounding_box.T        = Common::rotz(-M_PI / 2);
            bp->second.bounding_box.type     = BoundingBoxType::SPHERE;
        }
        if (bp->second.name == "Neck")
        {
            bp->second.bounding_box.length_x = std::max(_length.at("Neck_length") - 1 * _radius.at("Neck_radius"), 0.0);
            bp->second.bounding_box.length_y = 0.0;
            bp->second.bounding_box.radius   = std::max(_radius.at("Neck_radius"), 0.0);
            bp->second.bounding_box.T        = Common::rotz(-M_PI / 2);
            bp->second.bounding_box.type     = BoundingBoxType::CYLINDER;
        }
        if (bp->second.name == "Head")
        {
            bp->second.bounding_box.length_x = std::max(_length.at("Head_length"), 0.0);
            bp->second.bounding_box.length_y = 0.0;
            bp->second.bounding_box.radius   = std::max(_radius.at("Head_radius"), 0.0);
            Eigen::Matrix4d boxTranslation{Eigen::Matrix4d::Identity()};
            boxTranslation(1, 3)         = _radius.at("Hip_radius");
            bp->second.bounding_box.T    = boxTranslation;
            bp->second.bounding_box.type = BoundingBoxType::SPHERE;
        }
        if (bp->second.name == "LUArm")
        {
            bp->second.bounding_box.length_x = std::max(_length.at("LUArm_length") - 2 * _radius.at("LUArm_radius"), 0.0);
            Eigen::Matrix4d boxTranslation{Eigen::Matrix4d::Identity()};
            boxTranslation(0, 3)             = _radius.at("LUArm_radius");
            bp->second.bounding_box.T        = boxTranslation;
            bp->second.bounding_box.length_y = 0.0;
            bp->second.bounding_box.radius   = std::max(_radius.at("LUArm_radius"), 0.0);
            bp->second.bounding_box.type     = BoundingBoxType::CYLINDER;
        }
        if (bp->second.name == "LFArm")
        {
            bp->second.bounding_box.length_x = std::max(_length.at("LFArm_length") - 2 * _radius.at("LFArm_radius"), 0.0);
            bp->second.bounding_box.length_y = 0.0;
            Eigen::Matrix4d boxTranslation{Eigen::Matrix4d::Identity()};
            boxTranslation(0, 3)             = _radius.at("LFArm_radius");
            bp->second.bounding_box.T        = boxTranslation;
            bp->second.bounding_box.length_y = 0.0;
            bp->second.bounding_box.radius   = std::max(_radius.at("LFArm_radius"), 0.0);
            bp->second.bounding_box.type     = BoundingBoxType::CYLINDER;
        }
        if (bp->second.name == "LHand")
        {
            bp->second.bounding_box.length_x = 0.0;
            bp->second.bounding_box.length_y = 0.0;
            bp->second.bounding_box.radius   = 0.0;
            bp->second.bounding_box.type     = BoundingBoxType::SPHERE;
        }
        if (bp->second.name == "RUArm")
        {
            bp->second.bounding_box.length_x = std::max(_length.at("RUArm_length") - 2 * _radius.at("RUArm_radius"), 0.0);
            bp->second.bounding_box.length_y = 0.0;
            bp->second.bounding_box.radius   = std::max(_radius.at("RUArm_radius"), 0.0);
            Eigen::Matrix4d tmp              = Common::roty(M_PI);
            tmp(0, 3)                        = -_radius.at("RUArm_radius");
            bp->second.bounding_box.T        = tmp;

            bp->second.bounding_box.type = BoundingBoxType::CYLINDER;
        }
        if (bp->second.name == "RFArm")
        {
            bp->second.bounding_box.length_x = std::max(_length.at("RFArm_length") - 2 * _radius.at("RFArm_radius"), 0.0);
            bp->second.bounding_box.length_y = 0.0;
            bp->second.bounding_box.radius   = std::max(_radius.at("RFArm_radius"), 0.0);
            Eigen::Matrix4d tmp              = Common::roty(M_PI);
            tmp(0, 3)                        = -_radius.at("RFArm_radius");
            bp->second.bounding_box.T        = tmp;
            bp->second.bounding_box.type     = BoundingBoxType::CYLINDER;
        }
        if (bp->second.name == "RHand")
        {
            bp->second.bounding_box.length_x = 0.0;
            bp->second.bounding_box.length_y = 0.0;
            bp->second.bounding_box.radius   = 0.0;
            bp->second.bounding_box.type     = BoundingBoxType::SPHERE;
        }
    }
}

void Human::checkJointLimits(Eigen::Vector<double, 11>& check_angles)
{
    // checking if angles exceed the predefined joint angle limits of the skeleton
    for (int cnt = 0; cnt < check_angles.size(); ++cnt)
    {
        if (check_angles(cnt) < _lower_angle_limit(cnt))
        {
            check_angles(cnt) = _lower_angle_limit(cnt);
        }
        else if (check_angles(cnt) > _upper_angle_limit(cnt))
        {
            check_angles(cnt) = _upper_angle_limit(cnt);
        }
    }
}

Eigen::VectorXd Human::interpolateVectors(const Eigen::Ref<const Eigen::VectorXd>& vec1, const Eigen::Ref<const Eigen::VectorXd>& vec2,
                                          double scaling)
{
    Eigen::VectorXd interpolatedVector = Eigen::VectorXd::Zero(vec1.size());
    interpolatedVector                 = vec1 + (vec2 - vec1) * scaling;
    return interpolatedVector;
}

// Private functions from here:
// For information of the inverse kinematic functions refer to Heiko Renz's Master thesis
void Human::calculateShoulderAngle(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_shoulder,
                                   const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_elbow,
                                   const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_hand, bool flagLeft)
{
    Eigen::Matrix4d ToHipRot   = Eigen::Matrix4d::Identity();
    ToHipRot.block<3, 3>(0, 0) = _body_parts.at("Hip").state._poses[0].block<3, 3>(0, 0);

    Eigen::Vector3d posShoulder = (Common::rotx(_joint_angles[0] - M_PI / 2).transpose() * ToHipRot.transpose() * pose_shoulder).block<3, 1>(0, 3);
    Eigen::Vector3d posElbow    = (Common::rotx(_joint_angles[0] - M_PI / 2).transpose() * ToHipRot.transpose() * pose_elbow).block<3, 1>(0, 3);
    Eigen::Vector3d posHand = (Common::rotx(_joint_angles[0] - M_PI / 2).transpose() * ToHipRot.transpose() * pose_hand * ToHipRot).block<3, 1>(0, 3);
    Eigen::Vector3d shoulderToElbow2 = Common::rotx(-_joint_angles[0]).block<3, 3>(0, 0) * (posElbow - posShoulder);
    Eigen::Vector3d shoulderToHand2  = Common::rotx(-_joint_angles[0]).block<3, 3>(0, 0) * (posHand - posShoulder);
    Eigen::Vector3d shoulderToElbow  = (posElbow - posShoulder);
    Eigen::Vector3d shoulderToHand   = (posHand - posShoulder);

    if (flagLeft)
    {

        if (shoulderToElbow[2] <= 0)
        {
            _joint_angles[3] = M_PI / 2 - acos(shoulderToElbow[0] / _length.at("LUArm_length"));
        }
        else
        {
            _joint_angles[3] = M_PI / 2 + acos(shoulderToElbow[0] / _length.at("LUArm_length"));
        }
        _joint_angles[2] = asin((shoulderToElbow[1]) / (sin(M_PI / 2 + _joint_angles[3]) * (-_length.at("LUArm_length"))));

        if (abs(round((cos(M_PI / 2 + _joint_angles[5]) * sin(M_PI / 2 + _joint_angles[3]) * _length.at("LFArm_length")) * 10000) / 10000) <= 1e-5)
        {
            _joint_angles[4] = 0;
        }
        else
        {
            _joint_angles[4] = acos(-((shoulderToHand[0] + cos(M_PI / 2 + _joint_angles[3]) * _length.at("LUArm_length") +
                                       cos(M_PI / 2 + _joint_angles[3]) * sin(M_PI / 2 + _joint_angles[5]) * _length.at("LFArm_length")) /
                                      (cos(M_PI / 2 + _joint_angles[5]) * sin(M_PI / 2 + _joint_angles[3]) * _length.at("LFArm_length")))) -
                               M_PI / 2;
            if (isnan(_joint_angles[4]))
            {
                _joint_angles[4] = 0;
            }
        }
    }

    else
    {
        if (shoulderToElbow[2] <= 0)
        {
            _joint_angles[7] = M_PI / 2 - acos(-shoulderToElbow[0] / _length.at("RUArm_length"));
        }
        else
        {
            _joint_angles[7] = M_PI / 2 + acos(-shoulderToElbow[0] / _length.at("RUArm_length"));
        }
        _joint_angles[6] = asin((shoulderToElbow[1]) / (sin(M_PI / 2 + _joint_angles[7]) * (-_length.at("RUArm_length"))));

        if (abs(round((cos(M_PI / 2 + _joint_angles[9]) * sin(M_PI / 2 + _joint_angles[7]) * _length.at("RFArm_length")) * 10000) / 10000) <= 1e-5)
        {
            _joint_angles[8] = 0;
        }
        else
        {
            _joint_angles[8] = (acos(((-shoulderToHand[0] + cos(M_PI / 2 + _joint_angles[7]) * _length.at("RUArm_length") +
                                       cos(M_PI / 2 + _joint_angles[7]) * sin(M_PI / 2 + _joint_angles[9]) * _length.at("RFArm_length")) /
                                      (cos(M_PI / 2 + _joint_angles[9]) * sin(M_PI / 2 + _joint_angles[7]) * _length.at("RFArm_length")))) -
                                M_PI / 2);
            if (isnan(_joint_angles[8]))
            {
                _joint_angles[8] = 0;
            }
        }
    }
}

void Human::calculateHipAngle(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_hip,
                              const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_neck)
{
    Eigen::Vector3d positionHip  = pose_hip.block<3, 1>(0, 3);
    Eigen::Vector3d positionNeck = pose_neck.block<3, 1>(0, 3);

    Eigen::Vector3d spineTransform = pose_hip.block<3, 3>(0, 0).transpose() * (positionNeck - positionHip);
    if (positionHip[2] <= positionNeck[2])
    {
        _joint_angles[0] = M_PI / 2 - acos(spineTransform(2) / _length.at("Neck_length"));
    }
    else
    {
        _joint_angles[0] = M_PI / 2 + acos(spineTransform(2) / _length.at("Neck_length"));
    }
}

void Human::calculateElbowAngle(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_hand,
                                const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_shoulder, bool flag_left)
{
    Eigen::Vector3d positionHand     = pose_hand.block<3, 1>(0, 3);
    Eigen::Vector3d positionShoulder = pose_shoulder.block<3, 1>(0, 3);
    if (flag_left)
    {
        // Common::rotz(-_joint_angles[0]).block<3, 3>(0, 0).transpose() *pose_shoulder.block<3, 3>(0, 0).transpose() *
        Eigen::Vector3d ShoulderToHand = (positionHand - positionShoulder);
        double tmp1 = round((M_PI + (acos((pow(_length.at("LUArm_length"), 2) + pow(_length.at("LFArm_length"), 2) - pow(ShoulderToHand.norm(), 2)) /
                                          (2 * _length.at("LUArm_length") * _length.at("LFArm_length"))))) *
                            1000) /
                      1000;
        double tmp2 = round((M_PI - (acos((pow(_length.at("LUArm_length"), 2) + pow(_length.at("LFArm_length"), 2) - pow(ShoulderToHand.norm(), 2)) /
                                          (2 * _length.at("LUArm_length") * _length.at("LFArm_length"))))) *
                            1000) /
                      1000;

        if (tmp1 >= 0 && tmp1 < M_PI)
        {
            _joint_angles[5] = tmp1;
        }
        else if (tmp2 >= 0 && tmp2 < M_PI)
        {
            _joint_angles[5] = tmp2;
        }
        else
        {
            _joint_angles[5] = 0;
        }
    }
    else
    {
        Eigen::Vector3d ShoulderToHand = (positionHand - positionShoulder);

        double tmp1 = round((M_PI + (acos((pow(_length.at("RUArm_length"), 2) + pow(_length.at("RFArm_length"), 2) - pow(ShoulderToHand.norm(), 2)) /
                                          (2 * _length.at("RUArm_length") * _length.at("RFArm_length"))))) *
                            1000) /
                      1000;
        double tmp2 = round((M_PI - (acos((pow(_length.at("RUArm_length"), 2) + pow(_length.at("RFArm_length"), 2) - pow(ShoulderToHand.norm(), 2)) /
                                          (2 * _length.at("RUArm_length") * _length.at("RFArm_length"))))) *
                            1000) /
                      1000;
        if (tmp1 >= 0 && tmp1 < M_PI)
        {
            _joint_angles[9] = tmp1;
        }
        else if (tmp2 >= 0 && tmp2 < M_PI)
        {
            _joint_angles[9] = tmp2;
        }
        else
        {
            _joint_angles[9] = 0;
        }
    }
}

void Human::calculateHeadAngle(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_head,
                               const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_neck)
{
    Eigen::Vector3d positionHead = pose_head.block<3, 1>(0, 3);
    Eigen::Vector3d positionNeck = pose_neck.block<3, 1>(0, 3);

    Eigen::Vector3d HeadToNeck = pose_neck.block<3, 3>(0, 0).transpose() * (positionHead - positionNeck);
    //    std::cout << "Neck Pose:\n" << pose_neck << "Pose head:\n" << pose_head << "\n" << std::endl;
    //    std::cout << "Head to neck transofrmaiotn: \n" << HeadToNeck << "\n and length: " << HeadToNeck.norm() << std::endl;
    _joint_angles[1] = acos(HeadToNeck(1) / _radius.at("Head_radius"));

    if (isnan(_joint_angles[1]))
    {
        //        std::cout << "Head Joint Angle Calcualtions not possible. Head To Neck y: " << HeadToNeck(1)
        //                  << " belonging Head Radius: " << _radius.at("Head_radius") << "\n"
        //                  << std::endl;
        Eigen::Matrix4d Neck = _body_parts.find("Neck")->second.state._poses[0];
        double tempRadius;
        Eigen::Matrix4d Head = _body_parts.find("Head")->second.state._poses[0];
        tempRadius           = (Neck.block<3, 1>(0, 3) - Head.block<3, 1>(0, 3)).norm();

        _joint_angles[1] = acos(HeadToNeck(1) / tempRadius);
    }
}

void Human::calculateShoulderAngleTf(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_shoulder,
                                     const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_elbow,
                                     const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_hand, bool flagLeft)
{
    Eigen::Matrix4d ToHipRot   = Eigen::Matrix4d::Identity();
    ToHipRot.block<3, 3>(0, 0) = _body_parts.at("Hip").state._poses[0].block<3, 3>(0, 0);

    Eigen::Vector3d posShoulder = (Common::rotx(_joint_angles[0] - M_PI / 2).transpose() * ToHipRot.transpose() * pose_shoulder).block<3, 1>(0, 3);
    Eigen::Vector3d posElbow    = (Common::rotx(_joint_angles[0] - M_PI / 2).transpose() * ToHipRot.transpose() * pose_elbow).block<3, 1>(0, 3);
    Eigen::Vector3d posHand     = (Common::rotx(_joint_angles[0] - M_PI / 2).transpose() * ToHipRot.transpose() * pose_hand).block<3, 1>(0, 3);
    Eigen::Vector3d shoulderToElbow = (posElbow - posShoulder);
    Eigen::Vector3d shoulderToHand  = (posHand - posShoulder);

    if (flagLeft)
    {
        double temp = acos(shoulderToElbow[0] / _length.at("LUArm_length"));
        Eigen::Vector2d var1{asin((shoulderToElbow[1]) / (sin(M_PI / 2 + M_PI / 2 - temp) * (-_length.at("LUArm_length")))), M_PI / 2 - temp};

        Eigen::Vector2d var2{M_PI - asin((shoulderToElbow[1]) / (sin(M_PI / 2 + M_PI / 2 - temp) * (-_length.at("LUArm_length")))), M_PI / 2 - temp};

        Eigen::Vector2d var3{asin((shoulderToElbow[1]) / (sin(M_PI / 2 + M_PI / 2 + temp) * (-_length.at("LUArm_length")))), M_PI / 2 + temp};

        Eigen::Vector2d var4{M_PI - asin((shoulderToElbow[1]) / (sin(M_PI / 2 + M_PI / 2 + temp) * (-_length.at("LUArm_length")))), M_PI / 2 + temp};

        Eigen::Matrix<double, 2, 4> variants{{var1[0], var2[0], var3[0], var4[0]}, {var1[1], var2[1], var3[1], var4[1]}};
        if (shoulderToElbow[2] > 0)
        {
            Eigen::Matrix<int, 2, 4> upjoints =
                (variants.array() > (M_PI / 2)).select(Eigen::Matrix<int, 2, 4>::Ones(), Eigen::Matrix<int, 2, 4>::Zero());
            Eigen::Matrix<int, 2, 4> lowjoints =
                (variants.array() < (-M_PI / 2)).select(Eigen::Matrix<int, 2, 4>::Ones(), Eigen::Matrix<int, 2, 4>::Zero());

            Eigen::Matrix<int, 1, 4> tmp{upjoints.colwise().sum() + lowjoints.colwise().sum()};
            for (int var = 0; var < tmp.cols(); ++var)
            {
                if (tmp[var] == 0) variants.col(var).setZero();
            }
        }
        else if (shoulderToElbow[2] <= 0)
        {
            Eigen::Matrix<int, 2, 4> upjoints =
                (variants.array() > (M_PI / 2)).select(Eigen::Matrix<int, 2, 4>::Ones(), Eigen::Matrix<int, 2, 4>::Zero());
            Eigen::Matrix<int, 2, 4> lowjoints =
                (variants.array() < (-M_PI / 2)).select(Eigen::Matrix<int, 2, 4>::Ones(), Eigen::Matrix<int, 2, 4>::Zero());
            Eigen::Matrix<int, 1, 4> tmp{upjoints.colwise().sum() + lowjoints.colwise().sum()};
            for (int var = 0; var < tmp.cols(); ++var)
            {
                if (tmp[var] == 1) variants.col(var).setZero();
            }
        }

        Eigen::Matrix<bool, 1, Eigen::Dynamic> non_zeros = variants.cast<bool>().colwise().any();
        Eigen::MatrixXd res(variants.rows(), non_zeros.count());
        int j = 0;
        for (int i = 0; i < variants.cols(); ++i)
        {
            if (non_zeros(i)) res.col(j++) = variants.col(i);
        }

        Eigen::Vector<double, 2> old_angles{_joint_angles[2], _joint_angles[3]};
        Eigen::Vector<double, Eigen::Dynamic> norm = (res.colwise() - old_angles).colwise().norm();
        int min_idx;
        norm.minCoeff(&min_idx);

        _joint_angles[2] = res(0, min_idx);
        _joint_angles[3] = res(1, min_idx);
        if (isnan(_joint_angles[2]))
        {
            ROS_WARN("human.cpp: IK Shoulder is NaN. Manually set to old value");
            _joint_angles[2] = old_angles[0];
        }
        if (isnan(_joint_angles[3]))
        {
            ROS_WARN("human.cpp: IK Shoulder is NaN. Manually set to old value");
            _joint_angles[3] = old_angles[1];
        }
        //        std::cout << "OldAngles: " << old_angles << "\n Subctraction: " << (res.colwise() - old_angles) << "\n Norm: " << norm
        //                  << "\n at idx: " << min_idx << std::endl;

        double St1 = sin(_joint_angles[2]);
        double St2 = sin(_joint_angles[3] + M_PI / 2);
        double St4 = sin(_joint_angles[5] + M_PI / 2);
        double Ct1 = cos(_joint_angles[2]);
        double Ct2 = cos(_joint_angles[3] + M_PI / 2);
        double Ct4 = cos(_joint_angles[5] + M_PI / 2);

        if (fabs(St2 - 0) <= 1e-6 || fabs(Ct1 * Ct4 * _length.at("LFArm_length") - 0) <= 1e-6)
        {
            ROS_WARN("human.cpp: Singularity occurs for right shoulder: Set last Shoulder Joint to old one!");
            _joint_angles[4] = _joint_angles[4];
        }
        else
        {
            double q1_1 =
                M_PI / 2 -
                asin(((shoulderToHand(1) - St1 * St2 * -_length.at("LUArm_length") + St1 * St2 * St4 * _length.at("LFArm_length") +
                       (Ct2 * St1 * (-shoulderToHand(0) - Ct2 * -_length.at("LUArm_length") + Ct2 * St4 * _length.at("LFArm_length"))) / St2) /
                      (Ct1 * Ct4 * _length.at("LFArm_length"))));
            double q1_2 =
                -M_PI / 2 +
                asin(((shoulderToHand(1) - St1 * St2 * -_length.at("LUArm_length") + St1 * St2 * St4 * _length.at("LFArm_length") +
                       (Ct2 * St1 * (-shoulderToHand(0) - Ct2 * -_length.at("LUArm_length") + Ct2 * St4 * _length.at("LFArm_length"))) / St2) /
                      (Ct1 * Ct4 * _length.at("LFArm_length"))));
            Eigen::Transform<double, 3, Eigen::Affine> rot1(Eigen::AngleAxis<double>(_joint_angles[2], Eigen::Vector3d(-1, 0, 0)));
            Eigen::Transform<double, 3, Eigen::Affine> rot2(Eigen::AngleAxis<double>(_joint_angles[3], Eigen::Vector3d(0, -1, 0)));
            Eigen::Transform<double, 3, Eigen::Affine> rot3_1(Eigen::AngleAxis<double>(std::real(q1_1), Eigen::Vector3d(0, 0, -1)));
            Eigen::Transform<double, 3, Eigen::Affine> rot3_2(Eigen::AngleAxis<double>(std::real(q1_2), Eigen::Vector3d(0, 0, -1)));
            Eigen::Transform<double, 3, Eigen::Affine> rot4(Eigen::AngleAxis<double>(_joint_angles[5], Eigen::Vector3d(-1, 0, 0)));
            Eigen::Transform<double, 3, Eigen::Affine> deltaTrans_1 = rot1 * rot2 * rot3_1 * Eigen::Translation3d(0, 0, -_length.at("LUArm_length")) *
                                                                      rot4 * Eigen::Translation3d(0, 0, -_length.at("LFArm_length"));
            Eigen::Transform<double, 3, Eigen::Affine> deltaTrans_2 = rot1 * rot2 * rot3_2 * Eigen::Translation3d(0, 0, -_length.at("LUArm_length")) *
                                                                      rot4 * Eigen::Translation3d(0, 0, -_length.at("LFArm_length"));
            if (((deltaTrans_1.translation()) - shoulderToHand).norm() < ((deltaTrans_2.translation()) - shoulderToHand).norm())
            {
                _joint_angles[4] = q1_1;
            }
            else
            {
                _joint_angles[4] = q1_2;
            }
            if (isnan(_joint_angles[4]))
            {
                _joint_angles[4] = 0;
            }
        }

        //        if (shoulderToElbow[2] <= 0)
        //        {
        //            _joint_angles[3] = M_PI / 2 - acos(shoulderToElbow[0] / _length.at("LUArm_length"));
        //        }
        //        else
        //        {
        //            _joint_angles[3] = M_PI / 2 + acos(shoulderToElbow[0] / _length.at("LUArm_length"));
        //        }
        //        if (!isnan(asin((shoulderToElbow[1]) / (sin(M_PI / 2 + _joint_angles[3]) * (-_length.at("LUArm_length"))))))
        //        {
        //            _joint_angles[2] = asin((shoulderToElbow[1]) / (sin(M_PI / 2 + _joint_angles[3]) * (-_length.at("LUArm_length"))));
        //        }

        //        if (abs(round((cos(M_PI / 2 + _joint_angles[5]) * sin(M_PI / 2 + _joint_angles[3]) * _length.at("LFArm_length")) * 10000) /
        //        10000)
        //        <= 1e-5)
        //        {
        //            _joint_angles[4] = 0;
        //        }
        //        else
        //        {
        //            _joint_angles[4] = acos(-((shoulderToHand[0] + cos(M_PI / 2 + _joint_angles[3]) * _length.at("LUArm_length") +
        //                                       cos(M_PI / 2 + _joint_angles[3]) * sin(M_PI / 2 + _joint_angles[5]) * _length.at("LFArm_length"))
        //                                       /
        //                                      (cos(M_PI / 2 + _joint_angles[5]) * sin(M_PI / 2 + _joint_angles[3]) *
        //                                      _length.at("LFArm_length")))) -
        //                               M_PI / 2;
        //            if (isnan(_joint_angles[4]))
        //            {
        //                _joint_angles[4] = 0;
        //            }
        //        }
    }

    else
    {
        double temp = acos(-shoulderToElbow[0] / _length.at("RUArm_length"));

        Eigen::Vector2d var1{asin((shoulderToElbow[1]) / (sin(M_PI / 2 + M_PI / 2 - temp) * (-_length.at("RUArm_length")))), M_PI / 2 - temp};

        Eigen::Vector2d var2{M_PI - asin((shoulderToElbow[1]) / (sin(M_PI / 2 + M_PI / 2 - temp) * (-_length.at("RUArm_length")))), M_PI / 2 - temp};

        Eigen::Vector2d var3{asin((shoulderToElbow[1]) / (sin(M_PI / 2 + M_PI / 2 + temp) * (-_length.at("RUArm_length")))), M_PI / 2 + temp};

        Eigen::Vector2d var4{M_PI - asin((shoulderToElbow[1]) / (sin(M_PI / 2 + M_PI / 2 + temp) * (-_length.at("RUArm_length")))), M_PI / 2 + temp};

        Eigen::Matrix<double, 2, 4> variants{{var1[0], var2[0], var3[0], var4[0]}, {var1[1], var2[1], var3[1], var4[1]}};
        if (shoulderToElbow[2] > 0)
        {
            Eigen::Matrix<int, 2, 4> upjoints =
                (variants.array() > (M_PI / 2)).select(Eigen::Matrix<int, 2, 4>::Ones(), Eigen::Matrix<int, 2, 4>::Zero());
            Eigen::Matrix<int, 2, 4> lowjoints =
                (variants.array() < (-M_PI / 2)).select(Eigen::Matrix<int, 2, 4>::Ones(), Eigen::Matrix<int, 2, 4>::Zero());

            Eigen::Matrix<int, 1, 4> tmp{upjoints.colwise().sum() + lowjoints.colwise().sum()};
            for (int var = 0; var < tmp.cols(); ++var)
            {
                if (tmp[var] == 0) variants.col(var).setZero();
            }
        }
        else if (shoulderToElbow[2] <= 0)
        {
            Eigen::Matrix<int, 2, 4> upjoints =
                (variants.array() > (M_PI / 2)).select(Eigen::Matrix<int, 2, 4>::Ones(), Eigen::Matrix<int, 2, 4>::Zero());
            Eigen::Matrix<int, 2, 4> lowjoints =
                (variants.array() < (-M_PI / 2)).select(Eigen::Matrix<int, 2, 4>::Ones(), Eigen::Matrix<int, 2, 4>::Zero());
            Eigen::Matrix<int, 1, 4> tmp{upjoints.colwise().sum() + lowjoints.colwise().sum()};
            for (int var = 0; var < tmp.cols(); ++var)
            {
                if (tmp[var] == 1) variants.col(var).setZero();
            }
        }

        Eigen::Matrix<bool, 1, Eigen::Dynamic> non_zeros = variants.cast<bool>().colwise().any();
        Eigen::MatrixXd res(variants.rows(), non_zeros.count());
        int j = 0;
        for (int i = 0; i < variants.cols(); ++i)
        {
            if (non_zeros(i)) res.col(j++) = variants.col(i);
        }
        //        std::cout << "res:\n" << res << "\n\n";

        Eigen::Vector<double, 2> old_angles{_joint_angles[6], _joint_angles[7]};
        Eigen::Vector<double, Eigen::Dynamic> norm = (res.colwise() - old_angles).colwise().norm();
        int min_idx;
        norm.minCoeff(&min_idx);

        _joint_angles[6] = res(0, min_idx);
        _joint_angles[7] = res(1, min_idx);
        if (isnan(_joint_angles[6]))
        {
            ROS_WARN("human.cpp: IK Shoulder is NaN. Manually set to old value");
            _joint_angles[6] = old_angles[0];
        }

        if (isnan(_joint_angles[7]))
        {
            ROS_WARN("human.cpp: IK Shoulder is NaN. Manually set to old value");
            _joint_angles[7] = old_angles[1];
        }
        //        std::cout << "OldAngles: " << old_angles << "\n Subctraction: " << (res.colwise() - old_angles) << "\n Norm: " << norm
        //                  << "\n at idx: " << min_idx << std::endl;

        double St1 = sin(_joint_angles[6]);
        double St2 = sin(_joint_angles[7] + M_PI / 2);
        double St4 = sin(_joint_angles[9] + M_PI / 2);
        double Ct1 = cos(_joint_angles[6]);
        double Ct2 = cos(_joint_angles[7] + M_PI / 2);
        double Ct4 = cos(_joint_angles[9] + M_PI / 2);

        if (fabs(St2 - 0) <= 1e-6 || fabs(Ct1 * Ct4 * _length.at("RFArm_length") - 0) <= 1e-6)
        {
            ROS_WARN("human.cpp: Singularity occurs for right shoulder: Set last Shoulder Joint to old one!");
            _joint_angles[8] = _joint_angles[8];
        }
        else
        {
            double q1_1 =
                M_PI / 2 -
                asin(((shoulderToHand(1) - St1 * St2 * -_length.at("RUArm_length") + St1 * St2 * St4 * _length.at("RFArm_length") +
                       (Ct2 * St1 * (-shoulderToHand(0) - Ct2 * -_length.at("RUArm_length") + Ct2 * St4 * _length.at("RFArm_length"))) / St2) /
                      (Ct1 * Ct4 * _length.at("RFArm_length"))));
            double q1_2 =
                -M_PI / 2 +
                asin(((shoulderToHand(1) - St1 * St2 * -_length.at("RUArm_length") + St1 * St2 * St4 * _length.at("RFArm_length") +
                       (Ct2 * St1 * (-shoulderToHand(0) - Ct2 * -_length.at("RUArm_length") + Ct2 * St4 * _length.at("RFArm_length"))) / St2) /
                      (Ct1 * Ct4 * _length.at("RFArm_length"))));
            Eigen::Transform<double, 3, Eigen::Affine> rot1(Eigen::AngleAxis<double>(_joint_angles[6], Eigen::Vector3d(-1, 0, 0)));
            Eigen::Transform<double, 3, Eigen::Affine> rot2(Eigen::AngleAxis<double>(_joint_angles[7], Eigen::Vector3d(0, -1, 0)));
            Eigen::Transform<double, 3, Eigen::Affine> rot3_1(Eigen::AngleAxis<double>(std::real(q1_1), Eigen::Vector3d(0, 0, -1)));
            Eigen::Transform<double, 3, Eigen::Affine> rot3_2(Eigen::AngleAxis<double>(std::real(q1_2), Eigen::Vector3d(0, 0, -1)));
            Eigen::Transform<double, 3, Eigen::Affine> rot4(Eigen::AngleAxis<double>(_joint_angles[9], Eigen::Vector3d(-1, 0, 0)));
            Eigen::Transform<double, 3, Eigen::Affine> deltaTrans_1 = rot1 * rot2 * rot3_1 * Eigen::Translation3d(0, 0, -_length.at("RUArm_length")) *
                                                                      rot4 * Eigen::Translation3d(0, 0, -_length.at("RFArm_length"));
            Eigen::Transform<double, 3, Eigen::Affine> deltaTrans_2 = rot1 * rot2 * rot3_2 * Eigen::Translation3d(0, 0, -_length.at("RUArm_length")) *
                                                                      rot4 * Eigen::Translation3d(0, 0, -_length.at("RFArm_length"));

            if (((deltaTrans_1.translation()) - shoulderToHand).norm() < ((deltaTrans_2.translation()) - shoulderToHand).norm())
            {
                _joint_angles[8] = q1_1;
            }
            else
            {
                _joint_angles[8] = q1_2;
            }
            if (isnan(_joint_angles[8]))
            {
                _joint_angles[8] = 0;
            }
        }

        //        if (shoulderToElbow[2] <= 0)
        //        {
        //            _joint_angles[7] = M_PI / 2 - acos(-shoulderToElbow[0] / _length.at("RUArm_length"));
        //        }
        //        else
        //        {
        //            _joint_angles[7] = M_PI / 2 + acos(-shoulderToElbow[0] / _length.at("RUArm_length"));
        //        }
        //        if (!isnan(asin((shoulderToElbow[1]) / (sin(M_PI / 2 + _joint_angles[7]) * (-_length.at("RUArm_length"))))))
        //        {
        //            _joint_angles[6] = asin((shoulderToElbow[1]) / (sin(M_PI / 2 + _joint_angles[7]) * (-_length.at("RUArm_length"))));
        //        }

        //        if (abs(round((cos(M_PI / 2 + _joint_angles[9]) * sin(M_PI / 2 + _joint_angles[7]) * _length.at("RFArm_length")) * 10000) /
        //        10000)
        //        <= 1e-5)
        //        {
        //            _joint_angles[8] = 0;
        //        }
        //        else
        //        {
        //            _joint_angles[8] = (acos(((-shoulderToHand[0] + cos(M_PI / 2 + _joint_angles[7]) * _length.at("RUArm_length") +
        //                                       cos(M_PI / 2 + _joint_angles[7]) * sin(M_PI / 2 + _joint_angles[9]) * _length.at("RFArm_length"))
        //                                       /
        //                                      (cos(M_PI / 2 + _joint_angles[9]) * sin(M_PI / 2 + _joint_angles[7]) *
        //                                      _length.at("RFArm_length")))) -
        //                                M_PI / 2);
        //            if (isnan(_joint_angles[8]))
        //            {
        //                _joint_angles[8] = 0;
        //            }
        //        }
    }
}

void Human::calculateHipAngleTf(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_hip,
                                const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_neck)
{
    Eigen::Vector3d positionHip     = pose_hip.block<3, 1>(0, 3);
    Eigen::Vector3d positionNeck    = pose_neck.block<3, 1>(0, 3);
    Eigen::Vector3d spineTransformW = (positionNeck - positionHip);

    Eigen::Quaterniond quat(0.5, 0.5, 0.5, 0.5);
    Eigen::Matrix3d mocap = quat.toRotationMatrix();
    Eigen::Matrix3d relOr = mocap.transpose() * pose_hip.block<3, 3>(0, 0);
    Eigen::Vector3d spineTransformRel =
        Common::roty(atan2(relOr(0, 2), relOr(2, 2))).block<3, 3>(0, 0).transpose() * mocap.transpose() * positionNeck -
        Common::roty(atan2(relOr(0, 2), relOr(2, 2))).block<3, 3>(0, 0).transpose() * mocap.transpose() * positionHip;

    //    std::cout << "Mocap Rot: " << mocap << "\n" << std::endl;
    //    std::cout << "Rel Or to hip from Mocap: " << relOr << "\n" << std::endl;
    //    std::cout << "Rotation atround y: " << atan2(relOr(0, 2), relOr(2, 2))
    //    << "\n and as matrix: " << mocap * Common::roty(atan2(relOr(0, 2), relOr(2, 2))).block<3, 3>(0, 0) << "\n" << std::endl;
    //    std::cout << "Hip Pose:\n" << pose_hip << "Pose Neck:\n" << pose_neck << "\n" << std::endl;
    //    std::cout << "spine transofrmaiotn Rel: \n" << spineTransformRel << "\n and length: " << spineTransformRel.norm() << std::endl;
    //    std::cout << "spine transofrmaiotn World: \n" << spineTransformW << "\n and length: " << spineTransformW.norm() << std::endl;

    // Check if the neck is lower than the hip
    if (spineTransformW[2] >= 0)
    {
        if (spineTransformRel[2] < 0)
        // Check if human leans to the front or the back
        {
            // back lean
            _joint_angles[0] = -acos(spineTransformW[2] / _length.at("Neck_length"));
        }
        else
        {
            _joint_angles[0] = acos(spineTransformW[2] / _length.at("Neck_length"));
        }
    }
    else
    {
        _joint_angles[0] = acos(spineTransformW[2] / _length.at("Neck_length"));
    }

    _rot_hip << cos(_joint_angles[0]), 0, sin(_joint_angles[0]), 0, 1, 0, -sin(_joint_angles[0]), 0, cos(_joint_angles[0]);
    _rot_hip = -Common::roty(atan2(relOr(0, 2), relOr(2, 2))).block<3, 3>(0, 0).transpose() * mocap;
    //    std::cout << "Currently used _rot_hip: " << _rot_hip << "\n" << std::endl;
}

void Human::calculateElbowAngleTf(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_hand,
                                  const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_shoulder, bool flag_left)
{
    Eigen::Vector3d positionHand     = pose_hand.block<3, 1>(0, 3);
    Eigen::Vector3d positionShoulder = pose_shoulder.block<3, 1>(0, 3);
    if (flag_left)
    {
        Eigen::Vector3d ShoulderToHand = (positionHand - positionShoulder);

        double tmp1 =
            round((M_PI + abs(acos((pow(_length.at("LUArm_length"), 2) + pow(_length.at("LFArm_length"), 2) - pow(ShoulderToHand.norm(), 2)) /
                                   (2 * _length.at("LUArm_length") * _length.at("LFArm_length"))))) *
                  1000) /
            1000;
        double tmp2 =
            round((M_PI - abs(acos((pow(_length.at("LUArm_length"), 2) + pow(_length.at("LFArm_length"), 2) - pow(ShoulderToHand.norm(), 2)) /
                                   (2 * _length.at("LUArm_length") * _length.at("LFArm_length"))))) *
                  1000) /
            1000;

        if (tmp1 >= 0 && tmp1 < M_PI)
        {
            _joint_angles[5] = tmp1;
        }
        else if (tmp2 >= 0 && tmp2 < M_PI)
        {
            _joint_angles[5] = tmp2;
        }
        else
        {
            _joint_angles[5] = 0;
        }
    }
    else
    {
        Eigen::Vector3d ShoulderToHand = (positionHand - positionShoulder);

        double tmp1 =
            round((M_PI + abs(acos((pow(_length.at("RUArm_length"), 2) + pow(_length.at("RFArm_length"), 2) - pow(ShoulderToHand.norm(), 2)) /
                                   (2 * _length.at("RUArm_length") * _length.at("RFArm_length"))))) *
                  1000) /
            1000;
        double tmp2 =
            round((M_PI - abs(acos((pow(_length.at("RUArm_length"), 2) + pow(_length.at("RFArm_length"), 2) - pow(ShoulderToHand.norm(), 2)) /
                                   (2 * _length.at("RUArm_length") * _length.at("RFArm_length"))))) *
                  1000) /
            1000;

        if (tmp1 >= 0 && tmp1 < M_PI)
        {
            _joint_angles[9] = tmp1;
        }
        else if (tmp2 >= 0 && tmp2 < M_PI)
        {
            _joint_angles[9] = tmp2;
        }
        else
        {
            _joint_angles[9] = 0;
        }
    }
}

void Human::calculateHeadAngleTf(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_head,
                                 const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_neck)
{
    Eigen::Vector3d positionHead = pose_head.block<3, 1>(0, 3);
    Eigen::Vector3d positionNeck = pose_neck.block<3, 1>(0, 3);

    Eigen::Vector3d HeadToNeck = _body_parts.at("Hip").state._poses[0].block<3, 3>(0, 0).transpose() * (positionHead - positionNeck);
    //    std::cout << "Head to Neck:" << HeadToNeck << "\n with length: " << _radius.at("Head_radius") << std::endl;
    //    std::cout << "Rel or test: " << (pose_neck.block<3, 3>(0, 0).transpose() * pose_head.block<3, 3>(0, 0)).eulerAngles(2, 1, 0) << "\n" <<
    //    std::endl;

    _joint_angles[1] = asin(HeadToNeck(2) / _radius.at("Head_radius"));
    //    Eigen::Vector3d eul = (pose_neck.block<3, 3>(0, 0).transpose() * pose_head.block<3, 3>(0, 0)).eulerAngles(2, 1, 0);
    //    _joint_angles[1]    = eul[0];
}

Eigen::Matrix4d Human::calculateHipPose(const Eigen::Ref<const Eigen::Vector3d>& foot_print)
{
    // consider theta rotation around y-axis
    Eigen::Matrix4d hipPose = Eigen::Matrix4d::Identity();
    Common::rotxyz(M_PI / 2, foot_print[2], 0, hipPose.block<3, 3>(0, 0));

    // Add x and y translation and fixed hip height
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    translation << foot_print[0], foot_print[1], _length.at("Hip_length");
    hipPose.block<3, 1>(0, 3) = translation;

    return hipPose;
}

Eigen::Matrix4d Human::calculateNeckpose(const Eigen::Ref<const Eigen::Matrix4d>& hip_pose, const double hip_angle)
{

    //    Eigen::Matrix3d Rx = Common::rotx(hip_angle).block<3, 3>(0, 0);
    //    Eigen::Vector3d translationZ(0, 0, (length.at("Neck_length") + 1.0 * radius.at("Neck_radius") + 0.0 * radius.at("Hip_radius")));

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation                 = Common::rotx(hip_angle);
    Eigen::Vector3d translation;
    translation << 0, _length.at("Neck_length"), 0;
    Eigen::Matrix4d transTform   = Eigen::Matrix4d::Identity();
    transTform.block<3, 1>(0, 3) = translation;

    //    transformation.block<3, 3>(0, 0) = Rx;
    //    transformation.block<3, 1>(0, 3) = translationZ;
    //    double thet{M_PI / 2 + hip_angle};
    //    double alpha{0};
    //    double d{0};
    //    double a{-length.at("Neck_length")};
    //    transformation << cos(thet), -sin(thet) * cos(alpha), sin(thet) * sin(alpha), a * cos(thet), sin(thet), cos(thet) * cos(alpha),
    //        -cos(thet) * sin(alpha), a * sin(thet), 0, sin(alpha), cos(alpha), d, 0, 0, 0, 1;

    // std::cout << "Transformation Matrix: " << transformation << "\n for angle: " << hip_angle << std::endl;

    Eigen::Matrix4d neckPose = Eigen::Matrix4d::Identity();
    neckPose                 = hip_pose * transformation * transTform;
    //    std::cout << "Hip given:" << hip_pose << " \n" << std::endl;

    //    std::cout << "Neck calculated:" << neckPose << " \n" << std::endl;

    return neckPose;
}

Eigen::Matrix4d Human::calculateHeadPose(const Eigen::Ref<const Eigen::Matrix4d>& neck_pose, const double neck_angle)
{

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation                 = Common::rotx(neck_angle);
    Eigen::Vector3d translation;
    translation << 0, _radius.at("Head_radius"), 0;
    Eigen::Matrix4d transTform   = Eigen::Matrix4d::Identity();
    transTform.block<3, 1>(0, 3) = translation;

    Eigen::Matrix4d headPose = Eigen::Matrix4d::Identity();
    headPose                 = neck_pose * transformation * transTform;

    //    std::cout << "Head calculated:" << headPose << " \n" << std::endl;
    return headPose;
}

Eigen::Matrix4d Human::calculateLeftShoulderPose(const Eigen::Ref<const Eigen::Matrix4d>& hip_pose,
                                                 const Eigen::Ref<const Eigen::Vector3d>& shoulder_angle, const double hip_angle)
{

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation                 = Common::rotx(hip_angle);
    //    transformation.block<3, 3>(0, 0) = hip_pose.block<3, 3>(0, 0);

    Eigen::Vector3d translation;
    translation << _radius.at("Neck_radius") + _radius.at("LUArm_radius"), _length.at("Neck_length"), 0;
    Eigen::Matrix4d transTform   = Eigen::Matrix4d::Identity();
    transTform.block<3, 1>(0, 3) = translation;

    Eigen::Matrix3d firstRotation  = Common::rotz(-M_PI / 2).block<3, 3>(0, 0);
    Eigen::Matrix4d secondRotation = Common::roty(-shoulder_angle[0]);
    Eigen::Matrix4d thirdRotation  = Common::rotz(shoulder_angle[1]);
    Eigen::Matrix4d fourthRotation = Common::rotx(shoulder_angle[2]);

    Eigen::Matrix4d shoulderPose = Eigen::Matrix4d::Identity();
    //    shoulderPose = hip_pose * transformation * transTform * (secondRotation * thirdRotation * fourthRotation);
    shoulderPose = hip_pose * transformation * transTform * Common::rotz(-M_PI / 2) * secondRotation * thirdRotation * fourthRotation;
    return shoulderPose;
}

Eigen::Matrix4d Human::calculateLeftElbowPose(const Eigen::Ref<const Eigen::Matrix4d>& shoulder_pose, const double elbow_angle)
{
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    //    Eigen::Matrix4d transformation2 = Eigen::Matrix4d::Identity();
    transformation = Common::roty(-elbow_angle);

    Eigen::Vector3d translation;
    translation << _length.at("LUArm_length"), 0, 0;

    Eigen::Matrix4d transTform   = Eigen::Matrix4d::Identity();
    transTform.block<3, 1>(0, 3) = translation;

    Eigen::Matrix4d elbowPose = Eigen::Matrix4d::Identity();
    elbowPose                 = shoulder_pose * transTform * transformation;
    //    std::cout << "Left Elbow Position: " << elbowPose.block<3, 1>(0, 3) << "\n" << std ::endl;
    return elbowPose;
}

Eigen::Matrix4d Human::calculateLeftHandPose(const Eigen::Ref<const Eigen::Matrix4d>& elbow_pose)
{
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();

    Eigen::Vector3d translation;
    translation << _length.at("LFArm_length"), 0, 0;

    Eigen::Matrix4d transTform   = Eigen::Matrix4d::Identity();
    transTform.block<3, 1>(0, 3) = translation;

    Eigen::Matrix4d handPose = Eigen::Matrix4d::Identity();
    handPose                 = elbow_pose * transTform;

    return handPose;
}

Eigen::Matrix4d Human::calculateRightShoulderPose(const Eigen::Ref<const Eigen::Matrix4d>& hip_pose,
                                                  const Eigen::Ref<const Eigen::Vector3d>& shoulder_angle, const double hip_angle)
{
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation                 = Common::rotx(hip_angle);

    Eigen::Vector3d translation;
    translation << -(_radius.at("Neck_radius") + _radius.at("RUArm_radius")), _length.at("Neck_length"), 0;
    Eigen::Matrix4d transTform   = Eigen::Matrix4d::Identity();
    transTform.block<3, 1>(0, 3) = translation;

    Eigen::Matrix3d firstRotation  = Common::rotz(M_PI / 2).block<3, 3>(0, 0);
    Eigen::Matrix4d secondRotation = Common::roty(shoulder_angle[0]);
    Eigen::Matrix4d thirdRotation  = Common::rotz(-shoulder_angle[1]);
    Eigen::Matrix4d fourthRotation = Common::rotx(-shoulder_angle[2]);

    Eigen::Matrix4d shoulderPose = Eigen::Matrix4d::Identity();
    shoulderPose                 = hip_pose * transformation * transTform * Common::rotz(M_PI / 2) * secondRotation * thirdRotation * fourthRotation;

    //    std::cout << "Head calculated:" << headPose << " \n" << std::endl;
    return shoulderPose;
}

Eigen::Matrix4d Human::calculateRightElbowPose(const Eigen::Ref<const Eigen::Matrix4d>& shoulder_pose, const double elbow_angle)
{
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation                 = Common::roty(elbow_angle);

    Eigen::Vector3d translation;
    translation << -(_length.at("RUArm_length")), 0, 0;

    Eigen::Matrix4d transTform   = Eigen::Matrix4d::Identity();
    transTform.block<3, 1>(0, 3) = translation;

    Eigen::Matrix4d elbowPose = Eigen::Matrix4d::Identity();
    elbowPose                 = shoulder_pose * transTform * transformation;
    //    std::cout << "Right Elbow Position: " << elbowPose.block<3, 1>(0, 3) << "\n" << std ::endl;
    return elbowPose;
}

Eigen::Matrix4d Human::calculateRightHandPose(const Eigen::Ref<const Eigen::Matrix4d>& elbow_pose)
{
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    //    Eigen::Matrix4d transformation2 = Eigen::Matrix4d::Identity();
    Eigen::Vector3d translation;
    translation << -_length.at("RFArm_length"), 0, 0;

    Eigen::Matrix4d transTform   = Eigen::Matrix4d::Identity();
    transTform.block<3, 1>(0, 3) = translation;

    Eigen::Matrix4d handPose = Eigen::Matrix4d::Identity();
    handPose                 = elbow_pose * transTform;

    return handPose;
}

std::pair<int, Obstacle> Human::getNewUncertaintyBodyPart(const std::string& name)
{
    Obstacle newBodyPart;
    newBodyPart.group_id = _id;
    newBodyPart.id       = _id_count;
    _id_count++;
    newBodyPart.name = name;

    newBodyPart.bounding_box = getBodyPart(name).bounding_box;
    newBodyPart.state        = getBodyPart(name).state;
    return {_id_count, newBodyPart};
}
}  // namespace robot_misc
}  // namespace mhp_robot
