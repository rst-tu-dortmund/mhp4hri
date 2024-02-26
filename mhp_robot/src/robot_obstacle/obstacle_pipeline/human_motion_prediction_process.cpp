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
#include <mhp_robot/robot_obstacle/obstacle_pipeline/human_motion_prediction_process.h>
#include <fstream>
namespace mhp_robot {
namespace robot_obstacle {
namespace obstacle_pipeline {

HumanMotionPredictionProcess::HumanMotionPredictionProcess(const std::string& name) : BaseProcess(name)
{
    _prediction_subscriber = _nh.subscribe("/rel_orient_prediction", 1, &HumanMotionPredictionProcess::callback_prediction, this);
    //    _dist_listener         = nh->subscribe("/ur_distance_publisher/minimum_distances", 1000, &HumanMotionPredictionProcess::callback_distance,
    //    this);
}

bool HumanMotionPredictionProcess::process(std::map<int, robot_misc::Obstacle>& static_obstacles,
                                           std::map<int, robot_misc::Obstacle>& dynamic_obstacles, std::map<int, robot_misc::Human>& humans,
                                           std::map<int, robot_misc::UtilityObject>& utility_objects, std::map<int, robot_misc::Plane>& planes,
                                           bool forced)
{
    if (!_initialized)
    {
        ROS_WARN("HumanMotionPredictionProcess: Cannot process in an uninitialized state.");
        return false;
    }

    if (_active || forced)
    {
        // Check if an prediction message is already prepared
        if (_prediction_available_flag)
        {
            // Iterate over all humans
            for (auto& human : humans)
            {
                if (_future_poses.size() > 0)
                {
                    Eigen::Matrix<double, 9, Eigen::Dynamic> posDist = Eigen::MatrixXd::Zero(9, static_cast<long>(_future_poses.size()));
                    Eigen::Matrix<double, 9, Eigen::Dynamic> rotDist = Eigen::MatrixXd::Zero(9, static_cast<long>(_future_poses.size()));
                    Eigen::Vector3d hipPosition                      = human.second._body_parts.at("Hip").state._poses.at(0).block<3, 1>(0, 3);

                    // Get the prediction which is the closest to the current pose
                    int nearestPrediction = nearest_neighbour(human.second, hipPosition, posDist, rotDist);

                    // Iterate over all body parts of the human
                    for (auto& bdy : human.second._body_parts)
                    {
                        // clean current body part predictions
                        bdy.second.state._poses.erase(bdy.second.state._poses.begin() + 1, bdy.second.state._poses.end());

                        // Start with the Hip to extra update the footprints
                        if (bdy.second.name == "Hip")
                        {
                            for (int k = 1; k <= 30; ++k)
                            {
                                //                                human.second.updateFutureHipPoses(k, 30);
                            }
                        }

                        // Iterate over all Predictions which are available and fill the body part state poses
                        for (int i = nearestPrediction; i < static_cast<int>(_future_poses.size()); i++)
                        {
                            if (human.second._body_parts.at("Hip").state._poses.size() - 1 >= i)
                            {
                                hipPosition = human.second._body_parts.at("Hip").state._poses.at(i).block<3, 1>(0, 3);
                            }
                            else
                            {
                                hipPosition = human.second._body_parts.at("Hip").state._poses.back().block<3, 1>(0, 3);
                            }
                            // Decide which body part we have and fill  up the state poses.
                            // Attetnion The Hip Position have to be added because the prediction is relative with the hip in the origin
                            switch (bdy.second.id)
                            {
                                case 0: {  // Attention for the hip the position is always 0!
                                    Eigen::Matrix4d pose   = _future_poses.at(i).block<4, 4>(0, 0);
                                    pose.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3) + hipPosition;
                                    bdy.second.state._poses.push_back(pose);
                                    break;
                                }
                                case 1: {
                                    Eigen::Matrix4d pose   = _future_poses.at(i).block<4, 4>(4, 0);
                                    pose.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3) + hipPosition;
                                    bdy.second.state._poses.push_back(pose);
                                    break;
                                }
                                case 2: {
                                    Eigen::Matrix4d pose   = _future_poses.at(i).block<4, 4>(8, 0);
                                    pose.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3) + hipPosition;
                                    bdy.second.state._poses.push_back(pose);
                                    break;
                                }
                                case 3: {
                                    Eigen::Matrix4d pose   = _future_poses.at(i).block<4, 4>(12, 0);
                                    pose.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3) + hipPosition;
                                    bdy.second.state._poses.push_back(pose);
                                    break;
                                }
                                case 4: {
                                    Eigen::Matrix4d pose   = _future_poses.at(i).block<4, 4>(20, 0);
                                    pose.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3) + hipPosition;
                                    bdy.second.state._poses.push_back(pose);
                                    break;
                                }
                                case 5: {
                                    Eigen::Matrix4d pose = _future_poses.at(i).block<4, 4>(28, 0);
                                    //                                    std::cout << "Body Part name: " << bdy.second.name << "\n" << std::endl;
                                    //                                    std::cout << "LHand Position from index " << i << ": " << pose << "\n"
                                    //                                    << std::endl;
                                    pose.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3) + hipPosition;
                                    //                                    std::cout << "LHand Position from index after hip correct " << i << ": "
                                    //                                    << pose << "\n" << std::endl;
                                    bdy.second.state._poses.push_back(pose);

                                    break;
                                }
                                case 6: {
                                    Eigen::Matrix4d pose   = _future_poses.at(i).block<4, 4>(16, 0);
                                    pose.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3) + hipPosition;
                                    bdy.second.state._poses.push_back(pose);
                                    break;
                                }
                                case 7: {
                                    Eigen::Matrix4d pose   = _future_poses.at(i).block<4, 4>(24, 0);
                                    pose.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3) + hipPosition;
                                    bdy.second.state._poses.push_back(pose);
                                    break;
                                }
                                case 8: {
                                    Eigen::Matrix4d pose = _future_poses.at(i).block<4, 4>(32, 0);
                                    //                            std::cout << "set this pose : " << pose << "\n" << std::endl;
                                    pose.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3) + hipPosition;

                                    bdy.second.state._poses.push_back(pose);
                                    //                            std::cout << "after setting pose: " << bdy.second.state._poses[i + 1] << "\n" <<
                                    //                            std::endl;
                                    break;
                                }
                                case 9: {
                                    Eigen::Matrix4d pose   = _future_poses.at(i).block<4, 4>(0, 0);
                                    pose.block<3, 1>(0, 3) = pose.block<3, 1>(0, 3) + hipPosition;
                                    bdy.second.state._poses.push_back(pose);
                                    break;
                                }
                                default:
                                    std::cout << "In default \n" << std::endl;
                                    ROS_WARN_ONCE("No matching joint!");
                                    break;
                            }
                        }

                        while (bdy.second.state._poses.size() < (_prediction_time_planner * 10))
                        {
                            switch (bdy.second.id)
                            {
                                case 0: {
                                    //                                Eigen::Matrix4d lastPose = bdy.second.state._poses.back();
                                    bdy.second.state._poses.push_back(bdy.second.state._poses.back());
                                    break;
                                }
                                case 1: {
                                    bdy.second.state._poses.push_back(bdy.second.state._poses.back());
                                    break;
                                }
                                case 2: {
                                    bdy.second.state._poses.push_back(bdy.second.state._poses.back());
                                    break;
                                }
                                case 3: {
                                    bdy.second.state._poses.push_back(bdy.second.state._poses.back());
                                    break;
                                }
                                case 4: {
                                    bdy.second.state._poses.push_back(bdy.second.state._poses.back());
                                    break;
                                }
                                case 5: {
                                    bdy.second.state._poses.push_back(bdy.second.state._poses.back());
                                    break;
                                }
                                case 6: {
                                    bdy.second.state._poses.push_back(bdy.second.state._poses.back());
                                    break;
                                }
                                case 7: {
                                    bdy.second.state._poses.push_back(bdy.second.state._poses.back());
                                    break;
                                }
                                case 8: {
                                    bdy.second.state._poses.push_back(bdy.second.state._poses.back());
                                    break;
                                }
                                case 9: {
                                    bdy.second.state._poses.push_back(bdy.second.state._poses.back());
                                    break;
                                }
                                default:
                                    ROS_WARN_ONCE("Human_Motion_Predictor_Process:No matching joint!");
                                    break;
                            }
                        }
                    }
                }
            }
        }
    }
    return true;
}

void HumanMotionPredictionProcess::callback_prediction(const MsgRelativeOrientation& msg)
{
    // Getting the amount of predictions
    unsigned long amountPred = msg.data.size() / 4 / 4 / 9;
    std::vector<double> args(msg.data.begin(), msg.data.end());
    _future_poses.clear();

    size_t elemCounter = 0;
    // Fill in the Matrix based on the vector of input data
    for (unsigned long i = 0; i < amountPred; i++)
    {
        Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(36, 4);
        for (int j = 0; j < 36; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                tmp(j, k) = msg.data.at(elemCounter);
                elemCounter += 1;
            }
        }
        _future_poses.insert({i, tmp});
    };
    _prediction_available_flag = 1;
}

void HumanMotionPredictionProcess::callback_distance(const MsgDistances& msg)
{
    std::vector<double> humanDistances = msg.human_distances;
    std::ofstream distFile;
    distFile.open("test_mit.txt", std::ios_base::app);
    for (auto elem : humanDistances)
    {
        std::cout << "All Human Distances: " << elem << "\n" << std::endl;
        distFile << elem << " ";
    }
    distFile << "\n";
    distFile.close();
}

int HumanMotionPredictionProcess::nearest_neighbour(const robot_misc::Human& human, const Eigen::Ref<Eigen::Vector3d> hip_position,
                                                    Eigen::Ref<Eigen::MatrixXd> pos_dist, Eigen::Ref<Eigen::MatrixXd> rot_dist)
{

    int predAmount       = static_cast<int>(_future_poses.size());
    int NearestNeighbour = 0;

    // Iterate over all body parts
    for (auto& bdy : human._body_parts)
    {
        Eigen::Matrix4d currentPose     = bdy.second.state.getPose();
        int bdy_id                      = bdy.second.id;
        Eigen::Matrix3d currentRotation = currentPose.block<3, 3>(0, 0);
        Eigen::Vector3d currentPosition = currentPose.block<3, 1>(0, 3);
        //    std::cout << "Current Position: " << currentPosition << "\n and Current Rotation: " << currentRotation << "\n" << std::endl;

        // Iterate over all prediction steps
        for (int i = 0; i < predAmount; i++)
        {
            // Id: 8with name: RHand
            // Id: 7with name: RFArm
            // Id: 6with name: RUArm
            // Id: 1with name: Neck
            // Id: 2with name: Head
            // Id: 3with name: LUArm
            // Id: 4with name: LFArm
            // Id: 0with name: Hip
            // Id: 5with name: LHand
            // Not the same order as in the prediction is necessary for the body parts
            // order from prediction (saved in _future_poses) is hip - neck - head - LUArm - RUArm - LFArm - RFArm - LHand - RHand

            // Calculate the euclidean distance between the prediction and the current position for every joint
            switch (bdy_id)
            {
                case 0: {  // Attention for the hip the position is always 0!
                    Eigen::Matrix4d pose = _future_poses.at(i).block<4, 4>(0, 0);
                    Eigen::Matrix3d rot  = pose.block<3, 3>(0, 0);
                    Eigen::Vector3d pos  = pose.block<3, 1>(0, 3) + hip_position;

                    Eigen::Matrix3d diffRot = currentRotation.transpose() * rot;
                    if (diffRot.trace() > 3)
                    {
                        rot_dist(bdy_id, i) = std::acos(((3 - 1) / 2));
                    }
                    else
                    {
                        rot_dist(bdy_id, i) = std::acos(((diffRot.trace() - 1) / 2));
                    }
                    pos_dist(bdy_id, i) = (pos - currentPosition).norm();

                    break;
                }
                case 1: {
                    Eigen::Matrix4d pose = _future_poses.at(i).block<4, 4>(4, 0);
                    Eigen::Matrix3d rot  = pose.block<3, 3>(0, 0);
                    Eigen::Vector3d pos  = pose.block<3, 1>(0, 3) + hip_position;

                    Eigen::Matrix3d diffRot = currentRotation.transpose() * rot;
                    if (diffRot.trace() > 3)
                    {
                        rot_dist(bdy_id, i) = std::acos(((3 - 1) / 2));
                    }
                    else
                    {
                        rot_dist(bdy_id, i) = std::acos(((diffRot.trace() - 1) / 2));
                    }
                    pos_dist(bdy_id, i) = (pos - currentPosition).norm();
                    break;
                }
                case 2: {
                    Eigen::Matrix4d pose = _future_poses.at(i).block<4, 4>(8, 0);
                    Eigen::Matrix3d rot  = pose.block<3, 3>(0, 0);
                    Eigen::Vector3d pos  = pose.block<3, 1>(0, 3) + hip_position;

                    Eigen::Matrix3d diffRot = currentRotation.transpose() * rot;
                    if (diffRot.trace() > 3)
                    {

                        rot_dist(bdy_id, i) = std::acos(((3 - 1) / 2));
                    }
                    else
                    {
                        rot_dist(bdy_id, i) = std::acos(((diffRot.trace() - 1) / 2));
                    }
                    pos_dist(bdy_id, i) = (pos - currentPosition).norm();
                    break;
                }
                case 3: {
                    Eigen::Matrix4d pose = _future_poses.at(i).block<4, 4>(12, 0);
                    Eigen::Matrix3d rot  = pose.block<3, 3>(0, 0);
                    Eigen::Vector3d pos  = pose.block<3, 1>(0, 3) + hip_position;

                    Eigen::Matrix3d diffRot = currentRotation.transpose() * rot;
                    if (diffRot.trace() > 3)
                    {
                        rot_dist(bdy_id, i) = std::acos(((3 - 1) / 2));
                    }
                    else
                    {
                        rot_dist(bdy_id, i) = std::acos(((diffRot.trace() - 1) / 2));
                    }
                    pos_dist(bdy_id, i) = (pos - currentPosition).norm();
                    break;
                }
                case 4: {
                    Eigen::Matrix4d pose    = _future_poses.at(i).block<4, 4>(20, 0);
                    Eigen::Matrix3d rot     = pose.block<3, 3>(0, 0);
                    Eigen::Vector3d pos     = pose.block<3, 1>(0, 3) + hip_position;
                    Eigen::Matrix3d diffRot = currentRotation.transpose() * rot;
                    if (diffRot.trace() > 3)
                    {
                        rot_dist(bdy_id, i) = std::acos(((3 - 1) / 2));
                    }
                    else
                    {
                        rot_dist(bdy_id, i) = std::acos(((diffRot.trace() - 1) / 2));
                    }
                    pos_dist(bdy_id, i) = (pos - currentPosition).norm();
                    break;
                }
                case 5: {
                    Eigen::Matrix4d pose = _future_poses.at(i).block<4, 4>(28, 0);
                    Eigen::Matrix3d rot  = pose.block<3, 3>(0, 0);
                    Eigen::Vector3d pos  = pose.block<3, 1>(0, 3) + hip_position;

                    Eigen::Matrix3d diffRot = currentRotation.transpose() * rot;
                    if (diffRot.trace() > 3)
                    {
                        rot_dist(bdy_id, i) = std::acos(((3 - 1) / 2));
                    }
                    else
                    {
                        rot_dist(bdy_id, i) = std::acos(((diffRot.trace() - 1) / 2));
                    }
                    pos_dist(bdy_id, i) = (pos - currentPosition).norm();
                    break;
                }
                case 6: {
                    Eigen::Matrix4d pose = _future_poses.at(i).block<4, 4>(16, 0);
                    Eigen::Matrix3d rot  = pose.block<3, 3>(0, 0);
                    Eigen::Vector3d pos  = pose.block<3, 1>(0, 3) + hip_position;

                    Eigen::Matrix3d diffRot = currentRotation.transpose() * rot;
                    if (diffRot.trace() > 3)
                    {
                        rot_dist(bdy_id, i) = std::acos(((3 - 1) / 2));
                    }
                    else
                    {
                        rot_dist(bdy_id, i) = std::acos(((diffRot.trace() - 1) / 2));
                    }
                    pos_dist(bdy_id, i) = (pos - currentPosition).norm();
                    break;
                }
                case 7: {
                    Eigen::Matrix4d pose = _future_poses.at(i).block<4, 4>(24, 0);
                    Eigen::Matrix3d rot  = pose.block<3, 3>(0, 0);
                    Eigen::Vector3d pos  = pose.block<3, 1>(0, 3) + hip_position;

                    Eigen::Matrix3d diffRot = currentRotation.transpose() * rot;
                    if (diffRot.trace() > 3)
                    {
                        rot_dist(bdy_id, i) = std::acos(((3 - 1) / 2));
                    }
                    else
                    {
                        rot_dist(bdy_id, i) = std::acos(((diffRot.trace() - 1) / 2));
                    }
                    pos_dist(bdy_id, i) = (pos - currentPosition).norm();
                    break;
                }
                case 8: {
                    Eigen::Matrix4d pose = _future_poses.at(i).block<4, 4>(32, 0);
                    Eigen::Matrix3d rot  = pose.block<3, 3>(0, 0);
                    Eigen::Vector3d pos  = pose.block<3, 1>(0, 3) + hip_position;

                    Eigen::Matrix3d diffRot = currentRotation.transpose() * rot;
                    if (diffRot.trace() > 3)
                    {
                        rot_dist(bdy_id, i) = std::acos(((3 - 1) / 2));
                    }
                    else
                    {
                        rot_dist(bdy_id, i) = std::acos(((diffRot.trace() - 1) / 2));
                    }
                    pos_dist(bdy_id, i) = (pos - currentPosition).norm();
                    break;
                }

                case 9: {
                    break;
                }
                default:
                    ROS_WARN("No matching joint!");
                    break;
            }
        }
    }

    Eigen::MatrixXd::Index MinIdxPos[9], MinIdxRot[9];
    Eigen::VectorXd MinValPos(9), MinValRot(9);
    std::vector<int> v(static_cast<size_t>(predAmount), 0);
    // Check which prediction is most frequently the closest to the current position and return the prediction number
    for (int k = 0; k < 9; ++k)
    {
        MinValPos(k) = pos_dist.row(k).minCoeff(&MinIdxPos[k]);
        v[MinIdxPos[k]] += 1;

        // Rotation Difference is weighted 1/4 of the Position Difference
        MinValRot(k) = rot_dist.row(k).minCoeff(&MinIdxRot[k]);
        v[MinIdxRot[k]] += 0.25;
    }
    NearestNeighbour = std::max_element(v.begin(), v.end()) - v.begin();
    return NearestNeighbour;
}

}  // namespace obstacle_pipeline
}  // namespace robot_obstacle
}  // namespace mhp_robot
