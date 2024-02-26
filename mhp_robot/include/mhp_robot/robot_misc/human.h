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

#ifndef HUMAN_H
#define HUMAN_H

#include <mhp_robot/robot_misc/obstacle.h>
#include <tf/transform_listener.h>
#include <unordered_map>

namespace mhp_robot {
namespace robot_misc {

class Human
{
 public:
    Human(int ID, std::string HumanName = {"Human"}, std::vector<double> lengthLinks = {0.95, 0.6, 0.0, 0.3, 0.3, 0.0, 0.3, 0.3, 0.0},
          std::vector<double> radiusLinks = {0.2, 0.2, 0.12, 0.06, 0.06, 0.0, 0.06, 0.06, 0.0},
          std::vector<double> angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, Eigen::Vector3d foot_print = {0.5, 1, 0});

    Eigen::Vector<double, 11> _lower_angle_limit = Eigen::Vector<double, 11>::Zero();
    Eigen::Vector<double, 11> _upper_angle_limit = Eigen::Vector<double, 11>::Zero();

    std::unordered_map<std::string, Obstacle> _body_parts;

    std::vector<double> _joint_angles;
    Eigen::Vector<double, 11> _joint_velocities    = Eigen::Vector<double, 11>::Zero();
    Eigen::Vector<double, 11> _joint_accelerations = Eigen::Vector<double, 11>::Zero();
    /* hip,head,left_shoulder1 (front,back),left_shoulder2 (to the side),left_intern,left_elbow,
     * right_shoulder1 (front,back),right_shoulder2 (to the side),right_intern, right_elbow, legs*/

    std::vector<Eigen::Vector<double, 11>> _all_extrapolated_angles;

    Eigen::Vector3d _foot_print              = Eigen::Vector3d::Zero();
    Eigen::Vector3d _foot_print_velocity     = Eigen::Vector3d::Zero();
    Eigen::Vector3d _foot_print_acceleration = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> _future_foot_prints;

    std::unordered_map<std::string, double> _length = {{"Hip_length", 0.95},   {"Neck_length", 0.55}, {"Head_length", 0.0},  {"LUArm_length", 0.3},
                                                       {"LFArm_length", 0.36}, {"LHand_length", 0.0}, {"RUArm_length", 0.3}, {"RFArm_length", 0.36},
                                                       {"RHand_length", 0.0},  {"Leg_length", 0.0}};

    std::unordered_map<std::string, double> _radius = {{"Hip_radius", 0.2},    {"Neck_radius", 0.2},  {"Head_radius", 0.12},  {"LUArm_radius", 0.06},
                                                       {"LFArm_radius", 0.06}, {"LHand_radius", 0.0}, {"RUArm_radius", 0.06}, {"RFArm_radius", 0.06},
                                                       {"RHand_radius", 0.0},  {"Leg_radius", 0.0}};
    std::vector<std::string> _uncertainty_body_part_names = {"Neck",  "Head",  "LUArm", "LUArm", "LUArm", "LFArm",
                                                             "RUArm", "RUArm", "RUArm", "RFArm", "Leg"};

    Eigen::Matrix<double, 3, 3> _rot_hip = Eigen::Matrix<double, 3, 3>::Identity();

    int _id           = -1;  // MoCap assigns arbitrary IDs to skeletons automatically. One has to set the human ID in the yaml file accordingly.
    std::string _name = "Human";
    bool _gazebo      = false;
    static constexpr const int _body_part_count = 10;
    static constexpr const int _body_parts_size = 7;
    int _id_count                               = _body_part_count;

    bool _poses_updated       = false;
    bool _automatic_simulator = false;
    bool _with_leg            = true;
    const Obstacle& getBodyPart(const std::string& _name) const;

    // IK Functions
    void inverseKinematic();
    void inverseKinematicTf();

    // For stable IK results required to check and initialize Body lengths
    void initializeBodyLength();

    // Update Poses in body part obstacles
    void updatePoses(const Eigen::Ref<const Eigen::Vector3d>& foot_print);
    std::vector<Eigen::Matrix4d> returnPoses(const Eigen::Ref<const Eigen::Vector3d>& foot_print, const Eigen::VectorXd angles);
    void updateFutureHipPoses(const int futureStep, const int allSteps);
    void updateFuturePoses(const Eigen::Ref<Eigen::Vector<double, 11>> angles, int futureStep, int allSteps);
    void updatePosesFromTf(tf::TransformListener& listener);

    // Functions for uncertainty estimation addition
    void updateObstaclePose(Obstacle& obs, const std::string bodyPart, const Eigen::Ref<const Eigen::Vector<double, 11>> angles,
                            const size_t timeStep);
    void addUncertaintyObstacles(std::vector<Eigen::MatrixXd>& angles, int timeStep, std::vector<int>& splitIdx);
    void deleteUncertaintyObstacles();
    int getNumberUncertaintyHead() const { return getBodyPart("Head").uncertainty_states.size(); }
    int getNumberUncertaintyNeck() const { return getBodyPart("Neck").uncertainty_states.size(); }
    int getNumberUncertaintyLUArm() const { return getBodyPart("LUArm").uncertainty_states.size(); }
    int getNumberUncertaintyLFArm() const { return getBodyPart("LFArm").uncertainty_states.size(); }
    int getNumberUncertaintyLHand() const { return getBodyPart("LHand").uncertainty_states.size(); }
    int getNumberUncertaintyRUArm() const { return getBodyPart("RUArm").uncertainty_states.size(); }
    int getNumberUncertaintyRFArm() const { return getBodyPart("RFArm").uncertainty_states.size(); }
    int getNumberUncertaintyRHand() const { return getBodyPart("RHand").uncertainty_states.size(); }
    int getNumberUncertaintyLeg() const { return getBodyPart("Leg").uncertainty_states.size(); }

    // Helper function for human builder (calibration tool)
    void rewriteBoundingBoxes();

    // Joint angle check
    void checkJointLimits(Eigen::Vector<double, 11>& check_angles);

    Eigen::VectorXd interpolateVectors(const Eigen::Ref<const Eigen::VectorXd>& vec1, const Eigen::Ref<const Eigen::VectorXd>& vec2, double scaling);
    double _dt = 0.1;

    std::vector<std::pair<int, int>> _Neck_ids;
    std::vector<std::pair<int, int>> _Head_ids;
    std::vector<std::pair<int, int>> _LUArm_ids;
    std::vector<std::pair<int, int>> _LFArm_ids;
    std::vector<std::pair<int, int>> _LHand_ids;
    std::vector<std::pair<int, int>> _RUArm_ids;
    std::vector<std::pair<int, int>> _RFArm_ids;
    std::vector<std::pair<int, int>> _RHand_ids;
    std::vector<std::pair<int, int>> _Leg_ids;
    std::vector<std::string> _distance_body_part_names{"Neck", "Head", "LUArm", "LFArm", "RUArm", "RFArm", "Leg"};

    bool _skeleton_splitting = false;
    int _uncertainty_mode    = 0;

    Eigen::VectorXi
        _ms_mapping;  // assuming sorted ms planners--> _ms_mapping[0] belongs to planner 2 (0 is not set,1 is default controller) and so on
    std::vector<std::pair<int, std::vector<double>>> _all_uncertainty_angles;

 private:
    // IK function for each type
    void calculateShoulderAngle(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_shoulder,
                                const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_elbow,
                                const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_hand, bool flag_right);
    void calculateHipAngle(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_hip,
                           const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_neck);
    void calculateElbowAngle(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_hand,
                             const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_shoulder, bool flag_left);
    void calculateHeadAngle(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_head,
                            const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_neck);
    void calculateShoulderAngleTf(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_shoulder,
                                  const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_elbow,
                                  const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_hand, bool flag_right);
    void calculateHipAngleTf(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_hip,
                             const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_neck);
    void calculateElbowAngleTf(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_hand,
                               const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_shoulder, bool flag_left);
    void calculateHeadAngleTf(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_head,
                              const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& pose_neck);
    Eigen::Matrix4d calculateHipPose(const Eigen::Ref<const Eigen::Vector3d>& foot_print);
    Eigen::Matrix4d calculateNeckpose(const Eigen::Ref<const Eigen::Matrix4d>& hip_pose, const double hip_angle);
    Eigen::Matrix4d calculateHeadPose(const Eigen::Ref<const Eigen::Matrix4d>& neck_pose, const double neck_angle);
    Eigen::Matrix4d calculateLeftShoulderPose(const Eigen::Ref<const Eigen::Matrix4d>& hip_pose,
                                              const Eigen::Ref<const Eigen::Vector3d>& shoulder_angle, const double hip_angle);
    Eigen::Matrix4d calculateLeftElbowPose(const Eigen::Ref<const Eigen::Matrix4d>& shoulder_pose, const double elbow_angle);
    Eigen::Matrix4d calculateLeftHandPose(const Eigen::Ref<const Eigen::Matrix4d>& elbow_pose);
    Eigen::Matrix4d calculateRightShoulderPose(const Eigen::Ref<const Eigen::Matrix4d>& hip_pose,
                                               const Eigen::Ref<const Eigen::Vector3d>& shoulder_angle, const double hip_angle);
    Eigen::Matrix4d calculateRightElbowPose(const Eigen::Ref<const Eigen::Matrix4d>& shoulder_pose, const double elbow_angle);
    Eigen::Matrix4d calculateRightHandPose(const Eigen::Ref<const Eigen::Matrix4d>& elbow_pose);

    std::pair<int, Obstacle> getNewUncertaintyBodyPart(const std::string& name);
};
}  // namespace robot_misc
}  // namespace mhp_robot

#endif  // HUMAN_H
