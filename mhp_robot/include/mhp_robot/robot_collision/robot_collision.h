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

#ifndef ROBOT_COLLISION_H
#define ROBOT_COLLISION_H

#include <mhp_robot/robot_description/robot_description.h>
#include <mhp_robot/robot_kinematic/robot_kinematic.h>
#include <mhp_robot/robot_misc/obstacle.h>
#include <mhp_robot/robot_misc/plane.h>

namespace mhp_robot {
namespace robot_collision {

class RobotCollision
{
 public:
    using Ptr  = std::shared_ptr<RobotCollision>;
    using UPtr = std::unique_ptr<RobotCollision>;

    RobotCollision(robot_kinematic::RobotKinematic::Ptr robot_kinematic, int obstacle_dim, int plane_dim, int self_collision_dim);

    RobotCollision(const RobotCollision&)            = delete;
    RobotCollision(RobotCollision&&)                 = default;
    RobotCollision& operator=(const RobotCollision&) = delete;
    RobotCollision& operator=(RobotCollision&&)      = default;
    virtual ~RobotCollision() {}

    virtual UPtr createUniqueInstance() const = 0;
    virtual Ptr createSharedInstance() const  = 0;

    robot_kinematic::RobotKinematic::Ptr getRobotKinematic();

    // Segment - Segment
    bool getDistanceBetweenLinks(double& d, int j, int k) const;
    bool getDistanceBetweenLinks(double& d, int j, int k, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                 Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point) const;

    // Segment - Obstacle
    bool getDistanceToObstacle(double& d, int j, const robot_misc::Obstacle& obstacle, double t = 0, double t_radius = 0) const;
    bool getDistanceToObstacle(double& d, int j, const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                               Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0, double t_radius = 0) const;
    bool getDistanceToUncertaintyObstacle(double& d, int j, const robot_misc::Obstacle& obstacle, double t = 0, int iterator = 0) const;
    bool getDistanceToUncertaintyObstacle(double& d, int j, const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                          Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0, int iterator = 0) const;

    // Segment - Plane
    bool getDistanceToPlane(double& d, int j, const robot_misc::Plane& plane) const;
    bool getDistanceToPlane(double& d, int j, const robot_misc::Plane& plane, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                            Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point) const;

    // Obstacle - Obstacle
    static bool getDistanceBetweenObstacles(double& d, const robot_misc::Obstacle& obstacle1, const robot_misc::Obstacle& obstacle2, double t = 0);
    static bool getDistanceBetweenObstacles(double& d, const robot_misc::Obstacle& obstacle1, const robot_misc::Obstacle& obstacle2,
                                            Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point, Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point,
                                            double t = 0);

    // Obstacle - Plane
    static bool getDistanceBetweenPlaneObstacle(double& d, const robot_misc::Plane& plane, const robot_misc::Obstacle& obstacle, double t = 0);
    static bool getDistanceBetweenPlaneObstacle(double& d, const robot_misc::Plane& plane, const robot_misc::Obstacle& obstacle,
                                                Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0);

    // Obstacle - Human
    static bool getDistanceBetweenHumanObstacle(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, const robot_misc::Obstacle& obstacle,
                                                const robot_misc::Human& human, double t = 0);
    static bool getDistanceBetweenHumanObstacle(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, const robot_misc::Obstacle& obstacle,
                                                const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                                Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t = 0);

    static bool getMinDistanceBetweenHumanObstacle(double& d, const robot_misc::Obstacle& obstacle, const robot_misc::Human& human, double t = 0);
    static bool getMinDistanceBetweenHumanObstacle(double& d, const robot_misc::Obstacle& obstacle, const robot_misc::Human& human,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0);

    // Robot - Robot
    virtual bool getSelfCollisionDistances(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d) const          = 0;
    virtual bool getSelfCollisionDistances(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                           Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points) const = 0;

    virtual double getMinSelfCollisionDistance() const;
    virtual double getMinSelfCollisionDistance(Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                               Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point) const;

    // Robot - Obstacle
    virtual bool getObstacleDistances(const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                      double t_radius = 0) const                                                                             = 0;
    virtual bool getObstacleDistances(const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                      Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                      double t = 0, double t_radius = 0) const                                                               = 0;
    virtual bool getUncertaintyObstacleDistances(const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                 int interator = 0) const                                                                    = 0;
    virtual bool getUncertaintyObstacleDistances(const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                 Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                                 Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t = 0, int interator = 0) const = 0;

    virtual double getMinObstacleDistance(const robot_misc::Obstacle& obstacle, double t = 0) const;
    virtual double getMinObstacleDistance(const robot_misc::Obstacle& obstacle, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                          Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0) const;

    // Robot - Human
    virtual void getHumanDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual void getHumanDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                 double t_radius = 0) const;
    virtual void getHumanDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                   Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                   double t = 0) const;

    virtual void getMinHumanDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual void getMinHumanDistanceRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                   double t_radius = 0) const;

    virtual void getMinHumanDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                     Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                     double t = 0) const;
    virtual void getMinHumanDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                     Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                     std::vector<int>& segments, double t = 0) const;

    // Robot - Trunk
    virtual void getHumanTrunkDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual void getHumanTrunkDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                           const int uncertainty_instance, double t = 0) const;

    virtual void getHumanTrunkDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                      double t_radius = 0) const;
    virtual void getHumanTrunkDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                        Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                        double t = 0) const;
    virtual void getHumanUncertaintyTrunkDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;

    virtual void getMinHumanTrunkDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual void getMinHumanTrunkDistanceUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                             const int uncertainty_instance, double t = 0) const;
    virtual void getMinHumanTrunkDistanceRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                        double t_radius = 0) const;
    virtual void getMinHumanTrunkDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                          Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                          double t = 0) const;
    virtual void getMinHumanUncertaintyTrunkDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;

    // Robot - Limbs
    virtual void getHumanLimbsDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual void getHumanLimbsDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                           const int uncertainty_instance, double t = 0) const;

    virtual void getHumanLimbsDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                      double t_radius = 0) const;
    virtual void getHumanLimbsDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                        Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                        double t = 0) const;
    virtual void getHumanUncertaintyLimbsDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;

    virtual void getMinHumanLimbsDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual void getMinHumanLimbsDistanceUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                             const int uncertainty_instance, double t = 0) const;

    virtual void getMinHumanLimbsDistanceRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                        double t_radius = 0) const;
    virtual void getMinHumanLimbsDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                          Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                          double t = 0) const;
    virtual void getMinHumanUncertaintyLimbsDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;

    // Robot - Human (Head)
    virtual bool getHumanHeadDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual bool getHumanHeadDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                          const int uncertainty_instance, double t = 0) const;
    virtual bool getHumanHeadDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                     double t_radius = 0) const;
    virtual bool getHumanHeadDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                       Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                       double t = 0) const;
    virtual bool getHumanUncertaintyHeadDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;

    virtual double getMinHumanHeadDistance(const robot_misc::Human& human, double t = 0) const;
    virtual double getMinHumanHeadDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance, double t = 0) const;

    virtual double getMinHumanHeadDistanceRadiusIncrease(const robot_misc::Human& human, double t = 0, double t_radius = 0) const;
    virtual double getMinHumanHeadDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                           Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0) const;
    virtual double getMinHumanHeadDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                           Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t = 0) const;
    virtual double getMinHumanUncertaintyHeadDistance(const robot_misc::Human& human, double t) const;

    // Robot - Human (Thorso)
    virtual bool getHumanThorsoDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual bool getHumanThorsoDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                            const int uncertainty_instance, double t = 0) const;

    virtual bool getHumanThorsoDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                       double t_radius = 0) const;
    virtual bool getHumanThorsoDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                         Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                         double t = 0) const;
    virtual bool getHumanUncertaintyThorsoDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d, double t = 0) const;

    virtual double getMinHumanThorsoDistance(const robot_misc::Human& human, double t = 0) const;
    virtual double getMinHumanThorsoDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance, double t = 0) const;
    virtual double getMinHumanThorsoDistanceRadiusIncrease(const robot_misc::Human& human, double t = 0, double t_radius = 0) const;
    virtual double getMinHumanThorsoDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                             Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0) const;
    virtual double getMinHumanThorsoDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                             Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t = 0) const;
    virtual double getMinHumanUncertaintyThorsoDistance(const robot_misc::Human& human, double t = 0) const;

    // Robot - Human (Pelvis)
    virtual bool getHumanPelvisDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual bool getHumanPelvisDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                            const int uncertainty_instance, double t = 0) const;
    virtual bool getHumanPelvisDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                       double t_radius = 0) const;
    virtual bool getHumanPelvisDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                         Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                         double t = 0) const;
    virtual bool getHumanUncertaintyPelvisDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d, double t = 0) const;

    virtual double getMinHumanPelvisDistance(const robot_misc::Human& human, double t = 0) const;
    virtual double getMinHumanPelvisDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance, double t = 0) const;
    virtual double getMinHumanPelvisDistanceRadiusIncrease(const robot_misc::Human& human, double t = 0, double t_radius = 0) const;
    virtual double getMinHumanPelvisDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                             Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0) const;
    virtual double getMinHumanPelvisDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                             Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t = 0) const;
    virtual double getMinHumanUncertaintyPelvisDistance(const robot_misc::Human& human, double t = 0) const;

    // Robot - Human (Upper Arm Right)
    virtual bool getHumanUpperArmRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual bool getHumanUpperArmRightDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                                   const int uncertainty_instance, double t = 0) const;
    virtual bool getHumanUpperArmRightDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                              double t = 0, double t_radius = 0) const;
    virtual bool getHumanUpperArmRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                                Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t = 0) const;
    virtual bool getHumanUncertaintyUpperArmRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d,
                                                           double t = 0) const;

    virtual double getMinHumanUpperArmRightDistance(const robot_misc::Human& human, double t = 0) const;
    virtual double getMinHumanUpperArmRightDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance,
                                                                       double t = 0) const;
    virtual double getMinHumanUpperArmRightDistanceRadiusIncrease(const robot_misc::Human& human, double t = 0, double t_radius = 0) const;
    virtual double getMinHumanUpperArmRightDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                    Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0) const;
    virtual double getMinHumanUpperArmRightDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                    Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t = 0) const;
    virtual double getMinHumanUncertaintyUpperArmRightDistance(const robot_misc::Human& human, double t = 0) const;

    // Robot - Human (Lower Arm Right)
    virtual bool getHumanLowerArmRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual bool getHumanLowerArmRightDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                                   const int uncertainty_instance, double t = 0) const;
    virtual bool getHumanLowerArmRightDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                              double t = 0, double t_radius = 0) const;
    virtual bool getHumanLowerArmRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                                Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t = 0) const;
    virtual bool getHumanUncertaintyLowerArmRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d,
                                                           double t = 0) const;

    virtual double getMinHumanLowerArmRightDistance(const robot_misc::Human& human, double t = 0) const;
    virtual double getMinHumanLowerArmRightDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance,
                                                                       double t = 0) const;
    virtual double getMinHumanLowerArmRightDistanceRadiusIncrease(const robot_misc::Human& human, double t = 0, double t_radius = 0) const;
    virtual double getMinHumanLowerArmRightDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                    Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0) const;
    virtual double getMinHumanLowerArmRightDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                    Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t = 0) const;
    virtual double getMinHumanUncertaintyLowerArmRightDistance(const robot_misc::Human& human, double t = 0) const;

    // Robot - Human (Hand Right)
    virtual bool getHumanHandRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual bool getHumanHandRightDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                          double t_radius = 0) const;
    virtual bool getHumanHandRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                            Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                            Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t = 0) const;
    virtual bool getHumanUncertaintyHandRightDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d,
                                                       double t = 0) const;

    virtual double getMinHumanHandRightDistance(const robot_misc::Human& human, double t = 0) const;
    virtual double getMinHumanHandRightDistanceRadiusIncrease(const robot_misc::Human& human, double t = 0, double t_radius = 0) const;
    virtual double getMinHumanHandRightDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0) const;
    virtual double getMinHumanHandRightDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t = 0) const;
    virtual double getMinHumanUncertaintyHandRightDistance(const robot_misc::Human& human, double t = 0) const;

    // Robot - Human (Upper Arm Left)
    virtual bool getHumanUpperArmLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual bool getHumanUpperArmLeftDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                                  const int uncertainty_instance, double t = 0) const;
    virtual bool getHumanUpperArmLeftDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                             double t_radius = 0) const;
    virtual bool getHumanUpperArmLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                               Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                               Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t = 0) const;
    virtual bool getHumanUncertaintyUpperArmLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d,
                                                          double t = 0) const;

    virtual double getMinHumanUpperArmLeftDistance(const robot_misc::Human& human, double t = 0) const;
    virtual double getMinHumanUpperArmLeftDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance,
                                                                      double t = 0) const;
    virtual double getMinHumanUpperArmLeftDistanceRadiusIncrease(const robot_misc::Human& human, double t = 0, double t_radius = 0) const;
    virtual double getMinHumanUpperArmLeftDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0) const;
    virtual double getMinHumanUpperArmLeftDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t = 0) const;
    virtual double getMinHumanUncertaintyUpperArmLeftDistance(const robot_misc::Human& human, double t = 0) const;

    // Robot - Human (Lower Arm Left)
    virtual bool getHumanLowerArmLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual bool getHumanLowerArmLeftDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                                  const int uncertainty_instance, double t = 0) const;
    virtual bool getHumanLowerArmLeftDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                             double t_radius = 0) const;
    virtual bool getHumanLowerArmLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                               Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                               Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points, double t = 0) const;
    virtual bool getHumanUncertaintyLowerArmLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d,
                                                          double t = 0) const;

    virtual double getMinHumanLowerArmLeftDistance(const robot_misc::Human& human, double t = 0) const;
    virtual double getMinHumanLowerArmLeftDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance,
                                                                      double t = 0) const;
    virtual double getMinHumanLowerArmLeftDistanceRadiusIncrease(const robot_misc::Human& human, double t = 0, double t_radius = 0) const;
    virtual double getMinHumanLowerArmLeftDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0) const;
    virtual double getMinHumanLowerArmLeftDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                   Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t = 0) const;
    virtual double getMinHumanUncertaintyLowerArmLeftDistance(const robot_misc::Human& human, double t = 0) const;

    // Robot - Human (Hand Left)
    virtual bool getHumanHandLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual bool getHumanHandLeftDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                         double t_radius = 0) const;
    virtual bool getHumanHandLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                           Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                           double t = 0) const;
    virtual bool getHumanUncertaintyHandLeftDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d,
                                                      double t = 0) const;

    virtual double getMinHumanHandLeftDistance(const robot_misc::Human& human, double t = 0) const;
    virtual double getMinHumanHandLeftDistanceRadiusIncrease(const robot_misc::Human& human, double t = 0, double t_radius = 0) const;
    virtual double getMinHumanHandLeftDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                               Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0) const;
    virtual double getMinHumanHandLeftDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                               Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t = 0) const;
    virtual double getMinHumanUncertaintyHandLeftDistance(const robot_misc::Human& human, double t = 0) const;

    // Robot - Human (Leg)
    virtual bool getHumanLegDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0) const;
    virtual bool getHumanLegDistancesUncertaintyInstance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                                         const int uncertainty_instance, double t = 0) const;
    virtual bool getHumanLegDistancesRadiusIncrease(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, double t = 0,
                                                    double t_radius = 0) const;
    virtual bool getHumanLegDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                      Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points, Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points,
                                      double t = 0) const;
    virtual bool getHumanUncertaintyLegDistances(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, -1, -1>> d, double t = 0) const;

    virtual double getMinHumanLegDistance(const robot_misc::Human& human, double t = 0) const;
    virtual double getMinHumanLegDistanceUncertaintyInstance(const robot_misc::Human& human, const int uncertainty_instance, double t = 0) const;
    virtual double getMinHumanLegDistanceRadiusIncrease(const robot_misc::Human& human, double t = 0, double t_radius = 0) const;
    virtual double getMinHumanLegDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                          Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, double t = 0) const;
    virtual double getMinHumanLegDistance(const robot_misc::Human& human, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                          Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point, int& segment, double t = 0) const;
    virtual double getMinHumanUncertaintyLegDistance(const robot_misc::Human& human, double t = 0) const;

    // Robot - Ground
    virtual bool getGroundCollisionDistances(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d) const;
    virtual bool getGroundCollisionDistances(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                             Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points) const;

    virtual double getMinGroundCollisionDistance() const;
    virtual double getMinGroundCollisionDistance(Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                 Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point) const;

    // Robot - Roof
    virtual bool getRoofCollisionDistances(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d) const;
    virtual bool getRoofCollisionDistances(Eigen::Ref<Eigen::Matrix<double, -1, 1>> d, Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                           Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points) const;

    virtual double getMinRoofCollisionDistance() const;
    virtual double getMinRoofCollisionDistance(Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                               Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point) const;

    // Robot - Plane
    virtual bool getPlaneCollisionDistances(const robot_misc::Plane& plane, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d) const = 0;
    virtual bool getPlaneCollisionDistances(const robot_misc::Plane& plane, Eigen::Ref<Eigen::Matrix<double, -1, 1>> d,
                                            Eigen::Ref<Eigen::Matrix<double, 3, -1>> start_points,
                                            Eigen::Ref<Eigen::Matrix<double, 3, -1>> end_points) const                        = 0;

    virtual double getMinPlaneCollisionDistance(const robot_misc::Plane& plane) const;
    virtual double getMinPlaneCollisionDistance(const robot_misc::Plane& plane, Eigen::Ref<Eigen::Matrix<double, 3, 1>> start_point,
                                                Eigen::Ref<Eigen::Matrix<double, 3, 1>> end_point) const;

    void setJointState(const Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>& q);

    // Point to X
    static double getPointToPointDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p,
                                          const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1,
                                          Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2);

    static double getPointToLineDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p,
                                         const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                         const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1,
                                         Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2);

    static double getPointToRectangleDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p1,
                                              const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                              const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2,
                                              const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q3, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1,
                                              Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2);

    static double getPointToExtrudedRectangleDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p1,
                                                      const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                                      const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2,
                                                      const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q3,
                                                      Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2);

    static double getPointToPlaneDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p,
                                          const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q,
                                          const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& n, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1,
                                          Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2);

    // Line to X
    static double getLineToLineDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p1,
                                        const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p2,
                                        const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                        const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1,
                                        Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2);

    // This function is fastest(117ns) vs.MB(152ns) and GTE(234ns), but it does not fully support when s xor t is in unit intervall
    static double getLineToRectangleDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p1,
                                             const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p2,
                                             const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                             const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2,
                                             const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q3, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1,
                                             Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2);

    static double getLineToExtrudedRectangleDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p1,
                                                     const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p2,
                                                     const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                                     const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2,
                                                     const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q3,
                                                     Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2);

    static double getLineToPlaneDistance(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p1,
                                         const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p2,
                                         const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q,
                                         const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& n, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1,
                                         Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2);

    // MB Variants implemented by Fabian Menebroeker and modified for speed by Maximilian Kraemer
    // TODO This function does not work correctly for some cases. The reason might be the closest point assumption.
    static double getLineToRectangleDistanceMB(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p1,
                                               const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& p2,
                                               const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q1,
                                               const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q2,
                                               const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& q3, Eigen::Ref<Eigen::Matrix<double, 3, 1>> l1,
                                               Eigen::Ref<Eigen::Matrix<double, 3, 1>> l2);

    int _obstacle_distance_dimension        = 0;
    int _plane_collision_distance_dimension = 0;
    int _self_collision_distance_dimension  = 0;
    int _human_collision_distance_dimension = 0;

 protected:
    using RobotKinematic = robot_kinematic::RobotKinematic;

    RobotKinematic::Ptr _robot_kinematic;

    std::vector<robot_misc::Segment> _segment_structs;
    robot_misc::Plane _ground_plane, _roof_plane;
};

}  // namespace robot_collision
}  // namespace mhp_robot

#endif  // ROBOT_COLLISION_H
