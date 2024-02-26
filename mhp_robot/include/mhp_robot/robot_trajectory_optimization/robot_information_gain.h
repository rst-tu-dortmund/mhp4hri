/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2024,
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
 *  Authors: Maximilian Krämer, Heiko Renz
 *  Maintainer(s)/Modifier(s):
 *********************************************************************/

#ifndef ROBOT_INFORMATION_GAIN_H
#define ROBOT_INFORMATION_GAIN_H

#include <mhp_robot/robot_kinematic/robot_kinematic.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
namespace mhp_robot {
namespace robot_trajectory_optimization {

class RobotInformationGain
{
 public:
    using Ptr  = std::shared_ptr<RobotInformationGain>;
    using UPtr = std::unique_ptr<RobotInformationGain>;

    RobotInformationGain() = default;

    RobotInformationGain(const RobotInformationGain&)            = delete;
    RobotInformationGain(RobotInformationGain&&)                 = delete;
    RobotInformationGain& operator=(const RobotInformationGain&) = delete;
    RobotInformationGain& operator=(RobotInformationGain&&)      = delete;
    virtual ~RobotInformationGain() {}

    virtual double computeGain(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k) = 0;

    virtual void computeGradient(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> dx);

    virtual void computeHessian(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::MatrixXd> dxdx);

    bool initialize(robot_kinematic::RobotKinematic::UPtr robot_kinematic);
    bool update(double dt);
    bool isInitialized() const;

    void setPlannerId(const int id)
    {
        _planner_id = id;
        if (_planner_id != 0) _ms_planner_mode = true;
    }
    int getPlannerId() const { return _planner_id; }
    bool isPlannerSet() const { return _ms_planner_mode; }

 protected:
    using RobotKinematic = robot_kinematic::RobotKinematic;

    ros::Subscriber _information_gain_sub;
    pcl::PointCloud<pcl::PointXYZI> _information_pcl;

    Eigen::Matrix4d       _tf_cam_to_ee_link = Eigen::Matrix4d::Identity();
    RobotKinematic::UPtr _robot_kinematic;

    double _dt  = 0.1;
    double _eps = 1e-7;
    double _w_gain = 1.0;
    bool _initialized = false;

    void informationGainCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    // Variables for Multistage Planner
    int _planner_id       = 0;
    bool _ms_planner_mode = false;

    enum UncertaintyMode { NoUncertaintyEstimation, SkeletonSplitting, RadiusIncrease } _uncertainty_mode = NoUncertaintyEstimation;
};

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot

#endif  // ROBOT_INFORMATION_GAIN_H
