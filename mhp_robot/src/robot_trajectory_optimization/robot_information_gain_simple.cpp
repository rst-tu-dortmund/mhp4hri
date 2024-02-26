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

#include <mhp_robot/robot_trajectory_optimization/robot_information_gain_simple.h>

namespace mhp_robot {
namespace robot_trajectory_optimization {

double RobotInformationGainSimple::computeGain(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k)
{
    double factor = 1;
    double gain   = 0;
    if (_information_pcl.size() > 0)  // be sure we already received a point cloud
    {
        // get the transformation from depth camera to world for joint configuration x_k
        Eigen::Matrix4d T = _robot_kinematic->getEndEffectorMatrix(x_k) * _tf_cam_to_ee_link;

        // Transform POI to world frame
        Eigen::Vector4d poi_world{1.19, 0.16, 0.33, 1.0};
        Eigen::Vector4d z_axis{0, 0, 1, 1};
        Eigen::Vector4d z_axis_world = T * z_axis;
        // get the scalar product between the camera and the POI axis
        double scalar_product = Eigen::Vector3d{poi_world[0] - T(0, 3), poi_world[1] - T(1, 3), poi_world[2] - T(2, 3)}.dot(
                                    Eigen::Vector3d{z_axis_world[0] - T(0, 3), z_axis_world[1] - T(1, 3), z_axis_world[2] - T(2, 3)}) /
                                (Eigen::Vector3d{poi_world[0] - T(0, 3), poi_world[1] - T(1, 3), poi_world[2] - T(2, 3)}.norm() *
                                 Eigen::Vector3d{z_axis_world[0] - T(0, 3), z_axis_world[1] - T(1, 3), z_axis_world[2] - T(2, 3)}.norm());
        // set multiplication factor to 0 if the scalara product relates to an angle outside of the FOV (horizontal --> Azure camera 37.5° in each
        // direction) ~ 0.79
        if (scalar_product < 0.79)
        {
            factor = 0;
        }
        else
        {
            factor = scalar_product;
        }
        // if ((T.block<3, 1>(0, 3) - poi_world.block<3, 1>(0, 0)).norm() > 0.5)
        // {
        //     gain = 0;
        // }
        // else
        // {
        inverseDistanceWeigthing(T.block<3, 1>(0, 3), gain);
        // }
        // std::cout << "Factor: " << factor << std::endl;
        // std::cout << "Gain: " << gain << std::endl;
        // std::cout << "Gain with factor: " << 1 / (factor * gain + _eps) << std::endl;
    }

    return _w_gain / (factor * gain + _eps);  // add small epsilon to avoid division by zero
}

void RobotInformationGainSimple::inverseDistanceWeigthing(const Eigen::Ref<const Eigen::Vector3d>& point, double& gain)
{
    // get the distance to the point cloud points (with Eigen Matrix for vectorized operations)
    if(_first_pcl && _information_pcl.size()>0){
    Eigen::MatrixXd pcl_points = Eigen::MatrixXd::Zero(3, _information_pcl.size());
    for (int i = 0; i < _information_pcl.size(); ++i)
    {
        pcl_points.col(i) = Eigen::Vector3d{_information_pcl.points[i].x, _information_pcl.points[i].y, _information_pcl.points[i].z};
    }
    // Eigen::MatrixXd pcl_points = _information_pcl.getMatrixXfMap(3, 4, 0).cast<double>();

    Eigen::VectorXd dists = (pcl_points.colwise() - point).colwise().norm();

    int p         = 2;  // power of the inverse distance weighting
    dists.array() = dists.array().pow(-p);
    _weighted_dists = dists.array() / dists.sum();
    _first_pcl = false;
    }
    // get the inverse distance weighting

    gain          = 0;

    for (int i = 0; i < _information_pcl.size(); ++i)
    {
        gain += (_weighted_dists(i)) * _information_pcl.points[i].intensity;
    }
}

}  // namespace robot_trajectory_optimization
}  // namespace mhp_robot