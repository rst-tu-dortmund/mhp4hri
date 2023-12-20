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
#include <mhp_robot/robot_obstacle/obstacle_trajectory_generator.h>

namespace mhp_robot {
namespace robot_obstacle {

robot_misc::ObstacleTrajectory ObstacleTrajectoryGenerator::generateLinearMotion(const Eigen::Ref<const Eigen::Matrix4d>& p1,
                                                                                 const Eigen::Ref<const Eigen::Matrix4d>& p2, int samples,
                                                                                 double duration)
{
    robot_misc::ObstacleTrajectory trajectory;
    generateLinearMotion(p1, p2, samples, duration, trajectory);

    return trajectory;
}

void ObstacleTrajectoryGenerator::generateLinearMotion(const Eigen::Ref<const Eigen::Matrix4d>& p1, const Eigen::Ref<const Eigen::Matrix4d>& p2,
                                                       int samples, double duration, robot_misc::ObstacleTrajectory& trajectory)
{
    trajectory.poses = std::vector<Eigen::Matrix4d>(samples, Eigen::Matrix4d::Identity());
    Eigen::Quaterniond q1(p1.block<3, 3>(0, 0));
    Eigen::Quaterniond q2(p2.block<3, 3>(0, 0));
    Eigen::Quaterniond q;

    for (int i = 0; i < samples; ++i)
    {
        q                                     = q1.slerp((double)i / (samples - 1), q2);
        trajectory.poses[i].block<3, 3>(0, 0) = q.toRotationMatrix();
        trajectory.poses[i].block<3, 1>(0, 3) = p1.block<3, 1>(0, 3) + (p2.block<3, 1>(0, 3) - p1.block<3, 1>(0, 3)) * ((double)i / (samples - 1));
        trajectory.timestamps.push_back(((double)i / (samples - 1)) * duration);
    }
}

robot_misc::ObstacleTrajectory ObstacleTrajectoryGenerator::generateCircularMotion(const Eigen::Ref<const Eigen::Matrix4d>& p1,
                                                                                   const Eigen::Ref<const Eigen::Matrix4d>& p2,
                                                                                   const Eigen::Ref<const Eigen::Matrix4d>& p3, int samples,
                                                                                   double duration)
{
    robot_misc::ObstacleTrajectory trajectory;
    generateCircularMotion(p1, p2, p3, samples, duration, trajectory);

    return trajectory;
}

void ObstacleTrajectoryGenerator::generateCircularMotion(const Eigen::Ref<const Eigen::Matrix4d>& p1, const Eigen::Ref<const Eigen::Matrix4d>& p2,
                                                         const Eigen::Ref<const Eigen::Matrix4d>& p3, int samples, double duration,
                                                         robot_misc::ObstacleTrajectory& trajectory)
{
    // Create plane
    Eigen::Vector3d v1, v2;
    v1 = p2.block<3, 1>(0, 3) - p1.block<3, 1>(0, 3);
    v2 = p3.block<3, 1>(0, 3) - p1.block<3, 1>(0, 3);

    // Transformation into the plane
    Eigen::Matrix4d T;
    T.setIdentity();
    T.block<3, 1>(0, 3) = p1.block<3, 1>(0, 3);
    T.block<3, 1>(0, 0) = v1.normalized();
    T.block<3, 1>(0, 1) = ((v1.cross(v2)).cross(v1)).normalized();
    T.block<3, 1>(0, 2) = (v1.cross(v2)).normalized();

    // Transform points onto the plane
    Eigen::Vector4d a1, a2, a3;
    a1.noalias() = T.inverse() * p1.block<4, 1>(0, 3);
    a2.noalias() = T.inverse() * p2.block<4, 1>(0, 3);
    a3.noalias() = T.inverse() * p3.block<4, 1>(0, 3);

    // Solve circle equations
    Eigen::Matrix3d regressor;
    Eigen::Vector3d m;
    m << -(a1(0) * a1(0) + a1(1) * a1(1)), -(a2(0) * a2(0) + a2(1) * a2(1)), -(a3(0) * a3(0) + a3(1) * a3(1));
    regressor.block<3, 1>(0, 0) << 1, 1, 1;
    regressor.block<3, 1>(0, 1) << -a1(0), -a2(0), -a3(0);
    regressor.block<3, 1>(0, 2) << -a1(1), -a2(1), -a3(1);

    Eigen::HouseholderQR<Eigen::MatrixXd> qr(regressor);
    Eigen::Vector3d s = qr.solve(m);

    // Circle parameters
    double xm = s(1) / 2.0;
    double ym = s(2) / 2.0;
    double r  = std::sqrt(xm * xm + ym * ym - s(0));

    // Create trajectory
    trajectory.poses = std::vector<Eigen::Matrix4d>(samples, Eigen::Matrix4d::Identity());

    Eigen::Quaterniond q1(p1.block<3, 3>(0, 0));
    Eigen::Quaterniond q2(p3.block<3, 3>(0, 0));
    Eigen::Quaterniond q;
    Eigen::Vector4d p;
    p.setZero();
    double theta = 0;

    for (int i = 0; i < samples; ++i)
    {
        q     = q1.slerp((double)i / (samples - 1), q2);
        theta = i * 2 * M_PI / (samples - 1);
        p(0)  = xm + r * std::cos(theta);
        p(1)  = ym + r * std::sin(theta);
        p(3)  = 1;

        trajectory.poses[i].block<3, 1>(0, 3).noalias() = (T * p).head<3>();
        trajectory.poses[i].block<3, 3>(0, 0)           = q.toRotationMatrix();
        trajectory.timestamps.push_back(((double)i / (samples - 1)) * duration);
    }
}

robot_misc::HumanTrajectory ObstacleTrajectoryGenerator::generateHumanMotion(const Eigen::Ref<const Eigen::VectorXd>& q1,
                                                                             const Eigen::Ref<const Eigen::VectorXd>& q2,
                                                                             const Eigen::Ref<const Eigen::Vector3d>& f1,
                                                                             const Eigen::Ref<const Eigen::Vector3d>& f2, int samples,
                                                                             double duration, bool returnPath)
{
    robot_misc::HumanTrajectory trajectory;

    generateHumanMotion(q1, q2, f1, f2, samples, duration, trajectory, returnPath);

    return trajectory;
}

void ObstacleTrajectoryGenerator::generateHumanMotion(const Eigen::Ref<const Eigen::VectorXd>& q1, const Eigen::Ref<const Eigen::VectorXd>& q2,
                                                      const Eigen::Ref<const Eigen::Vector3d>& f1, const Eigen::Ref<const Eigen::Vector3d>& f2,
                                                      int samples, double duration, robot_misc::HumanTrajectory& trajectory, bool returnPath)
{

    if (returnPath)
    {
        trajectory.footprints     = std::vector<Eigen::Vector3d>(2 * samples, Eigen::Vector3d::Zero());
        trajectory.configurations = std::vector<Eigen::VectorXd>(2 * samples, Eigen::Vector<double, 11>::Zero());
        for (int i = 0; i < samples; ++i)
        {
            trajectory.footprints[i]     = f1 + (f2 - f1) * ((double)i / (samples - 1));
            trajectory.configurations[i] = q1 + (q2 - q1) * ((double)i / (samples - 1));
            trajectory.timestamps.push_back(((double)i / (2 * samples - 1)) * (2 * duration));
        }
        for (int i = samples; i < 2 * samples; ++i)
        {
            trajectory.footprints[i]     = f2 + (f1 - f2) * (((double)i - samples) / (samples - 1));
            trajectory.configurations[i] = q2 + (q1 - q2) * (((double)i - samples) / (samples - 1));
            trajectory.timestamps.push_back(((double)i / (2 * samples - 1)) * (2 * duration));
        }
    }
    else
    {
        trajectory.footprints     = std::vector<Eigen::Vector3d>(samples, Eigen::Vector3d::Zero());
        trajectory.configurations = std::vector<Eigen::VectorXd>(samples, Eigen::Vector<double, 11>::Zero());
        for (int i = 0; i < samples; ++i)
        {
            trajectory.footprints[i]     = f1 + (f2 - f1) * ((double)i / (samples - 1));
            trajectory.configurations[i] = q1 + (q2 - q1) * ((double)i / (samples - 1));
            trajectory.timestamps.push_back(((double)i / (samples - 1)) * duration);
        }
    }
}
std::map<int, robot_misc::ObstacleTrajectory> ObstacleTrajectoryGenerator::parseObstacleTrajectories(const std::string param)
{
    ros::NodeHandle n("~");

    // load trajectories from parameter server
    XmlRpc::XmlRpcValue obstacle_param;
    n.getParam(param, obstacle_param);
    // parse
    return createObstacleTrajectoryMap(obstacle_param);
}

std::map<int, robot_misc::HumanTrajectory> ObstacleTrajectoryGenerator::parseHumanTrajectories(const std::string param)
{
    ros::NodeHandle n("~");

    // load trajectories from parameter server
    XmlRpc::XmlRpcValue human_param;
    n.getParam(param, human_param);
    // parse
    return createHumanTrajectoryMap(human_param);
}

std::map<int, robot_misc::ObstacleTrajectory> ObstacleTrajectoryGenerator::createObstacleTrajectoryMap(const XmlRpc::XmlRpcValue& param)
{
    std::map<int, robot_misc::ObstacleTrajectory> list;

    if (param.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        // Iterate over trajectories
        for (int i = 0; i < param.size(); ++i)
        {

            const XmlRpc::XmlRpcValue& traj_el = param[i];

            if (traj_el.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                robot_misc::ObstacleTrajectory trajectory;
                bool duration = false, steps = false, id = false, p1 = false, p2 = false, p3 = false;

                Eigen::Matrix3d Rx, Ry, Rz;
                Eigen::Matrix4d T1, T2, T3;
                T1.setIdentity();
                T2.setIdentity();
                T3.setIdentity();
                double d = 0.0;
                int n    = 0;

                // Iterate over properties
                for (XmlRpc::XmlRpcValue::const_iterator prop_it = traj_el.begin(); prop_it != traj_el.end(); ++prop_it)
                {
                    if (prop_it->first == "duration" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    {
                        d        = prop_it->second;
                        duration = true;
                    }
                    if (prop_it->first == "steps" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
                    {
                        n     = prop_it->second;
                        steps = true;
                    }
                    if (prop_it->first == "t1" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray && prop_it->second.size() == 3)
                    {
                        T1.block<4, 1>(0, 3) << prop_it->second[0], prop_it->second[1], prop_it->second[2], 1.0;
                        p1 = true;
                    }
                    if (prop_it->first == "r1" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray && prop_it->second.size() == 3)
                    {
                        Rz                             = robot_misc::Common::rotz(prop_it->second[0]).block<3, 3>(0, 0);
                        Ry                             = robot_misc::Common::roty(prop_it->second[1]).block<3, 3>(0, 0);
                        Rx                             = robot_misc::Common::rotx(prop_it->second[2]).block<3, 3>(0, 0);
                        T1.block<3, 3>(0, 0).noalias() = Rz * Ry * Rx;
                        p1                             = true;
                    }
                    if (prop_it->first == "t2" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray && prop_it->second.size() == 3)
                    {
                        T2.block<4, 1>(0, 3) << prop_it->second[0], prop_it->second[1], prop_it->second[2], 1.0;
                        p2 = true;
                    }
                    if (prop_it->first == "r2" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray && prop_it->second.size() == 3)
                    {
                        Rz                             = robot_misc::Common::rotz(prop_it->second[0]).block<3, 3>(0, 0);
                        Ry                             = robot_misc::Common::roty(prop_it->second[1]).block<3, 3>(0, 0);
                        Rx                             = robot_misc::Common::rotx(prop_it->second[2]).block<3, 3>(0, 0);
                        T2.block<3, 3>(0, 0).noalias() = Rz * Ry * Rx;
                        p2                             = true;
                    }
                    if (prop_it->first == "t3" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray && prop_it->second.size() == 3)
                    {
                        T3.block<4, 1>(0, 3) << prop_it->second[0], prop_it->second[1], prop_it->second[2], 1.0;
                        p3 = true;
                    }
                    if (prop_it->first == "r3" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray && prop_it->second.size() == 3)
                    {
                        Rz                             = robot_misc::Common::rotz(prop_it->second[0]).block<3, 3>(0, 0);
                        Ry                             = robot_misc::Common::roty(prop_it->second[1]).block<3, 3>(0, 0);
                        Rx                             = robot_misc::Common::rotx(prop_it->second[2]).block<3, 3>(0, 0);
                        T3.block<3, 3>(0, 0).noalias() = Rz * Ry * Rx;
                        p3                             = true;
                    }
                    if (prop_it->first == "obstacle_id" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
                    {
                        trajectory.id = prop_it->second;
                        id            = true;
                    }
                    if (prop_it->first == "loop" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                    {
                        trajectory.loop = prop_it->second;
                    }
                }

                // evaluating properties
                if (duration && steps && id)
                {
                    // Check whether a trajectory for the same ID already on the map
                    auto traj_iter = std::find_if(list.begin(), list.end(), [&](const std::tuple<int, robot_misc::ObstacleTrajectory>& traj) {
                        return trajectory.id == std::get<1>(traj).id;
                    });

                    if (traj_iter == list.end())
                    {
                        if (p1 && p2 && p3)  // circular
                        {
                            generateCircularMotion(T1, T2, T3, n, d, trajectory);
                        }
                        else  // linear
                        {
                            generateLinearMotion(T1, T2, n, d, trajectory);
                        }

                        list.insert(std::pair<int, robot_misc::ObstacleTrajectory>(trajectory.id, trajectory));
                    }
                    else
                    {
                        ROS_WARN("ObstacleTrajectoryGenerator: Trajectory for ID %d already exists. Trajectory skipped.", trajectory.id);
                    }
                }
                else
                {
                    ROS_WARN("ObstacleTrajectoryGenerator: Not all trajectory properties were set. Trajectory skipped.");
                }
            }
            else
            {
                ROS_WARN("ObstacleTrajectoryGenerator: Invalid trajectory format. Trajectory skipped.");
            }
        }
    }
    else
    {
        ROS_WARN("ObstacleTrajectoryGenerator: No obstacle trajectories found.");
    }

    return list;
}

std::map<int, robot_misc::HumanTrajectory> ObstacleTrajectoryGenerator::createHumanTrajectoryMap(const XmlRpc::XmlRpcValue& param)
{
    std::map<int, robot_misc::HumanTrajectory> list;
    if (param.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        // Iterate over trajectories
        for (int i = 0; i < param.size(); ++i)
        {

            const XmlRpc::XmlRpcValue& traj_el = param[i];

            if (traj_el.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                robot_misc::HumanTrajectory trajectory;
                bool duration = false, steps = false, id = false, f1 = false, f2 = false, q1 = false, q2 = false;

                Eigen::Vector3d foot1 = Eigen::Vector3d::Zero(), foot2 = Eigen::Vector3d::Zero();
                Eigen::VectorXd angle1 = Eigen::Vector<double, 11>::Zero(), angle2 = Eigen::Vector<double, 11>::Zero();

                double d = 0.0;
                int n    = 0;

                // Iterate over properties
                for (XmlRpc::XmlRpcValue::const_iterator prop_it = traj_el.begin(); prop_it != traj_el.end(); ++prop_it)
                {
                    if (prop_it->first == "duration" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    {
                        d        = prop_it->second;
                        duration = true;
                    }
                    if (prop_it->first == "steps" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
                    {
                        n     = prop_it->second;
                        steps = true;
                    }
                    if (prop_it->first == "human_id" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeInt)
                    {
                        trajectory.id = prop_it->second;
                        id            = true;
                    }
                    if (prop_it->first == "loop" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                    {
                        trajectory.loop = prop_it->second;
                    }
                    if (prop_it->first == "return" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                    {
                        trajectory.returnPath = prop_it->second;
                    }
                    if (prop_it->first == "foot1" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray)
                    {
                        foot1 << prop_it->second[0], prop_it->second[1], prop_it->second[2];
                        f1 = true;
                    }
                    if (prop_it->first == "foot2" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray)
                    {
                        foot2 << prop_it->second[0], prop_it->second[1], prop_it->second[2];
                        f2 = true;
                    }
                    if (prop_it->first == "angles1" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray)
                    {
                        angle1 << prop_it->second[0], prop_it->second[1], prop_it->second[2], prop_it->second[3], prop_it->second[4],
                            prop_it->second[5], prop_it->second[6], prop_it->second[7], prop_it->second[8], prop_it->second[9], prop_it->second[10];
                        q1 = true;
                    }
                    if (prop_it->first == "angles2" && prop_it->second.getType() == XmlRpc::XmlRpcValue::TypeArray)
                    {
                        angle2 << prop_it->second[0], prop_it->second[1], prop_it->second[2], prop_it->second[3], prop_it->second[4],
                            prop_it->second[5], prop_it->second[6], prop_it->second[7], prop_it->second[8], prop_it->second[9], prop_it->second[10];
                        q2 = true;
                    }
                }

                // evaluating properties
                if (duration && steps && id)
                {
                    // Check whether a trajectory for the same ID already on the map
                    auto traj_iter = std::find_if(list.begin(), list.end(), [&](const std::tuple<int, robot_misc::HumanTrajectory>& traj) {
                        return trajectory.id == std::get<1>(traj).id;
                    });

                    if (traj_iter == list.end())
                    {
                        generateHumanMotion(angle1, angle2, foot1, foot2, n, d, trajectory, trajectory.returnPath);
                        list.insert(std::pair<int, robot_misc::HumanTrajectory>(trajectory.id, trajectory));
                    }
                    else
                    {
                        ROS_WARN("ObstacleTrajectoryGenerator: Trajectory for ID %d already exists. Trajectory skipped.", trajectory.id);
                    }
                }
                else
                {
                    ROS_WARN("ObstacleTrajectoryGenerator: Not all trajectory properties were set. Trajectory skipped.");
                }
            }
            else
            {
                ROS_WARN("ObstacleTrajectoryGenerator: Invalid trajectory format. Trajectory skipped.");
            }
        }
    }
    else
    {
        ROS_WARN("ObstacleTrajectoryGenerator: No human trajectories found.");
    }
    return list;
}

}  // namespace robot_obstacle
}  // namespace mhp_robot
