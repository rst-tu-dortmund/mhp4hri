/*********************************************************************
 *
 * Software License Agreement
 *
 *  Copyright (c) 2023,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  This software is currently not released.
 *  Redistribution and use in source and binary forms,
 *  with or without modification, are prohibited.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Heiko Renz
 *********************************************************************/
#ifndef UR_INFORMATION_GAIN_H
#define UR_INFORMATION_GAIN_H

#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mhp_robot/MsgDistances.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <ur_utilities/ur_kinematic/ur_kinematic.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
using URKinematic = mhp_robot::robot_kinematic::URKinematic;
using URUtility   = mhp_robot::robot_misc::URUtility;

class URInformationGain
{
 public:
    using Ptr  = std::shared_ptr<URInformationGain>;
    using UPtr = std::unique_ptr<URInformationGain>;

    URInformationGain(ros::NodeHandle* nh, URKinematic* kinematic, URUtility* utility);

    //    Destructor
    virtual ~URInformationGain() {}

    //    Copy and Move and Constructors
    URInformationGain(const URInformationGain&)            = delete;
    URInformationGain(URInformationGain&&)                 = default;
    URInformationGain& operator=(const URInformationGain&) = delete;
    URInformationGain& operator=(URInformationGain&&)      = default;

    // Callbacks

    // Publish function for ongoing calculation
    void publish();

 protected:
 private:
    // Ros Publisher and Subscriber
    ros::Subscriber _joint_state_sub;
    ros::Subscriber _info_pcl_sub;

    ros::Publisher _info_gain_pub;

    URKinematic _kinematic;
    URUtility _utility;

    std::vector<double> _joint_states;
    pcl::PointCloud<pcl::PointXYZI> _information_pcl;

    Eigen::Matrix4d _tf_cam_to_ee_link = Eigen::Matrix4d::Identity();

    bool _first_joint_state = true;

    void inverseDistanceWeigthing(const Eigen::Ref<const Eigen::Vector3d>& point, double& gain) const;

    // Callbacks
    void informationGainCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
};

#endif  // UR_DANGER_INDEX_H
