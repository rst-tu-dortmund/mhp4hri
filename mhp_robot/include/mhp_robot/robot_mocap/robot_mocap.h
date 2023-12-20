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

#ifndef ROBOTMOCAP_H
#define ROBOTMOCAP_H

#include <NatNet/NatNetCAPI.h>
#include <NatNet/NatNetClient.h>
#include <NatNet/NatNetTypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace mhp_robot {
namespace robot_mocap {

class RobotMocap
{
 public:
    using Ptr  = std::shared_ptr<RobotMocap>;
    using UPtr = std::unique_ptr<RobotMocap>;

    using RigidBodyMap     = std::unordered_map<int, sRigidBodyDescription>;
    using SkeletonMap      = std::unordered_map<int, sSkeletonDescription>;
    using NatNetClientUPtr = std::unique_ptr<NatNetClient>;

    RobotMocap(const std::string& mocap_frame, bool relative = false, int pred_N = 0, double pred_dt = 0.1);
    virtual ~RobotMocap();

    RobotMocap(const RobotMocap&)            = delete;
    RobotMocap(RobotMocap&&)                 = default;
    RobotMocap& operator=(const RobotMocap&) = delete;
    RobotMocap& operator=(RobotMocap&&)      = default;

    bool connect(const std::string& server_address, const std::string& multicast_address, ushort command_port, ushort data_port);
    void disconnect();
    void publish();
    bool isConnected() const;

 private:
    NatNetClientUPtr _g_pClient;
    sNatNetClientConnectParams _g_connectParams;
    sServerDescription _g_serverDescription;
    RigidBodyMap _rigid_body_info;
    SkeletonMap _skeleton_info;

    tf2_ros::TransformBroadcaster _tf_broadcaster;
    ros::Publisher _pred_pub;

    std::string _mocap_frame = "/mocap";
    double _pred_dt          = 0.01;
    int _pred_N              = 0;
    bool _relative           = false;
    bool _connected          = false;

    void publishTF(const sRigidBodyData& data, const std::string& frame, const std::string& parent_frame);
    void updateDescription(RigidBodyMap& rigid_bodies, SkeletonMap& skeletons) const;

    static std::mutex _frame_mutex;
    static sFrameOfMocapData* _frame;
    static void frameCallback(sFrameOfMocapData* data, void* pUserData);
    static void messageCallback(Verbosity msgType, const char* msg);

 protected:
    virtual bool isRelevant(const std::string& name) const;
};

}  // namespace robot_mocap
}  // namespace mhp_robot

#endif  // ROBOTMOCAP_H
