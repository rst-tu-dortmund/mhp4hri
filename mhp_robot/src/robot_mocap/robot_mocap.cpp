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

#include <geometry_msgs/TransformStamped.h>
#include <mhp_robot/robot_mocap/robot_mocap.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <iostream>

namespace mhp_robot {
namespace robot_mocap {

RobotMocap::RobotMocap(const std::string& mocap_frame, bool relative, int pred_N, double pred_dt)
    : _mocap_frame(mocap_frame), _pred_dt(pred_dt), _pred_N(pred_N), _relative(relative)
{
    // Install logging callback
    NatNet_SetLogCallback(messageCallback);

    // Get NatNet Version
    unsigned char ver[4];
    NatNet_GetVersion(ver);

    ROS_INFO("Client NatNet version: %d.%d.%d.%d", ver[0], ver[1], ver[2], ver[3]);

    _g_pClient = std::make_unique<NatNetClient>();

    // Set the frame callback handler
    _g_pClient->SetFrameReceivedCallback(RobotMocap::frameCallback, _g_pClient.get());

    // Set up prediction publisher
    ros::NodeHandle n("~");
    _pred_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("prediction", 10);
}

RobotMocap::~RobotMocap()
{
    disconnect();
    NatNet_FreeFrame(_frame);
}

bool RobotMocap::connect(const std::string& server_address, const std::string& multicast_address, ushort command_port, ushort data_port)
{
    // Release previous server
    _g_pClient->Disconnect();

    // Connection settings
    _g_connectParams.connectionType    = ConnectionType_Multicast;
    _g_connectParams.serverAddress     = server_address.c_str();
    _g_connectParams.serverDataPort    = data_port;
    _g_connectParams.serverCommandPort = command_port;
    _g_connectParams.multicastAddress  = multicast_address.c_str();

    ROS_INFO_STREAM("Attempting to connect to:");
    ROS_INFO_STREAM("Server: " << server_address);
    ROS_INFO_STREAM("Multicast: " << multicast_address);
    ROS_INFO_STREAM("Data port: " << data_port);
    ROS_INFO_STREAM("Command port: " << command_port);

    // Init Client and connect to NatNet server
    int retCode = _g_pClient->Connect(_g_connectParams);

    if (retCode != ErrorCode_OK)
    {
        ROS_ERROR("Unable to connect to server.  Error code: %d. Exiting", retCode);
        return false;
    }
    else
    {
        // print server info
        ErrorCode ret = ErrorCode_OK;
        memset(&_g_serverDescription, 0, sizeof(_g_serverDescription));
        ret = _g_pClient->GetServerDescription(&_g_serverDescription);

        bool success = true;

        if (ret != ErrorCode_OK)
        {
            ROS_ERROR("Unable to connect to server.");
            success = false;
        }

        if (!_g_serverDescription.HostPresent)
        {
            ROS_ERROR("Server not present.");
            success = false;
        }

        if (!_g_serverDescription.ConnectionMulticast)
        {
            ROS_ERROR("Server does not stream in multicast. Please change streaming settings.");
            success = false;
        }

        if (!success)
        {
            ROS_ERROR("Connection failed. Exiting...");
            return false;
        }
        else
        {
            // Print some connection infos

            ROS_INFO("Connected to server: %d.%d.%d.%d", _g_serverDescription.HostComputerAddress[0], _g_serverDescription.HostComputerAddress[1],
                     _g_serverDescription.HostComputerAddress[2], _g_serverDescription.HostComputerAddress[3]);

            ROS_INFO("NatNet server version: %d.%d.%d.%d", _g_serverDescription.NatNetVersion[0], _g_serverDescription.NatNetVersion[1],
                     _g_serverDescription.NatNetVersion[2], _g_serverDescription.NatNetVersion[3]);
        }
    }
    _connected = true;

    return true;
}

void RobotMocap::disconnect()
{
    if (!_connected) return;
    if (_g_pClient) _g_pClient->Disconnect();
}

void RobotMocap::publish()
{
    if (!_connected) return;

    ros::Rate loop(100);
    ros::Time description_update = ros::Time::now();

    while (ros::ok())
    {
        if ((ros::Time::now().toSec() - description_update.toSec()) >= 1.0)
        {
            // maybe we can call it with 100Hz aswell
            updateDescription(_rigid_body_info, _skeleton_info);
            description_update = ros::Time::now();
        }

        sFrameOfMocapData* frame = new sFrameOfMocapData();

        _frame_mutex.lock();
        NatNet_CopyFrame(_frame, frame);
        _frame_mutex.unlock();

        // publish rigid bodies (obstacle, utility objects)
        for (const auto& rigid_body : _rigid_body_info)
        {
            // look for data with matching ID
            for (int i = 0; i < frame->nRigidBodies; ++i)
            {
                if (frame->RigidBodies[i].ID == rigid_body.second.ID)
                {
                    // Publish
                    publishTF(frame->RigidBodies[i], rigid_body.second.szName, _mocap_frame);  // use global frame always
                    break;
                }
            }
        }

        // publish skeleton bodies
        for (const auto& skeleton : _skeleton_info)
        {
            // look for data with matching ID
            for (int i = 0; i < frame->nSkeletons; ++i)
            {
                if (frame->Skeletons[i].skeletonID == skeleton.second.skeletonID)
                {
                    // go through relevant body parts and publish
                    for (int j = 0; j < skeleton.second.nRigidBodies; ++j)
                    {
                        if (isRelevant(skeleton.second.RigidBodies[j].szName))
                        {
                            // look for this body in the frame data
                            for (int k = 0; k < frame->Skeletons[i].nRigidBodies; ++k)
                            {
                                int skelenton_id, rigid_body_id;
                                NatNet_DecodeID(frame->Skeletons[i].RigidBodyData[k].ID, &skelenton_id, &rigid_body_id);

                                if (rigid_body_id == skeleton.second.RigidBodies[j].ID)
                                {
                                    // prepare coordinate frames
                                    std::string frame_id = std::string(skeleton.second.RigidBodies[j].szName) + "_" + std::to_string(skelenton_id);
                                    std::string parent_frame_id = "";

                                    if (_relative)
                                    {
                                        // the root of the skeleton should be mocap frame
                                        if (skeleton.second.RigidBodies[j].parentID == 0)
                                        {
                                            parent_frame_id = _mocap_frame;
                                        }
                                        else
                                        {
                                            // get name of parent by id
                                            for (const auto& rb : skeleton.second.RigidBodies)
                                            {
                                                if (rb.ID == skeleton.second.RigidBodies[j].parentID)
                                                {
                                                    if (!isRelevant(rb.szName))
                                                    {
                                                        // not supported
                                                        ROS_WARN_STREAM_ONCE("Parent frame " << rb.szName << " of rigid body "
                                                                                             << skeleton.second.RigidBodies[j].szName
                                                                                             << " is not relevant and may not be published");
                                                    }
                                                    parent_frame_id = std::string(rb.szName) + "_" + std::to_string(skelenton_id);
                                                }
                                            }
                                        }
                                    }
                                    else
                                    {
                                        // use global frame
                                        parent_frame_id = _mocap_frame;
                                    }

                                    // Publish
                                    publishTF(frame->Skeletons[i].RigidBodyData[k], frame_id, parent_frame_id);
                                    break;
                                }
                            }
                        }
                    }
                    break;
                }
            }
        }

        // publish rigid body predictions
        if (_pred_N > 0)
        {
            ROS_WARN_ONCE("NatNet pose prediction does not work at the moment. Do not use!");

            trajectory_msgs::MultiDOFJointTrajectory msg;
            msg.points.resize(_pred_N);

            // for all rigid bodies
            for (const auto& rigid_body : _rigid_body_info)
            {
                // get predictions
                for (int i = 0; i < _pred_N; ++i)
                {
                    sRigidBodyData pose;
                    ErrorCode ret = ErrorCode_OK;
                    ret           = _g_pClient->GetPredictedRigidBodyPose(rigid_body.second.ID, pose, i * _pred_dt);

                    if (ret == ErrorCode_OK)
                    {
                        geometry_msgs::Transform T;
                        T.translation.x = pose.x;
                        T.translation.y = pose.y;
                        T.translation.z = pose.z;
                        T.rotation.w    = pose.qw;
                        T.rotation.x    = pose.qx;
                        T.rotation.y    = pose.qy;
                        T.rotation.z    = pose.qz;

                        msg.points[i].transforms.push_back(T);
                        msg.points[i].time_from_start = ros::Duration(i * _pred_dt);
                    }
                    else
                    {
                        ROS_ERROR_STREAM("Not able to retrieve prediction for rigid body " << rigid_body.second.szName);
                    }
                }

                msg.joint_names.push_back(rigid_body.second.szName);
            }

            msg.header.frame_id = _mocap_frame;
            msg.header.stamp    = ros::Time::now();

            // publish
            _pred_pub.publish(msg);
        }

        NatNet_FreeFrame(frame);
        delete frame;

        loop.sleep();
    }
}

bool RobotMocap::isConnected() const { return _connected; }

void RobotMocap::publishTF(const sRigidBodyData& data, const std::string& frame, const std::string& parent_frame)
{
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp    = ros::Time::now();
    transformStamped.header.frame_id = parent_frame;
    transformStamped.child_frame_id  = frame;

    transformStamped.transform.translation.x = data.x;
    transformStamped.transform.translation.y = data.y;
    transformStamped.transform.translation.z = data.z;

    transformStamped.transform.rotation.x = data.qx;
    transformStamped.transform.rotation.y = data.qy;
    transformStamped.transform.rotation.z = data.qz;
    transformStamped.transform.rotation.w = data.qw;

    _tf_broadcaster.sendTransform(transformStamped);

    // RB params [...b0]
    // b0: Number of markers below minimum threshold
    // b1
}

void RobotMocap::updateDescription(RigidBodyMap& rigid_bodies, SkeletonMap& skeletons) const
{
    // Retrieve Data from Motive
    sDataDescriptions* pDataDefs = nullptr;
    int result                   = _g_pClient->GetDataDescriptionList(&pDataDefs);

    if (result != ErrorCode_OK || pDataDefs == nullptr)
    {
        ROS_WARN("Unable to retrieve description data.");
    }
    else
    {
        // ROS_INFO("Received %d data descriptions:\n", pDataDefs->nDataDescriptions);

        // Loop though data and copy supported types
        for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
        {
            // Ignore unknown types
            if (pDataDefs->arrDataDescriptions[i].type > 5) continue;

            // ROS_INFO("Data description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);

            if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
            {
                // MarkerSet not supported
                sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                ROS_WARN_ONCE("Recieved marker set %s, which is not supported.", pMS->szName);
            }
            else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;

                if (rigid_bodies.find(pRB->ID) == rigid_bodies.end())
                {
                    ROS_INFO("Recieved new rigid body %s with ID %d", pRB->szName, pRB->ID);
                    rigid_bodies[pRB->ID] = *pRB;

                    // DEBUG
                    // ROS_INFO_STREAM("Infos: Parent ID" << pRB->parentID << std::endl);
                    // ROS_INFO_STREAM("Infos: Offset X" << pRB->offsetx << std::endl);
                    // ROS_INFO_STREAM("Infos: Offset Y" << pRB->offsety << std::endl);
                    // ROS_INFO_STREAM("Infos: Offset Z" << pRB->offsetz << std::endl);
                }
            }
            else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
            {
                // Skeleton
                sSkeletonDescription* pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;

                if (skeletons.find(pSK->skeletonID) == skeletons.end())
                {
                    ROS_INFO("Recieved new skeleton %s with ID %d", pSK->szName, pSK->skeletonID);
                    skeletons[pSK->skeletonID] = *pSK;

                    // DEBUG
                    // for (int k = 0; k < pSK->nRigidBodies; ++k)
                    // {
                    //    ROS_INFO_STREAM("Infos: Parent ID" << pSK->RigidBodies[k].parentID << std::endl);
                    //    ROS_INFO_STREAM("Infos: Offset X" << pSK->RigidBodies[k].offsetx << std::endl);
                    //    ROS_INFO_STREAM("Infos: Offset Y" << pSK->RigidBodies[k].offsety << std::endl);
                    //    ROS_INFO_STREAM("Infos: Offset Z" << pSK->RigidBodies[k].offsetz << std::endl);
                    // }
                }
            }
            else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate)
            {
                // Force Plate not supported
                sForcePlateDescription* pFP = pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
                ROS_WARN_ONCE("Recieved force plate with ID %d, which is not supported.", pFP->ID);
            }
            else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Device)
            {
                // Peripheral Device not supported
                sDeviceDescription* pDevice = pDataDefs->arrDataDescriptions[i].Data.DeviceDescription;
                ROS_WARN_ONCE("Recieved peripheral device %s with ID %d, which is not supported.", pDevice->strName, pDevice->ID);
            }
            else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Camera)
            {
                // Camera not supported
                sCameraDescription* pCamera = pDataDefs->arrDataDescriptions[i].Data.CameraDescription;
                ROS_WARN_ONCE("Recieved camera device %s, which is not supported.", pCamera->strName);
            }
        }
    }

    if (pDataDefs)
    {
        NatNet_FreeDescriptions(pDataDefs);
        pDataDefs = nullptr;
    }
}

bool RobotMocap::isRelevant(const std::string& name) const
{
    // TODO only consider some body parts in case there are too many
    // Names are in the format: <SkeletonName>_LFArm

    return true;
}

void RobotMocap::frameCallback(sFrameOfMocapData* data, void* pUserData)
{
    // Copy data
    _frame_mutex.lock();
    NatNet_FreeFrame(_frame);
    NatNet_CopyFrame(data, _frame);
    _frame_mutex.unlock();
}

void RobotMocap::messageCallback(Verbosity msgType, const char* msg)
{
    // Filter out debug messages
    if (msgType < Verbosity_Info)
    {
        return;
    }

    switch (msgType)
    {
        case Verbosity_Debug:
            ROS_DEBUG("%s", msg);
            break;
        case Verbosity_Info:
            ROS_INFO("%s", msg);
            break;
        case Verbosity_Warning:
            ROS_WARN("%s", msg);
            break;
        case Verbosity_Error:
            ROS_ERROR("%s", msg);
            break;
        default:
            ROS_ERROR("%s", msg);
            break;
    }
}

sFrameOfMocapData* RobotMocap::_frame = new sFrameOfMocapData();
std::mutex RobotMocap::_frame_mutex;

}  // namespace robot_mocap
}  // namespace mhp_robot
