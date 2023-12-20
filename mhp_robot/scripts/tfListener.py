#!/usr/bin/env python2.7

# /*********************************************************************
# *
# *  Software License Agreement
# *
# *  Copyright (c) 2023,
# *  TU Dortmund University, Institute of Control Theory and System Engineering
# *  All rights reserved.
# *
# *  This program is free software: you can redistribute it and/or modify
# *  it under the terms of the GNU General Public License as published by
# *  the Free Software Foundation, either version 3 of the License, or
# *  (at your option) any later version.
# *
# *  This program is distributed in the hope that it will be useful,
# *  but WITHOUT ANY WARRANTY; without even the implied warranty of
# *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# *  GNU General Public License for more details.
# *
# *  You should have received a copy of the GNU General Public License
# *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
# *
# *  Authors: Heiko Renz
# *********************************************************************/


import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import numpy as np
from mhp_robot.msg import MsgRelativeOrientation
import std_msgs.msg
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String

if __name__ == '__main__':

    relativeOrientationPublisher = rospy.Publisher('/rel_orient',numpy_msg(MsgRelativeOrientation),queue_size=10) # Publishing Orientation data

    relativeOrientationWithTargetPublisher = rospy.Publisher('/rel_orient_with_target',numpy_msg(MsgRelativeOrientation),queue_size=10) #Include target data for accuracy comparison

    rospy.init_node('python_tf_listener') #Start Node
    target_length = rospy.get_param('/human_motion_extrapolation/spl_prediction/target_prediction_length') # Prediction length --> in frames (60 frames per minute)

    listener = tf.TransformListener() # Get tf data
    human = rospy.get_param('/obstacles/humans')
    human_name = (human[0]['name']);

    frameNames = []
    frameNames.append('/'+human_name+'Hip_0')
    frameNames.append('/'+human_name+'Neck_0')
    frameNames.append('/'+human_name+'Head_0')
    frameNames.append('/'+human_name+'LUArm_0')
    frameNames.append('/'+human_name+'RUArm_0')
    frameNames.append('/'+human_name+'LFArm_0')
    frameNames.append('/'+human_name+'RFArm_0')
    frameNames.append('/'+human_name+'LHand_0')
    frameNames.append('/'+human_name+'RHand_0')

    pose = [0]*len(frameNames)

    # Predefine required matrices and counters
    relOrient = np.zeros((120,9,3,3))
    relOrientTarget = np.zeros((120 + target_length,9,3,3))
    counter = 0
    counterTarget = 0

    # Use Fixed rate --> same as the input data for the SPL network
    rate = rospy.Rate(60.0)

    while not rospy.is_shutdown():
        try:
            for x in range(len(frameNames)):
                # get transformation for every frame and transform to a homogenous transformation matrix(pose)
                (trans,rot) = listener.lookupTransform('/world', frameNames[x], rospy.Time(0))
                pose[x] = listener.fromTranslationRotation(trans,rot)

            # define relative orientations between frame poses
            relOr = np.zeros((9,3,3))
            relOr[0,:,:] = pose[0][:3,:3]
            relOr[1,:,:] = np.matmul(np.transpose(pose[0][:3,:3]),pose[1][:3,:3])
            relOr[2,:,:] = np.matmul(np.transpose(pose[1][:3,:3]),pose[2][:3,:3])
            relOr[3,:,:] = np.matmul(np.transpose(pose[0][:3,:3]),pose[3][:3,:3])
            relOr[4,:,:] = np.matmul(np.transpose(pose[0][:3,:3]),pose[4][:3,:3])
            relOr[5,:,:] = np.matmul(np.transpose(pose[3][:3,:3]),pose[5][:3,:3])
            relOr[6,:,:] = np.matmul(np.transpose(pose[4][:3,:3]),pose[6][:3,:3])
            relOr[7,:,:] = np.matmul(np.transpose(pose[5][:3,:3]),pose[7][:3,:3])
            relOr[8,:,:] = np.matmul(np.transpose(pose[6][:3,:3]),pose[8][:3,:3])

            if counter<119: # if not 2 seconds of data are available
                relOrient[counter,:,:,:] = relOr
                counter = counter + 1
            elif counter == 119: # exactly 2 seconds of data
                relOrient[counter,:,:,:] = relOr
                counter = counter + 1
                orientationMsg = np.reshape(relOrient,[relOrient.shape[0]*relOrient.shape[1]*relOrient.shape[2]*relOrient.shape[3]])
                relativeOrientationPublisher.publish(orientationMsg)
            elif counter == 120: # over 2 seconds of input data
                relOrient = np.roll(relOrient,-1,axis=0) # discard oldest data
                relOrient[counter-1,:,:,:] = relOr    #fill up with new data
                orientationMsg = np.reshape(relOrient,[relOrient.shape[0]*relOrient.shape[1]*relOrient.shape[2]*relOrient.shape[3]])
                relativeOrientationPublisher.publish(orientationMsg)

            # prepare data the same but add the target orientations for the network
            if counterTarget<119+target_length:
                relOrientTarget[counterTarget,:,:,:] = relOr
                counterTarget = counterTarget + 1
            elif counterTarget == 119+target_length:
                relOrientTarget[counterTarget,:,:,:] = relOr
                counterTarget = counterTarget + 1
                orientationMsg = np.reshape(relOrientTarget,[relOrientTarget.shape[0]*relOrientTarget.shape[1]*relOrientTarget.shape[2]*relOrientTarget.shape[3]])
                relativeOrientationWithTargetPublisher.publish(orientationMsg)
            elif counterTarget == 120+target_length:
                relOrientTarget = np.roll(relOrientTarget,-1,axis=0)
                relOrientTarget[counterTarget-1,:,:,:] = relOr
                orientationMsg = np.reshape(relOrientTarget,[relOrientTarget.shape[0]*relOrientTarget.shape[1]*relOrientTarget.shape[2]*relOrientTarget.shape[3]])
                relativeOrientationWithTargetPublisher.publish(orientationMsg)



        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        rate.sleep()
