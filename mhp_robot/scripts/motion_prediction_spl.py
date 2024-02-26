#!/usr/bin/env python3

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


import rospy
import sys
import os
import glob
import json
import tensorflow as tf

import warnings

try:
    from spl.data.amass_tf import TFRecordMotionDataset
    from spl.model.zero_velocity import ZeroVelocityBaseline
    from spl.model.rnn import RNN
    from spl.model.seq2seq import Seq2SeqModel
    from common.constants import Constants as C
except:
    raise Exception("SPL Module not found! Please compile with USE_SPL flag or change prediction mode")
    


from rospy.numpy_msg import numpy_msg
from mhp_robot.msg import MsgRelativeOrientation
from std_msgs.msg import Float64

import numpy as np

import time

class PredictionParameters:
    def __init__(self):
        # define normalization matrices
        self.mean_channel = np.array([-0.12032636, -0.00997981,  0.24743576,  0.24991486, -0.00669773,
                                  0.11978491, -0.0036907 ,  0.97110646,  0.04926244,  0.97895525,
                                  0.00616678,  0.01761275, -0.0093735 ,  0.97360618,  0.00153849,
                                 -0.02034631, -0.00439403,  0.96279212,  0.97954638, -0.03615321,
                                 -0.00508451,  0.03640128,  0.96846146, -0.02078537, -0.00209931,
                                  0.01571659,  0.95941864,  0.26351927,  0.63916649, -0.46022949,
                                 -0.71767243,  0.34693531, -0.01958417,  0.13458883,  0.33846018,
                                  0.73661427,  0.29264559, -0.5980269 ,  0.4901239 ,  0.68350721,
                                  0.37481824, -0.02659088, -0.14676624,  0.34487022,  0.71115249,
                                  0.61304517, -0.21951135, -0.53330477,  0.12826328,  0.90696147,
                                 -0.18553698,  0.54973869,  0.00180654,  0.60883196,  0.60887986,
                                  0.20583455,  0.53851432, -0.16187005,  0.91093503, -0.12736878,
                                 -0.536377  , -0.06287609,  0.61406295,  0.88192858, -0.16430348,
                                 -0.17001846,  0.17955798,  0.85661423,  0.0128524 ,  0.17681938,
                                 -0.05359487,  0.9105834 ,  0.89728114,  0.10265626,  0.12602474,
                                 -0.11881239,  0.84861717, -0.00717506, -0.16567865, -0.02545013,
                                  0.89902416])

        self.var_channel = np.array([0.38397698, 0.02520414, 0.51501648, 0.53187299, 0.01811313,
                                0.37316325, 0.00720063, 0.01349053, 0.03382068, 0.00129741,
                                0.01034344, 0.02965753, 0.00988677, 0.00160691, 0.0405071 ,
                                0.02996061, 0.04008332, 0.00255413, 0.0011046 , 0.0101646 ,
                                0.02788679, 0.00973983, 0.0021252 , 0.0484603 , 0.02831501,
                                0.04823854, 0.00271092, 0.09260193, 0.13571813, 0.08189257,
                                0.15996905, 0.1090637 , 0.09516593, 0.14481879, 0.111765  ,
                                0.06814621, 0.08927024, 0.14387026, 0.08336049, 0.18157435,
                                0.11103657, 0.09901123, 0.15479159, 0.12803286, 0.07096193,
                                0.18970708, 0.03421191, 0.06765744, 0.04306022, 0.01023241,
                                0.07325283, 0.07274425, 0.08478809, 0.16957544, 0.19827696,
                                0.02876087, 0.06986198, 0.04280078, 0.01090682, 0.07406505,
                                0.0742854 , 0.08420843, 0.17277922, 0.01722997, 0.11821699,
                                0.03085312, 0.11046102, 0.02472992, 0.09861488, 0.03100483,
                                0.09339712, 0.01229843, 0.01522915, 0.1237847 , 0.02945219,
                                0.10679193, 0.02830974, 0.13057938, 0.0312997 , 0.11656846,
                                0.01579029])


        # Human data for Forward Kinematic
        # Load Human Parameters for defining the offset Matrix
        human = rospy.get_param('/obstacles/humans')[0]
        length = human['length']
        radius = human['radius']

        self.data_mode = rospy.get_param('/robot_workspace_monitor/workspace_monitor_mode','automatic')

        self.joint_parents = np.array([-1,0,1,0,0,3,4,5,6])

        self.offsets = np.array([[0, 0, 0],
                            [0, length[1], 0],
                            [0, radius[2], 0],
                            [radius[1]+radius[3], length[1], 0],
                            [-radius[1]-radius[6], length[1], 0],
                            [length[3], 0, 0],
                            [-length[6], 0, 0],
                            [length[4], 0, 0],
                            [-length[7], 0, 0]])
        self.offsets = self.offsets[np.newaxis,...,np.newaxis]

def forward_kinematic_relative_orientation(relative_orientations,parameters):
    assert relative_orientations.shape[1]==81,"Size of Relative Orientation Matrix incorrect!"
    if relative_orientations.shape[0]==0:
        rospy.logwarn('Attention: Calculation Time to high, no prediction anymore')

    #    print(relative_orientations.shape)
    n_frames = relative_orientations.shape[0]
    n_joints = int(relative_orientations.shape[1]/9)

    relative_orientations = np.reshape(relative_orientations,[relative_orientations.shape[0],n_joints,3,3])

    positions = np.zeros([n_frames, n_joints, 3])
    rotations = np.zeros([n_frames, n_joints, 3, 3])
    poses = np.zeros([n_frames,n_joints,4,4])
    poses[:,:] = np.identity(4)

    for k in range(n_joints):
        if parameters.joint_parents[k]==-1:
            positions[:,k] = 0.0
            rotations[:,k] = relative_orientations[:,k]
            poses[:,k,0:3,3] = positions[:,k]
            poses[:,k,0:3,0:3] = rotations[:,k]

        elif k == 2 and parameters.data_mode=="automatic":
            rotations[:, k] = np.matmul(rotations[:, parameters.joint_parents[k]], relative_orientations[:, k])
            positions[:, k] = np.squeeze(np.matmul(rotations[:, k], parameters.offsets[:, k])) + positions[:, parameters.joint_parents[k]]
            poses[:,k,0:3,3] = positions[:,k]
            poses[:,k,0:3,0:3] = rotations[:,k]
        else:         
            positions[:, k] = np.squeeze(np.matmul(rotations[:, parameters.joint_parents[k]], parameters.offsets[:, k])) + positions[:, parameters.joint_parents[k]]
            rotations[:, k] = np.matmul(rotations[:, parameters.joint_parents[k]], relative_orientations[:, k])
            poses[:,k,0:3,3] = positions[:,k]
            poses[:,k,0:3,0:3] = rotations[:,k]

    return poses

def load_latest_checkpoint(session, saver, experiment_dir):
    """Restore the latest checkpoint found in `experiment_dir`."""
    ckpt = tf.train.get_checkpoint_state(experiment_dir, latest_filename="checkpoint")

    if ckpt and ckpt.model_checkpoint_path:
        # Check if the specific checkpoint exists
        ckpt_name = os.path.basename(ckpt.model_checkpoint_path)
        print("Loading model checkpoint {0}".format(ckpt_name))
        saver.restore(session, ckpt.model_checkpoint_path)
    else:
        raise (ValueError, "Checkpoint {0} does not seem to exist".format(ckpt.model_checkpoint_path))
def get_model_cls(model_type):
    if model_type == C.MODEL_ZERO_VEL:
        return ZeroVelocityBaseline
    elif model_type == C.MODEL_RNN:
        return RNN
    elif model_type == C.MODEL_SEQ2SEQ:
        return Seq2SeqModel
    else:
        raise Exception("Unknown model type.")


def create_and_restore_model(session, experiment_dir, data_dir, config, dynamic_test_split):
    model_cls = get_model_cls(config["model_type"])
    window_length = config["source_seq_len"] + config["target_seq_len"]

    if config["use_h36m"]:
        data_dir = os.path.join(data_dir, '../../h3.6m/tfrecords/')

    if dynamic_test_split:
        data_split = "test_dynamic"
    else:
        assert window_length <= 180, "TFRecords are hardcoded with length of 180."
        data_split = "test"

    # Create model.
    placeholder_input = tf.placeholder("float32",[None,None,81])
    placeholder_target = tf.placeholder("float32",[None,None,81])
    placeholder_seq_len = tf.placeholder("int32",[None])
    placeholder_id = tf.placeholder("string",[None])
    placeholder_dict = {
        "inputs":placeholder_input,
        "targets":placeholder_target,
        "seq_len":placeholder_seq_len,
        "id":placeholder_id
    }
    with tf.name_scope(C.TEST):
        test_model = model_cls(
            config=config,
            data_pl=placeholder_dict,
            mode=C.SAMPLE,
            reuse=False)
        test_model.build_graph()
        test_model.summary_routines()

    num_param = 0
    for v in tf.trainable_variables():
        num_param += np.prod(v.shape.as_list())
    print("# of parameters: " + str(num_param))

    # Restore model parameters.
    saver = tf.train.Saver(tf.global_variables(), max_to_keep=1, save_relative_paths=True)
    load_latest_checkpoint(session, saver, experiment_dir)
    return test_model

def callback(data,args):
    start = time.time()
    target_length=args[0]
    sess = args[1]
    pub = args[2]
    parameters = args[3]
    timePub = args[4]
    seedPub = args[5]
    tf_parameters = args[6]
    allPredPublisher =args[7]
    publishAllPoseData = args[8]

    # Check data size and prepare
    if data.data.shape[0]==9720:
        #        Publish raw data (relative orientations) for test scripts
        seedPub.publish(data.data)
        rotmats = np.reshape(data.data,[120,81])
        #seedPoses = forward_kinematic_relative_orientation(rotmats,parameters)
        # Publish poses (evaluatetd with forward kinematics) for saving data
        #seedPosesReshaped = np.reshape(seedPoses,seedPoses.shape[0]*seedPoses.shape[1]*seedPoses.shape[2]*seedPoses.shape[3])

    else:
        rospy.logerr("Seed Sequence size is not correct!")

    # Prepare data for prediction
    #    rotmats = np.reshape(rotmats,[rotmats.shape[0],rotmats.shape[1]*rotmats.shape[2]*rotmats.shape[3]])

    # Normalization Data from rotmat in /home/renz/Dokumente/amass_ownJustTest


    rotmats = (rotmats-parameters.mean_channel)/ parameters.var_channel

    # Extract seed (2 seconds) for prediction
    seed_sequence = rotmats[:,:];

    seed_sequence = seed_sequence[np.newaxis,...]

    # Better way, directly extract from meta data
    one_step_seq_len = np.ones(seed_sequence.shape[0])

    # Prepare dict which will be feed in to network
    feed_dict = {tf_parameters['prediction_inputs']: seed_sequence,
                 tf_parameters['prediction_seq_len']: np.ones(seed_sequence.shape[0])*seed_sequence.shape[1]}

    # Run the session on the graph with the dictionary as feed in argument
    # and the operations to update the rnn state and the prediction
    state,prediction = sess.run([tf_parameters['rnn_state'],tf_parameters['addOp']], feed_dict=feed_dict)

    # Rearrange Prediction
    prediction = prediction[:, -1:]
    predictions = [prediction]

    # Iterate over all prediction steps and get the new predictions while feeding in
    # the previous one
    for step in range(target_length-1):
        # get the prediction
        feed_dict = {tf_parameters['prediction_inputs']: prediction,
                     tf_parameters['initial_state']: state,
                     tf_parameters['prediction_seq_len']: one_step_seq_len}
        state, prediction = sess.run([tf_parameters['rnn_state'],tf_parameters['addOp']], feed_dict=feed_dict)
        predictions.append(prediction)

    # Get Predictions
    #prediction = _test_model.sample(sess,seed_sequence,target_length)

    # Unnormalize data
    p = predictions* parameters.var_channel + parameters.mean_channel
    p =  p[:,0,0,:]

    if (publishAllPoseData==0):
        #seedPub.publish(seedPosesReshaped)
        allpredictions = np.reshape(p,p.shape[0]*p.shape[1])
        #allPredictedPoses = forward_kinematic_relative_orientation(p[:,:],parameters)
        #allPredOrientation = np.reshape(allPredictedPoses[:,:],[allPredictedPoses.shape[0]*allPredictedPoses.shape[1]*allPredictedPoses.shape[2]*allPredictedPoses.shape[3]])
        allPredPublisher.publish(allpredictions)
        #rospy.signal_shutdown
        publishAllPoseData = publishAllPoseData+1

    p = p[np.newaxis,...]
    # Take time up to now and get prediction frames which are already in the past
    ende = time.time()
    lostFrames = np.ceil(((ende-start))/(1/60))
    #print('Needed Time: {} means {} lost frames from overall {}'.format(ende-start,lostFrames,target_length))
    # Downsample to 10 Hz because more we can't use in the motion planner later
    predictionDownsample = p[0,(int(lostFrames)-1)::6,:]
    #rotmats = rotmats * parameters.var_channel + parameters.mean_channel
    #predictionDownsample = rotmats[118:119,:]

    # Get Poses for all 9 Joints from the predictions
    predictedPoses = forward_kinematic_relative_orientation(predictionDownsample,parameters)

    # Reshape predicted poses and Publish them
    predOrientation=np.reshape(predictedPoses[:,:],[predictedPoses.shape[0]*predictedPoses.shape[1]*predictedPoses.shape[2]*predictedPoses.shape[3]])
    pub.publish(predOrientation)

    # Publish loss of frames
    ende2 = time.time()
    #print(ende2-start)
    timePub.publish((ende2-start))

def relative_orientation_listener():
    # Starting Prediction Node
    predPublisher = rospy.Publisher('/rel_orient_prediction',numpy_msg(MsgRelativeOrientation),queue_size=10)
    allPredPublisher = rospy.Publisher('/all_predictions',numpy_msg(MsgRelativeOrientation),queue_size=10)
    predSeedPublisher = rospy.Publisher('/rel_orient_prediction_seed',numpy_msg(MsgRelativeOrientation),queue_size=10)
    timePublisher = rospy.Publisher('/calculation_time',Float64,queue_size = 10)
    rospy.init_node('motion_prediction_spl',anonymous=True)

    # Getting Target Length parameter
    target_length=rospy.get_param("/human_motion_extrapolation/spl_prediction/target_prediction_length")

    # Define Model And directories where to find model and test data set for building the correct graph
    model_id = rospy.get_param("/human_motion_extrapolation/spl_prediction/model_id")
    _save_dir = rospy.get_param("/human_motion_extrapolation/spl_prediction/save_dir")

    _experiment_dir = glob.glob(os.path.join(_save_dir, str(model_id) + "-*"), recursive=False)[0]
    _data_dir = rospy.get_param("/human_motion_extrapolation/spl_prediction/data_dir")
    _config = json.load(open(os.path.abspath(os.path.join(_experiment_dir, 'config.json')), 'r'))

    # Parameter Class
    parameters = PredictionParameters()

    # Options for GPU Usage
    config = tf.compat.v1.ConfigProto()
    config.gpu_options.allow_growth = True

    tf.compat.v1.reset_default_graph()

    with tf.compat.v1.Session(config=config) as sess:

        # Load Model From the Meta data and restore parameter from last checkpoint
        ckpt = tf.train.get_checkpoint_state(_experiment_dir, latest_filename="checkpoint")

        test_model = tf.compat.v1.train.import_meta_graph(os.path.join(_experiment_dir,os.path.basename(os.path.normpath(ckpt.model_checkpoint_path))+'.meta')) # checkpoint-40000.meta
        test_model.restore(sess,tf.train.latest_checkpoint(os.path.join(_experiment_dir,"./")))


        # Extract graph
        graph = tf.compat.v1.get_default_graph()

        # Get Add operation tensor which has the Prediction as Output
        addOp = graph.get_tensor_by_name("test/add:0")

        # Initialise the rnn_state update operation tensors
        c = graph.get_tensor_by_name("test/rnn_layer/rnn/while/Exit_3:0")
        h = graph.get_tensor_by_name("test/rnn_layer/rnn/while/Exit_4:0")
        rnn_state = tf.compat.v1.nn.rnn_cell.LSTMStateTuple(c,h)

        # Initialize the RNN initial state tensors
        c = graph.get_tensor_by_name("test/LSTMCellZeroState/zeros:0")
        h = graph.get_tensor_by_name("test/LSTMCellZeroState/zeros_1:0")
        initial_states = tf.compat.v1.nn.rnn_cell.LSTMStateTuple(c,h)

        # Get Tensor in which the seed sequence need to be feed in
        prediction_inputs = graph.get_tensor_by_name("test/strided_slice:0")
        # Get Tensor with the sequence length
        prediction_seq_len = graph.get_tensor_by_name("test/mul:0")

        tf_parameters = {'addOp':addOp,
                         'rnn_state':rnn_state,
                         'initial_state':initial_states,
                         'prediction_inputs': prediction_inputs,
                         'prediction_seq_len':prediction_seq_len}

        # Create model
        #_test_model = create_and_restore_model(sess, _experiment_dir, _data_dir, _config,
        #                                                   True)

        # Start Subscriber. Everytime new messages on rel_orient are available the callback is executed
        publishAllPoseData = 0

        rospy.Subscriber("/rel_orient", numpy_msg(MsgRelativeOrientation),callback,(target_length,sess,predPublisher,parameters,timePublisher,predSeedPublisher,tf_parameters,allPredPublisher,publishAllPoseData),queue_size=1,buff_size=2**24)
        print('In Session')

        rospy.spin()


if __name__ == "__main__":
    # calling listener function
    relative_orientation_listener()

