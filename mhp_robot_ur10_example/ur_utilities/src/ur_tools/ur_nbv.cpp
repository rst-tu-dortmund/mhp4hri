#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Sensor messages
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>

// UR
#include <mhp_robot/robot_setpoint_manager/objectives/objectives.h>
#include <mhp_robot/robot_setpoint_manager/robot_setpoint_manager.h>
#include <ur_utilities/ur_collision/ur_collision.h>
#include <ur_utilities/ur_kinematic/ur_inverse_kinematic.h>
#include <ur_utilities/ur_kinematic/ur_kinematic.h>
#include <ur_utilities/ur_misc/ur_utility.h>
pcl::PointCloud<pcl::PointXYZI> information_pcl;
bool first_joint_state = true;
std::vector<double> joint_states(6);
Eigen::VectorXd joint_states_eigen(6);
Eigen::Matrix4d tf_cam_to_ee_link = Eigen::Matrix4d::Identity();
Eigen::Matrix4d tf_world_to_fixed = Eigen::Matrix4d::Identity();

void informationGainCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) { pcl::fromROSMsg(*msg, information_pcl); }

void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    if (first_joint_state)
    {
        first_joint_state = false;
        joint_states      = msg->position;
        joint_states_eigen.resize(joint_states.size());
        joint_states_eigen[0] = joint_states[2];
        joint_states_eigen[1] = joint_states[1];
        joint_states_eigen[2] = joint_states[0];
        joint_states_eigen[3] = joint_states[3];
        joint_states_eigen[4] = joint_states[4];
        joint_states_eigen[5] = joint_states[5];
    }
    else
    {
        joint_states = msg->position;
        joint_states_eigen.resize(joint_states.size());
        joint_states_eigen[0] = joint_states[2];
        joint_states_eigen[1] = joint_states[1];
        joint_states_eigen[2] = joint_states[0];
        joint_states_eigen[3] = joint_states[3];
        joint_states_eigen[4] = joint_states[4];
        joint_states_eigen[5] = joint_states[5];
    }
}

int main(int argc, char* argv[])
{
#ifndef NDEBUG
    sleep(5);
#endif
    ros::init(argc, argv, "ur_nbv");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ros::Subscriber information_gain_sub;
    information_gain_sub = nh.subscribe("/ufomap_server_node/info_dist_cloud", 1, informationGainCallback);

    ros::Subscriber joint_state_sub;
    joint_state_sub = nh.subscribe("/ur_driver/joint_states", 1, jointStateCallback);

    ros::Publisher joint_target_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur10_task_planner/joint_targets", 1);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker_point", 10);

    mhp_robot::robot_kinematic::URKinematic ur_kinematic;
    mhp_robot::robot_kinematic::URInverseKinematic::UPtr ur_inverse_kinematic =
        std::make_unique<mhp_robot::robot_kinematic::URInverseKinematic>();

    mhp_robot::robot_set_point_manager::RobotSetPointManager setpoint_manager(
        std::make_unique<mhp_robot::robot_collision::URCollision>());

    mhp_robot::robot_set_point_manager::objectives::BaseSetpointObjective::UPtr redundancy_objective =
        std::make_unique<mhp_robot::robot_set_point_manager::objectives::EuclideanSetpointObjective>(Eigen::VectorXd::Zero(6));
    // Init camera ee link transformation
    tf::StampedTransform transform;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    try
    {
        ros::Time now = ros::Time::now();
        Eigen::Affine3d tmp;
        // tf_listener.waitForTransform("/world", "/fixed_depth_camera", ros::Time(0), ros::Duration(10.0));
        // tf_listener.lookupTransform("/world", "/fixed_depth_camera", ros::Time(0), transform);
        // tf::transformTFToEigen(transform, tmp);
        // tf_world_to_fixed = tmp.matrix();

        tf_listener.waitForTransform("/ee_link", "/camera_3d_depth_camera_link", ros::Time(0), ros::Duration(10.0));
        tf_listener.lookupTransform("/ee_link", "/camera_3d_depth_camera_link", ros::Time(0), transform);
        tf::transformTFToEigen(transform, tmp);
        tf_cam_to_ee_link = tmp.matrix();
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    // Init elements for the IK
    std::vector<Eigen::VectorXd> validated_targets(1);
    int n_solutions;
    std::vector<Eigen::MatrixXd> q(1);
    std::vector<double> times(1);
    validated_targets[0] = Eigen::VectorXd::Zero(6);

    // Set up rate loop
    ros::Rate loop_rate(20);
    bool first_run = true;
    trajectory_msgs::JointTrajectory target;
    visualization_msgs::Marker marker;

    while (ros::ok())
    {
        // Check if the pcl is not empty and if we reached previous target (or have first run)
        if ((validated_targets[0].isApprox(joint_states_eigen, 1e-1) || first_run) && information_pcl.size() > 0)
        {
            // Create a matrix with the pcl points
            Eigen::MatrixXd pcl_points = Eigen::MatrixXd::Zero(4, information_pcl.size());
            for (int i = 0; i < information_pcl.size(); ++i)
            {
                Eigen::Vector4d tmp{information_pcl.points[i].x, information_pcl.points[i].y, information_pcl.points[i].z, 1};
                // Eigen::Vector4d tmp2 = tf_world_to_fixed * tmp;
                pcl_points.col(i)    = Eigen::Vector4d{tmp(0), tmp(1), tmp(2), information_pcl.points[i].intensity};
            }

            // Sort the columns in descending order of elements in the last row but don'T change the elements in one column; only reorder
            Eigen::MatrixXd sorted_pcl_points = pcl_points;
            std::vector<Eigen::VectorXd> vec;
            for (int64_t i = 0; i < pcl_points.cols(); ++i) vec.push_back(pcl_points.col(i));
            std::sort(vec.begin(), vec.end(), [](Eigen::VectorXd a, Eigen::VectorXd b) { return a(3) > b(3); });
            for (int64_t i = 0; i < pcl_points.cols(); ++i) sorted_pcl_points.col(i) = vec[i];

            // Check for valid configuration of the points with the highest intensity
            bool target_found = false;  // If a valid target was found don't look for another one
            for (int i = 0; i < sorted_pcl_points.cols(); ++i)
            {
                times[0] = 0.0;  // Set time to 0 --> Immediate movement
                // POI and current camera z axis in world frame
                Eigen::Vector4d poi_world{1.19, 0.16, 0.33, 1.0};
                                tf::Transform transform;
                    transform.setOrigin(tf::Vector3(poi_world(0, i), poi_world(1, i), poi_world(2, i)));
                    transform.setRotation(tf::Quaternion(0,0,1,1));
                    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "po"));

                // Get the PCL point we desire
                Eigen::Vector4d sorted_pcl_point{sorted_pcl_points(0, i), sorted_pcl_points(1, i), sorted_pcl_points(2, i), 1.0};

                // Get the orientation at sorted_pcl_point towards poi_world
                Eigen::Vector3d orientation_vector = poi_world.head<3>() - sorted_pcl_point.head<3>();
                Eigen::Quaterniond orientation     = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), orientation_vector.normalized()); 
                // X due to rotation between camaera z and ee x

                // Set the task point into Ik element
                ur_inverse_kinematic->setTaskPoint(sorted_pcl_points.block<3, 1>(0, i), orientation);
                q[0] = ur_inverse_kinematic->getSolutions();

                if (ur_inverse_kinematic->isSolutionFeasible() > 0 && !target_found)
                {
                    // Send the transformation to the world frame in tf
                    tf::Transform transform;
                    transform.setOrigin(tf::Vector3(sorted_pcl_points(0, i), sorted_pcl_points(1, i), sorted_pcl_points(2, i)));
                    transform.setRotation(tf::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()));
                    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "goal"));

                    // Set the joint space waypoints
                    setpoint_manager.setJointSpaceWaypoints(q, times);
                    setpoint_manager.validate();
                    validated_targets.clear();

                    // Get the optimal waypoint (joint angles we aim to reach)
                    target_found = setpoint_manager.getOptimalWaypoints(*redundancy_objective, validated_targets);

                    std::cout << "Set Joint configuration " << validated_targets[0] << " at PCL Point number: " << i << std::endl;

                    // Create message to plot NBV start Point
                    target.points.resize(1);
                    target.joint_names = ur_kinematic.getRobotUtility().getJointNames();
                    std::vector<double> mat(validated_targets[0].data(),
                                            validated_targets[0].data() + validated_targets[0].rows() * validated_targets[0].cols());
                                            mat[0] = 0.86;
                                            mat[1] = 0;
                                            mat[2] =0;
                                            mat[3] =-3.1;
                                            mat[4] =-2.36;
                                            mat[5] =-1.36;



                                            // mat[0] = -2.03103;
                                            // mat[1] = -2.93005;
                                            // mat[2] =-0.595799;
                                            // mat[3] =-2.08551;
                                            // mat[4] =0.682748;
                                            // mat[5] =0;
                                            // mat[0] = 0.53537;
                                            // mat[1] = 0.522;
                                            // mat[2] = -1.01492;
                                            // mat[3] = -0.99717;
                                            // mat[4] = -2.42187;
                                            // mat[5] = 1.22;
                    target.points[0].positions       = mat;
                    target.points[0].velocities      = std::vector<double>(q[0].size(), 0.0);
                    target.points[0].time_from_start = ros::Duration(0);

                    // Send target
                    target.header.stamp = ros::Time::now();
                    joint_target_pub.publish(target);
                    first_run = false;

                    // Create marker
                    marker.header.frame_id    = "world";
                    marker.header.stamp       = ros::Time::now();
                    marker.ns                 = "my_namespace";
                    marker.id                 = 0;
                    marker.type               = visualization_msgs::Marker::SPHERE;
                    marker.action             = visualization_msgs::Marker::ADD;
                    marker.pose.position.x    = sorted_pcl_points(0, i);
                    marker.pose.position.y    = sorted_pcl_points(1, i);
                    marker.pose.position.z    = sorted_pcl_points(2, i);
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x            = 0.05;
                    marker.scale.y            = 0.05;
                    marker.scale.z            = 0.05;
                    marker.color.a            = 1.0;  // Don't forget to set the alpha!
                    marker.color.r            = 0.0;
                    marker.color.g            = 1.0;
                    marker.color.b            = 0.0;
                    marker_pub.publish(marker);
                }
            }
        }
        if (!first_run)
        {   
            // std::cout<<"Pub"<<std::endl;
            joint_target_pub.publish(target);
            marker_pub.publish(marker);
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
