#include <ur_utilities/ur_misc/ur_information_gain.h>

URInformationGain::URInformationGain(ros::NodeHandle* nh, URKinematic* kinematic, URUtility* utility)
{

    // Subscribers for distances, collision Points and current joint velocities
    _joint_state_sub = nh->subscribe("/ur_driver/joint_states", 1, &URInformationGain::jointStateCallback, this);
    _info_pcl_sub    = nh->subscribe("/ufomap_server_node/info_dist_cloud", 1, &URInformationGain::informationGainCallback, this);

    // Publisher for the gain
    _info_gain_pub = nh->advertise<std_msgs::Float64>("/ur10/info_gain", 1);
    // Kinematic structure as class variable to get partial Jacobians
    _kinematic    = std::move(*kinematic);
    _utility      = std::move(*utility);
    _joint_states = std::vector<double>(_utility.getJointsCount(), 0.0);

    tf::StampedTransform transform;
    tf::TransformListener tf_listener;
    try
    {
        ros::Time now = ros::Time::now();
        Eigen::Affine3d tmp;
        tf_listener.waitForTransform("/ee_link", "/camera_3d_depth_camera_link", ros::Time(0), ros::Duration(10.0));
        tf_listener.lookupTransform("/ee_link", "/camera_3d_depth_camera_link", ros::Time(0), transform);
        tf::transformTFToEigen(transform, tmp);
        _tf_cam_to_ee_link = tmp.matrix();
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void URInformationGain::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (_first_joint_state)
    {
        if (!_utility.initJointMapping(msg->name))
        {
            return;
        }
        _first_joint_state = false;
        _utility.parseJointStates(_joint_states, msg->position);
    }
    else
    {
        _utility.parseJointStates(_joint_states, msg->position);
    }
}
void URInformationGain::informationGainCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) { pcl::fromROSMsg(*msg, _information_pcl); }

void URInformationGain::publish()
{
    // Fixed Frequency for calculating and publishing danger index
    ros::Rate loop(30);

    // Loop to publish danger index
    while (ros::ok())
    {
        double factor = 1;
        double gain   = 0;
        // std::cout << "Information pcl size: " << _information_pcl.size() << std::endl;
        // std::cout << "Joint states size: " << _joint_states.size() << std::endl;
        if (_information_pcl.size() > 0 && _joint_states.size() > 0)  // be sure we already received a point cloud
        {
            // get the transformation from depth camera to world for joint configuration x_k
            Eigen::VectorXd x_k = Eigen::Map<Eigen::VectorXd>(_joint_states.data(), _joint_states.size());
            // std::cout << _joint_states.size() << std::endl;
            Eigen::Matrix4d T = _kinematic.getEndEffectorMatrix(x_k) * _tf_cam_to_ee_link;

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
            // std::cout<<T.block<3, 1>(0, 3)<<std::endl;
            inverseDistanceWeigthing(T.block<3, 1>(0, 3), gain);
        }
        // std::cout << "Factor: " << factor << std::endl;
        // std::cout << "Gain: " << gain << std::endl;
        std_msgs::Float64 msg;
        msg.data = factor * gain;
        _info_gain_pub.publish(msg);  // add small epsilon to avoid division by zero

        ros::spinOnce();  // execute callbacks
        loop.sleep();
    }
}

void URInformationGain::inverseDistanceWeigthing(const Eigen::Ref<const Eigen::Vector3d>& point, double& gain) const
{
    // get the distance to the point cloud points (with Eigen Matrix for vectorized operations)
    Eigen::MatrixXd pcl_points = Eigen::MatrixXd::Zero(3, _information_pcl.size());
    for (int i = 0; i < _information_pcl.size(); ++i)
    {
        pcl_points.col(i) = Eigen::Vector3d{_information_pcl.points[i].x, _information_pcl.points[i].y, _information_pcl.points[i].z};
    }
    // Eigen::MatrixXd pcl_points = _information_pcl.getMatrixXfMap(3, 4, 0).cast<double>();
    // std::cout<< "PCL Points: " << pcl_points << std::endl;
    Eigen::VectorXd dists = Eigen::VectorXd::Zero(_information_pcl.size());
    dists = (pcl_points.colwise() - point).colwise().norm();
    // std::cout << "Dists: " << dists << std::endl;
    // get the inverse distance weighting
    int p         = 2;  // power of the inverse distance weighting
    dists.array() = dists.array().pow(-p);
    // std::cout<< "Dists after pow: " << dists << std::endl;
    gain          = 0;
    for (int i = 0; i < _information_pcl.size(); ++i)
    {
        gain += (dists(i) / dists.sum()) * _information_pcl.points[i].intensity;
    }
}
