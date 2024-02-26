#include <mhp_robot/MsgDistances.h>
#include <mhp_robot/robot_kinematic/robot_kinematic.h>
#include <ros/ros.h>
#include <ur_utilities/ur_misc/ur_information_gain.h>
#include <ur_utilities/ur_kinematic/ur_kinematic.h>
#include <ur_utilities/ur_misc/ur_utility.h>
#include <Eigen/Eigen>

using URKinematic = mhp_robot::robot_kinematic::URKinematic;
using URUtility   = mhp_robot::robot_misc::URUtility;

int main(int argc, char** argv)
{
#ifndef NDEBUG
    sleep(5);
#endif
    ros::init(argc, argv, "ur_danger_index");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("UR Danger Index Node Started...");

    URKinematic kinematic = URKinematic();
    URUtility   utility   = URUtility();
    URInformationGain gain = URInformationGain(&nh, &kinematic,&utility);

    gain.publish();

    ros::waitForShutdown();
    return 0;
}
