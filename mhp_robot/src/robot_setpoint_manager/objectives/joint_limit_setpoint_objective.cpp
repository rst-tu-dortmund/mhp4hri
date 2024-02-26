#include <mhp_robot/robot_setpoint_manager/objectives/joint_limit_setpoint_objective.h>

namespace mhp_robot {
namespace robot_set_point_manager {
namespace objectives {

JointLimitSetpointObjective::JointLimitSetpointObjective(const robot_misc::RobotUtility& robot_utility) { setJointLimits(robot_utility); }

void JointLimitSetpointObjective::setJointLimits(const robot_misc::RobotUtility& robot_utility)
{
    std::vector<double> q_max = robot_utility.getJointMaxLimits();
    std::vector<double> q_min = robot_utility.getJointMinLimits();

    setJointLimits(q_min, q_max);
}

void JointLimitSetpointObjective::setJointLimits(const Eigen::Ref<const Eigen::VectorXd>& q_min, const Eigen::Ref<const Eigen::VectorXd>& q_max)
{
    _q_min = q_min;
    _q_max = q_max;
}

void JointLimitSetpointObjective::setJointLimits(const std::vector<double>& q_min, const std::vector<double>& q_max)
{
    _q_max = Eigen::Map<const Eigen::VectorXd>(q_max.data(), q_max.size());
    _q_min = Eigen::Map<const Eigen::VectorXd>(q_min.data(), q_min.size());
}

double JointLimitSetpointObjective::calculateObjective(const Eigen::Ref<const Eigen::VectorXd>& current,
                                                       const Eigen::Ref<const Eigen::VectorXd>& next) const
{
    return (current - (_q_max + _q_min) / 2).transpose() * (current - (_q_max + _q_min) / 2);
}

}  // namespace objectives
}  // namespace robot_set_point_manager
}  // namespace mhp_robot
