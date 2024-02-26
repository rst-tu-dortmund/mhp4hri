#include <mhp_robot/robot_setpoint_manager/objectives/length_setpoint_objective.h>

namespace mhp_robot {
namespace robot_set_point_manager {
namespace objectives {

double LengthSetpointObjective::calculateObjective(const Eigen::Ref<const Eigen::VectorXd>& current,
                                                   const Eigen::Ref<const Eigen::VectorXd>& next) const
{
    Eigen::VectorXd diff = current - next;
    return diff.transpose() * diff;
}

}  // namespace objectives
}  // namespace robot_set_point_manager
}  // namespace mhp_robot
