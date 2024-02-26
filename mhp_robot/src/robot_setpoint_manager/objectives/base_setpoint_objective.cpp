#include <mhp_robot/robot_setpoint_manager/objectives/base_setpoint_objective.h>

namespace mhp_robot {
namespace robot_set_point_manager {
namespace objectives {

double BaseSetpointObjective::calculateObjective(const Eigen::Ref<const Eigen::VectorXd>& current,
                                                 const Eigen::Ref<const Eigen::VectorXd>& next) const
{
    return 0.0;
}

}  // namespace objectives
}  // namespace robot_set_point_manager
}  // namespace mhp_robot
