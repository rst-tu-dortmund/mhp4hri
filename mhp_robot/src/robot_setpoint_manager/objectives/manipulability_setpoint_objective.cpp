#include <mhp_robot/robot_setpoint_manager/objectives/manipulability_setpoint_objective.h>

namespace mhp_robot {
namespace robot_set_point_manager {
namespace objectives {

ManipulabilitySetpointObjective::ManipulabilitySetpointObjective(RobotKinematic::UPtr robot_kinematic) : _robot_kinematic(std::move(robot_kinematic))
{
}

double ManipulabilitySetpointObjective::calculateObjective(const Eigen::Ref<const Eigen::VectorXd>& current,
                                                           const Eigen::Ref<const Eigen::VectorXd>& next) const
{
    Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian = _robot_kinematic->getGeometricJacobian(current);

    return 1.0 / std::sqrt((jacobian * jacobian.transpose()).determinant());
}

}  // namespace objectives
}  // namespace robot_set_point_manager
}  // namespace mhp_robot
