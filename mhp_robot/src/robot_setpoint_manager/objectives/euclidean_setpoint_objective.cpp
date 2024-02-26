#include <mhp_robot/robot_setpoint_manager/objectives/euclidean_setpoint_objective.h>

namespace mhp_robot {
namespace robot_set_point_manager {
namespace objectives {

EuclideanSetpointObjective::EuclideanSetpointObjective(const Eigen::Ref<const Eigen::VectorXd>& ref) : _ref(ref) {}

void EuclideanSetpointObjective::setReferenceState(const Eigen::Ref<const Eigen::VectorXd>& ref) { _ref = ref; }

double EuclideanSetpointObjective::calculateObjective(const Eigen::Ref<const Eigen::VectorXd>& current,
                                                      const Eigen::Ref<const Eigen::VectorXd>& next) const
{
    Eigen::VectorXd diff = current - _ref;
    return diff.transpose() * diff;
}

}  // namespace objectives
}  // namespace robot_set_point_manager
}  // namespace mhp_robot
