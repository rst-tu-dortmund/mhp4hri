#include <mhp_robot/robot_setpoint_manager/robot_setpoint_manager.h>

namespace mhp_robot {
namespace robot_set_point_manager {

RobotSetPointManager::RobotSetPointManager(RobotCollision::UPtr robot_collision) : _robot_collision(std::move(robot_collision))
{
    const RobotUtility& robot_utility = _robot_collision->getRobotKinematic()->getRobotUtility();
    _q_min                            = robot_utility.getJointMinLimits();
    _q_max                            = robot_utility.getJointMaxLimits();
}

void RobotSetPointManager::setJointSpaceWaypoints(const std::vector<Eigen::MatrixXd>& waypoints, const std::vector<double>& time)
{
    int n = waypoints.size();
    if (n == 0 || n != time.size()) return;

    _waypoints = waypoints;
    _time      = time;
}

void RobotSetPointManager::validate()
{
    int n = _waypoints.size();
    if (n == 0) return;

    _selector = std::vector<std::vector<bool>>(n, std::vector<bool>(_waypoints[0].rows(), true));
    _dp_table = std::vector<std::vector<std::pair<int, double>>>(
        n, std::vector<std::pair<int, double>>(_waypoints[0].rows(), std::pair<int, double>(-1, std::numeric_limits<double>::max())));

    for (int i = 0; i < n; ++i)
    {
        validateJointState(_waypoints[i], _selector[i]);
    }
}

bool RobotSetPointManager::getOptimalWaypoints(const objectives::BaseSetpointObjective& objective, std::vector<Eigen::VectorXd>& waypoints)
{
    int K = _waypoints.size();

    if (_selector.size() != K || _dp_table.size() != K) validate();

    // Init last waypoint
    for (int i = 0; i < _waypoints[K - 1].rows(); ++i)
    {
        if (_selector[K - 1][i])
        {
            _dp_table[K - 1][i].second = objective.calculateObjective(_waypoints[K - 1].row(i).transpose(), _waypoints[K - 1].row(i).transpose());
        }
    }

    // Iterate backwards from 2nd last waypoint to front
    for (int k = K - 2; k >= 0; --k)
    {
        for (int i = 0; i < _waypoints[k].rows(); ++i)
        {
            if (_selector[k][i])
            {
                double min_cost = std::numeric_limits<double>::max();

                for (int j = 0; j < _waypoints[k + 1].rows(); ++j)
                {
                    if (_selector[k + 1][j] && checkContinuity(_waypoints[k].row(i), _waypoints[k + 1].row(j), _time[k + 1] - _time[k]))
                    {
                        double cost = _dp_table[k + 1][j].second +
                                      objective.calculateObjective(_waypoints[k].row(i).transpose(), _waypoints[k + 1].row(j).transpose());

                        if (cost < min_cost)
                        {
                            min_cost               = cost;
                            _dp_table[k][i].first  = j;
                            _dp_table[k][i].second = cost;
                        }
                    }
                }
            }
        }
    }

    // Find best i
    double min_cost = std::numeric_limits<double>::max();
    int idx         = -1;

    for (int i = 0; i < _waypoints[0].rows(); ++i)
    {
        if ((_dp_table[0][i].first >= 0 || K == 1) && _dp_table[0][i].second < min_cost && _selector[0][i])
        {
            idx      = i;
            min_cost = _dp_table[0][i].second;
        }
    }

    // add waypoints
    waypoints.clear();
    waypoints.reserve(K);

    for (int k = 0; k < K; ++k)
    {
        if (idx < 0) break;

        waypoints.push_back(_waypoints[k].row(idx));
        idx = _dp_table[k][idx].first;
    }

    if (waypoints.size() != K)
    {
        ROS_WARN("RobotSetPointManager: Could not find complete trajectory.");
        return false;
    }
    return true;
}

bool RobotSetPointManager::getOptimalWaypoints(std::vector<Eigen::VectorXd>& waypoints)
{
    objectives::BaseSetpointObjective no_objective;
    return getOptimalWaypoints(no_objective, waypoints);
}

void RobotSetPointManager::setVelocityLimits(double qd_min, double qd_max)
{
    _qd_max = qd_max;
    _qd_min = qd_min;
}

void RobotSetPointManager::setCollisionThresholds(double self_collision, double obstacle_collision, double ground_collision, double roof_collision,
                                                  double plane_collision, double human_collision)
{
    _min_ground_collision   = ground_collision;
    _min_obstacle_collision = obstacle_collision;
    _min_roof_collision     = roof_collision;
    _min_self_collision     = self_collision;
    _min_plane_collision    = plane_collision;
    _min_human_collision    = human_collision;
}

void RobotSetPointManager::considerDynamics(bool obstacle, bool human)
{
    _dynamic_obstacles = obstacle;
    _human_obstacles   = human;
}

void RobotSetPointManager::validateJointState(const Eigen::Ref<const Eigen::MatrixXd>& solutions, std::vector<bool>& selector)
{
    for (int i = 0; i < solutions.rows(); ++i)
    {
        // check joint limits
        for (int j = 0; j < 6; ++j)
        {
            double qi = solutions.row(i)(j);
            if (qi > _q_max[j] || qi < _q_min[j])
            {
                selector[i] = false;
                break;
            }
        }

        // check self collision distances
        _robot_collision->setJointState(solutions.row(i).transpose());

        if (_robot_collision->getMinSelfCollisionDistance() < _min_self_collision)
        {
            selector[i] = false;
            continue;
        }

        _obstacle_manager._mutex.lock();
        std::vector<Obstacle> static_obstacles = _obstacle_manager._static_obstacles;
        std::vector<robot_misc::Plane> planes  = _obstacle_manager._planes;
        _obstacle_manager._mutex.unlock();

        // check for static collisions
        for (const Obstacle& obs : static_obstacles)
        {
            if (_robot_collision->getMinObstacleDistance(obs) < _min_obstacle_collision)
            {
                selector[i] = false;
                break;
            }
        }

        // check for plane collisions
        for (const robot_misc::Plane& plane : planes)
        {
            if (_robot_collision->getMinPlaneCollisionDistance(plane) < _min_plane_collision)
            {
                selector[i] = false;
                break;
            }
        }

        // check for ground collisions
        if (_robot_collision->getMinGroundCollisionDistance() < _min_ground_collision)
        {
            selector[i] = false;
            continue;
        }

        // check for roof collisions
        if (_robot_collision->getMinRoofCollisionDistance() < _min_roof_collision)
        {
            selector[i] = false;
            continue;
        }

        // check for dynamic collisions
        if (_dynamic_obstacles)
        {
            _obstacle_manager._mutex.lock();
            std::vector<Obstacle> dynamic_obstacles = _obstacle_manager._dynamic_obstacles;
            _obstacle_manager._mutex.unlock();

            for (const Obstacle& obs : dynamic_obstacles)
            {
                if (_robot_collision->getMinObstacleDistance(obs) < _min_obstacle_collision)
                {
                    selector[i] = false;
                    break;
                }
            }
        }

        // check for human collisions
        if (_human_obstacles)
        {
            _obstacle_manager._mutex.lock();
            std::vector<robot_misc::Human> humans = _obstacle_manager._humans;
            _obstacle_manager._mutex.unlock();

            for (const robot_misc::Human& human : humans)
            {
                Eigen::VectorXd d(robot_misc::Human::_body_parts_size);
                _robot_collision->getMinHumanDistance(human, d);

                if (d.minCoeff() < _min_human_collision)
                {
                    selector[i] = false;
                    break;
                }
            }
        }
    }
}

bool RobotSetPointManager::checkContinuity(const Eigen::Ref<const Eigen::VectorXd>& current, const Eigen::Ref<const Eigen::VectorXd>& next,
                                           double time) const
{
    Eigen::VectorXd qd = (next - current) / time;
    // return !(qd.minCoeff() < _qd_min || qd.maxCoeff() > _qd_max);

    // TODO(kraemer) there seems to be a problem with angle wrappings of the pose trajectory which causes discontinuities
    return true;
}

}  // namespace robot_set_point_manager
}  // namespace mhp_robot
