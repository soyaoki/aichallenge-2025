#include <planning/lateral_planning.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

std::size_t N = 20;

LateralPlanner::LateralPlanner() = default;
LateralPlanner::~LateralPlanner() = default;

// The upstream planner uses CppAD + IPOPT. Those packages are not available in
// the challenge's reproducible Humble image. Keep the E2E perception/fusion
// pipeline unchanged and use its crosstrack, heading and curvature estimates in
// a compact feedback/feed-forward controller instead.
std::vector<double> LateralPlanner::compute_steering(
    const double L,
    const Eigen::VectorXd& state,
    const Eigen::VectorXd& v_schedule,
    const Eigen::VectorXd& kappa_schedule)
{
    constexpr std::size_t horizon = 19;
    constexpr double max_tire_angle = 0.442;
    constexpr double k_cte = 0.30;
    constexpr double k_heading = 0.85;

    if (state.size() < 3 || v_schedule.size() == 0 || kappa_schedule.size() == 0) {
        return std::vector<double>(horizon, 0.0);
    }

    const double speed = std::max(0.5, std::abs(v_schedule[0]));
    const double feed_forward = std::atan(L * kappa_schedule[0]);
    const double feedback = -k_heading * state[1] - std::atan2(k_cte * state[0], speed);
    const double command = std::clamp(feed_forward + feedback, -max_tire_angle, max_tire_angle);

    return std::vector<double>(horizon, command);
}
