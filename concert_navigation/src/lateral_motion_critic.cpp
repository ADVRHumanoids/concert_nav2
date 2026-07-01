#include "concert_navigation/lateral_motion_critic.hpp"

#include <algorithm>

#include "pluginlib/class_list_macros.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xoperation.hpp"
#include "xtensor/xreducer.hpp"

namespace mppi::critics
{

void LateralMotionCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 8.0f);
  getParam(deadband_, "deadband", 0.03f);
  deadband_ = std::max(0.0f, deadband_);

  RCLCPP_INFO(
    logger_,
    "LateralMotionCritic instantiated with %u power, %.3f weight, %.3f m/s deadband.",
    power_, weight_, deadband_);
}

void LateralMotionCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;

  if (!enabled_ || !data.motion_model->isHolonomic()) {
    return;
  }

  auto lateral_speed = xt::maximum(xt::fabs(data.state.vy) - deadband_, 0.0f);
  auto lateral_cost = xt::sum(lateral_speed * data.model_dt, {1}, immediate) * weight_;

  if (power_ > 1u) {
    data.costs += xt::pow(std::move(lateral_cost), power_);
  } else {
    data.costs += std::move(lateral_cost);
  }
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::LateralMotionCritic, mppi::critics::CriticFunction)