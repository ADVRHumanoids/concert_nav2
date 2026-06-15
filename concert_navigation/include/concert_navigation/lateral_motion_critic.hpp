#ifndef CONCERT_NAVIGATION__LATERAL_MOTION_CRITIC_HPP_
#define CONCERT_NAVIGATION__LATERAL_MOTION_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"

namespace mppi::critics
{

class LateralMotionCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(mppi::CriticData & data) override;

protected:
  unsigned int power_{1};
  float weight_{8.0f};
  float deadband_{0.03f};
};

}  // namespace mppi::critics

#endif  // CONCERT_NAVIGATION__LATERAL_MOTION_CRITIC_HPP_
