#ifndef DTE3607_PHYSENGINE_MECHANICS_COMPUTE_TRAJECTORY_H
#define DTE3607_PHYSENGINE_MECHANICS_COMPUTE_TRAJECTORY_H


#include "../bits/types.h"

#include "../utils/type_conversion.h"

#include <cmath>

namespace dte3607::physengine::mechanics
{

  inline std::pair<types::Vector3, types::Vector3> computeLinearTrajectory(
    [[maybe_unused]] types::Vector3 const& velocity,
    [[maybe_unused]] types::Vector3 const& external_forces,
    [[maybe_unused]] types::Duration       timestep)
  {
    auto const dt = utils::toDtScalar(timestep);
    auto const ds = velocity * dt + 0.5 * external_forces * std::pow(dt, 2);
    auto const a =external_forces * dt;
    return {ds,a};
  }


}   // namespace dte3607::physengine::mechanics

#endif // DTE3607_PHYSENGINE_MECHANICS_COMPUTE_TRAJECTORY_H
