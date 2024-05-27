#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H

#include "../bits/types.h"

// stl
#include <optional>

namespace dte3607::physengine::mechanics
{


  inline types::Vector3 computeImpactResponseSphereFixedPlane(
    [[maybe_unused]] types::Vector3 const& sphere_v,
    [[maybe_unused]] types::Vector3 const& fplane_n)
  {
    auto const new_v  = sphere_v-blaze::inner(sphere_v, fplane_n) * fplane_n * 2;
    return new_v;
  }


}   // namespace dte3607::physengine::mechanics


#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_RESPONSE_H
