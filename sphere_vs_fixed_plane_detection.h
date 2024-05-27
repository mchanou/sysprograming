#ifndef DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
#define DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H

#include "../bits/types.h"
#include <optional>

// stl
#include "../utils/type_conversion.h"
#include "compute_trajectory.h"


namespace dte3607::physengine::mechanics
{
  inline std::optional<types::HighResolutionTP> detectCollisionSphereFixedPlane(
    [[maybe_unused]] types::HighResolutionTP const& sphere_tc,
    [[maybe_unused]] types::Point3 const&           sphere_p,
    [[maybe_unused]] types::ValueType               sphere_r,
    [[maybe_unused]] types::Vector3 const&          ds,
    [[maybe_unused]] types::Point3 const&           fplane_q,
    [[maybe_unused]] types::Vector3 const&          fplane_n,
    [[maybe_unused]] types::HighResolutionTP const& t_0,
    [[maybe_unused]] types::Duration                timestep)
  {
    auto const epsilon = 10e-5;

    auto const d = (fplane_q + sphere_r * fplane_n) - sphere_p;

    auto const Q = blaze::inner(d, fplane_n);
    auto const R = blaze::inner(ds,fplane_n);

             //Check Singularities first
    if (std::abs(R)<epsilon)
      return std::nullopt;//Sphere moves in parrallel to the plane


    if (std::abs(Q)<epsilon)
      return std::nullopt;//Sphere touches the plane

             //Condition for collsion xR=Q;
    auto const x = Q/R;

    if (x <= 0 or x>1)
      return std::nullopt;

             //find collision position in time (this does takes sphere_tc into account)
    auto const tp = sphere_tc + utils::toDuration(x * timestep - (sphere_tc - t_0));
    return tp;

  }

  inline std::optional<types::HighResolutionTP> detectCollisionSphereFixedPlane(
    [[maybe_unused]] types::HighResolutionTP const& sphere_tc,
    [[maybe_unused]] types::Point3 const&           sphere_p,
    [[maybe_unused]] types::ValueType               sphere_r,
    [[maybe_unused]] types::Vector3 const&          sphere_v,
    [[maybe_unused]] types::Point3 const&           fplane_q,
    [[maybe_unused]] types::Vector3 const&          fplane_n,
    [[maybe_unused]] types::Vector3 const&          external_forces,
    [[maybe_unused]] types::HighResolutionTP const& t_0,
    [[maybe_unused]] types::Duration                timestep)


  {

      //Compute ds by the compute trajectory
      auto const [ds,a] = computeLinearTrajectory(sphere_v, external_forces, timestep-(sphere_tc-t_0));


      return detectCollisionSphereFixedPlane(sphere_tc, sphere_p, sphere_r, ds, fplane_q, fplane_n, t_0, timestep);

  }

}   // namespace dte3607::physengine::mechanics



#endif   // DTE3607_PHYSENGINE_MECHANICS_SPHERE_VS_FIXED_PLANE_DETECTION_H
