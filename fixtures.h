#ifndef DTE3607_PHYSENGINE_FIXTURES_H
#define DTE3607_PHYSENGINE_FIXTURES_H

#include "types.h"

// stl
#include <variant>

namespace dte3607::physengine::fixtures
{


  struct FixtureLevel1 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;


    /*** API concept required methods ***/

    // Global properties
    size_t noRigidBodies() const { return m_spheres.size(); }
    void   setGravity([[maybe_unused]] Forces G) {m_gravity = G;}

    // RB properties
    types::Point3 globalFramePosition([[maybe_unused]] size_t rid) const
    {
      return m_spheres.at(rid).translation;
    }

    /*** Fixture unit-test setup API ***/
    size_t createSphere([[maybe_unused]] ValueType radius      = 1.,
                        [[maybe_unused]] Vector3   velocity    = {0, 0, 0},
                        [[maybe_unused]] Vector3   translation = {0, 0, 0})
    {
      m_spheres.push_back({radius, velocity, translation});
        return m_spheres.size()-1;
    }

    /*** END API requirements ***/


    struct Sphere{
      ValueType radius;
      Vector3 velocity;
      Vector3 translation;
    };
    std::vector<Sphere> m_spheres;
    Forces m_gravity;
  };


  struct FixtureLevel2 {

    /*** API concept required types ***/

             // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;
    using RBState = types::RBState;

             // Fixture types
    using Forces = types::Vector3;


    /*** API concept required methods ***/

             // Environment
    void setGravity(Forces G) {m_gravity = G;}
    // void   setGravity([[maybe_unused]] Forces G) { m_gravity = G; }
    const Forces& getGravity() const {return m_gravity;}

             // RBs
             // size_t              noRigidBodies() const { return m_spheres.size(); } //  + m_fixed_planes.size()
    size_t              noRigidBodies() const { return m_rigid_bodies.size(); } //  + m_fixed_planes.size() // m_spheres
    std::vector<size_t> nonFixedSphereRBs() const { return m_sphere_indexes; }
    std::vector<size_t> fixedInfPlaneRBs() const { return m_plane_indexes; }

    ValueType rbSphereRadius([[maybe_unused]] size_t s_rid) const { return m_rigid_bodies.at(s_rid).radius; }
    Vector3   rbPlaneNormal([[maybe_unused]] size_t p_rid) const { return m_rigid_bodies.at(p_rid).normal; }
    Vector3 sphereVelocity([[maybe_unused]] size_t r_id) const { return m_rigid_bodies.at(r_id).velocity; } // types::Vector3
             // RB properties
    types::Point3 globalFramePosition([[maybe_unused]] size_t rid) const
    {
      // return {};
      return m_rigid_bodies.at(rid).translation; // m_spheres
    }

    //    types::Point3 globalFramePositionPlane([[maybe_unused]] size_t pid) const
    //    {
    //      // return {};
    //      return m_fixed_planes.at(pid).translation;
    //    }

    void setPosition([[maybe_unused]] size_t rid, [[maybe_unused]] Vector3 pos) { m_rigid_bodies.at(rid).translation = pos; } // m_spheres
    void setVelocity([[maybe_unused]] size_t rid, [[maybe_unused]] Vector3 vel) { m_rigid_bodies.at(rid).velocity = vel; } // m_spheres

    /*** Fixture unit-test setup API ***/

    size_t createSphere([[maybe_unused]] ValueType radius      = 1.,
                        [[maybe_unused]] Vector3   velocity    = {0, 0, 0},
                        [[maybe_unused]] Vector3   translation = {0, 0, 0})
    {
      m_rigid_bodies.push_back({radius, velocity, translation, {}});
      m_sphere_indexes.push_back(m_rigid_bodies.size() - 1);
      return m_rigid_bodies.size() - 1;
      //      m_spheres.push_back({radius, velocity, translation});
      //      return m_spheres.size() - 1;
    }

    size_t createFixedInfPlane([[maybe_unused]] Vector3 normal      = {0, 1, 0},
                               [[maybe_unused]] Vector3 translation = {0, 0, 0})
    {
      m_rigid_bodies.push_back({{}, {}, translation, normal});
      const size_t plane_index = m_rigid_bodies.size() - 1;
      m_plane_indexes.push_back(plane_index);
      return plane_index;
      //      m_fixed_planes.push_back({normal, translation});
      //      return m_fixed_planes.size() - 1;
    }

    /*** END API requirements ***/


    //    struct Sphere {
    //      ValueType radius;
    //      Vector3 velocity;
    //      Vector3 translation;
    //    };

    //    struct FixedPlane {
    //      Vector3 normal;
    //      Vector3 translation;
    //    };

    struct Rigidbody {
      ValueType radius;
      Vector3 velocity;
      Vector3 translation;
      Vector3 normal;
    };

    //    std::vector<Sphere> m_spheres;
    //    std::vector<FixedPlane> m_fixed_planes;
    std::vector<Rigidbody> m_rigid_bodies;
    std::vector<size_t> m_sphere_indexes;
    std::vector<size_t> m_plane_indexes;
    Forces m_gravity;
  };



  struct FixtureLevel3 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;

    // RigidBody types
    using RBState = types::RBState;


    /*** API concept required methods ***/
    size_t              noRigidBodies() const { return m_rigid_bodies.size(); } //  + m_fixed_planes.size() // m_spheres
    std::vector<size_t> nonFixedSphereRBs() const { return m_sphere_indexes; }
    std::vector<size_t> fixedInfPlaneRBs() const { return m_plane_indexes; }
    void setPosition([[maybe_unused]] size_t rid, [[maybe_unused]] Vector3 pos) { m_rigid_bodies.at(rid).translation = pos; } // m_spheres
    void setVelocity([[maybe_unused]] size_t rid, [[maybe_unused]] Vector3 vel) { m_rigid_bodies.at(rid).velocity = vel; } // m_spheres
    void setState([[maybe_unused]] size_t rid, [[maybe_unused]] RBState state) { m_rigid_bodies.at(rid).state = state; } // m_spheres
    // Environment
    void setGravity(Forces f ) { m_gravity =f;}
    const Forces& getGravity() const {return m_gravity;}
    ValueType rbMaxFrictionCoef(size_t rid) const { return m_rigid_bodies.at(rid).friction_coef; }
    ValueType rbSphereRadius([[maybe_unused]] size_t s_rid) const { return m_rigid_bodies.at(s_rid).radius; }
    Vector3   rbPlaneNormal([[maybe_unused]] size_t p_rid) const { return m_rigid_bodies.at(p_rid).normal; }
    Vector3 sphereVelocity([[maybe_unused]] size_t r_id) const { return m_rigid_bodies.at(r_id).velocity; } // types::Vector3
    ValueType rbMass(size_t rid) const { return m_rigid_bodies.at(rid).mass; }
    ValueType rbPlaneMaxFrictionCoef() const { return m_max_p_friction; }
    ValueType rbSphereMaxFrictionCoef() const { return m_max_s_friction; }
    types::Point3 globalFramePosition([[maybe_unused]] size_t rid) const
    {
      // return {};
      return m_rigid_bodies.at(rid).translation; // m_spheres
    }

    RBState rbState([[maybe_unused]] size_t rid) const {
      return m_rigid_bodies.at(rid).state; }


    /*** Fixture unit-test setup API ***/

    size_t createSphere([[maybe_unused]] ValueType radius      = 1.,
                        [[maybe_unused]] Vector3   velocity    = {0, 0, 0},
                        [[maybe_unused]] Vector3   translation = {0, 0, 0},
                        [[maybe_unused]] RBState initial_state = RBState::Free,
                        [[maybe_unused]] ValueType friction_coef = 0.,
                        [[maybe_unused]] ValueType mass          = 1.)
    {
      m_rigid_bodies.push_back({radius, velocity, translation, {}, initial_state, friction_coef, mass});
      auto const s_idx = m_rigid_bodies.size()-1;
      m_sphere_indexes.push_back(s_idx);
      return s_idx;
    }

    size_t createFixedInfPlane([[maybe_unused]] Vector3 normal      = {0, 1, 0},
                               [[maybe_unused]] Vector3 translation = {0, 0, 0},
                               [[maybe_unused]] ValueType friction_coef = 0.) 
    {
      m_rigid_bodies.push_back({{},{},translation, normal, {}, friction_coef, {}});
      auto const p_idx = m_rigid_bodies.size()-1;
      m_plane_indexes.push_back(p_idx);
      return p_idx;
    }

    /*** END API requirements ***/
    struct Rigidbody{
      ValueType radius;
      Vector3 velocity;
      Vector3 translation;
      Vector3 normal;
      RBState state;
      ValueType friction_coef;
      ValueType mass;
    };
    std::vector<Rigidbody> m_rigid_bodies;
    std :: vector<size_t> m_sphere_indexes;
    std :: vector<size_t> m_plane_indexes;
    Forces m_gravity;

    ValueType const m_max_p_friction = {0.9};
    ValueType const m_max_s_friction = {0.9};
  };


  struct FixtureLevel4 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;

    // RigidBody types
    using RBState = types::RBState;



    /*** END API requirements ***/
  };


}   // namespace dte3607::physengine::fixtures


#endif   // DTE3607_PHYSENGINE_FIXTURES_H
