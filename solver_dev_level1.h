#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL1_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL1_H

#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../mechanics/compute_trajectory.h"
#include <algorithm>


namespace dte3607::physengine::solver_dev::level1
{

  using namespace dte3607::physengine::types;
    struct Params{
      Vector3          F;
      Duration                dt;
    };

    struct CacheProcDataBlock{
      Vector3          in_p;//Position
      Vector3          in_v;//Velocity
      Vector3          out_ds;//Trajectory

    };

    using CacheProcData = std::vector<CacheProcDataBlock>;

    struct SimProcDataBlock{
      Vector3          in_p; //Position
      Vector3          in_ds;//Trajectory
      Vector3          out_p;//New Position

    };
  using SimProcData = std::vector<SimProcDataBlock>;

  void computeCache(CacheProcData& data, Params const& params ){
    auto const proc_kernel =[&params](auto& data_block ){
      auto const& [F,dt]=params;
      auto& [pos, vel, out_ds]= data_block;
      auto [ds,a]=mechanics::computeLinearTrajectory(vel, F, dt);
      out_ds = ds;
    };
    std::ranges::for_each(data,proc_kernel);
  }

  void computeSimulation(SimProcData& sim_data){
    auto const sim_kernel=[](auto& sim_data_block){
      auto& [p, ds, out_p]=sim_data_block;
      out_p=p+ds;
    };
    std::ranges::for_each(sim_data, sim_kernel);
  }
  template <concepts::SolverFixtureLevel1 Fixture_T>
  void solve([[maybe_unused]] Fixture_T&         scenario,
             [[maybe_unused]] types::NanoSeconds timestep)
    {
    Params param_scenario = {scenario.m_gravity, timestep};
    CacheProcData cache_data;
    SimProcData sim_data;

    auto no_rbs = scenario.noRigidBodies();
    //Compute data cache for all RBs
    for(auto rid =0; rid<no_rbs; ++rid){
      auto& [radius, velocity, translation] = scenario.m_spheres[rid];
      cache_data.push_back({translation, velocity, {}});
     };

     computeCache(cache_data, param_scenario);
    //Perfom simulation of all RBs
    for(auto rid =0; rid<no_rbs; ++rid){
      auto& [c_p, c_v, c_ds] = cache_data[rid];
      sim_data.push_back({c_p, c_ds,{}});
    };

      computeSimulation(sim_data);
    //Write results back to scenario
    for(auto rid =0; rid<no_rbs; ++rid){
        scenario.m_spheres[rid].translation = sim_data[rid].out_p;
      };
  };



}   // namespace dte3607::physengine::solver_dev::level1


#endif // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL1_H
