#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL3_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL3_H

#include "../bits/types.h"
#include "../bits/concepts.h"
//#include "../../benchmarking/benchmarks/my_dummy_bechmarks.cpp"
#include <algorithm>
#include "../mechanics/compute_trajectory.h"
#include "../mechanics/sphere_vs_fixed_plane_detection.h"
#include "../mechanics/sphere_vs_fixed_plane_response.h"
#include <set>

namespace dte3607::physengine::solver_dev::level3
{
  using namespace dte3607::physengine::types;
  struct Params{
    Vector3          F;
    Duration                dt;
  };

  class Timer{
  public:
    Timer(){
      m_StartTimepoint = std::chrono::high_resolution_clock::now();
    }
    ~Timer(){
      Stop();
    }
    void Stop(){
      auto endTimepoint = std::chrono::high_resolution_clock::now();

      auto start = std::chrono::time_point_cast<std::chrono::microseconds>(m_StartTimepoint).time_since_epoch().count();
      auto end = std::chrono::time_point_cast<std::chrono::microseconds>(endTimepoint).time_since_epoch().count();

      auto duration = end-start;
      double ms = duration * 0.001;

      std::cout << "The time is " <<duration <<"us (" <<ms << "ms)" << std::endl;
    }
  private:
    std::chrono::time_point<std::chrono::high_resolution_clock > m_StartTimepoint;
  };

  using TP=HighResolutionTP;
  struct CacheProcDataBlock{
    size_t rb_id;
    Point3 in_p;
    Vector3 in_v;
    TP in_tp;
    Vector3 out_ds;
    Vector3 out_a;
  };

  struct SphereGeomDataBlock {
    Point3    p = {};
    ValueType r = {};
    Vector3   ds = {};
    Vector3 velocity = {};
    TP        tp = {};
    size_t    cache_index = {};
    ValueType sFriction;
    ValueType mass;
    bool operator<(SphereGeomDataBlock const& rhs) const {
      return tp < rhs.tp;
    }
  };

  struct SimProcDataBlock{
    Point3 in_p;
    Vector3 in_ds;
    Point3 out_p;
  };

  struct InfPlaneGeomDataBlock{
    Point3 p;
    Vector3 n;
    ValueType pFriction;
    size_t rb_id; //Normal in global coordinate space
  };

  struct IntsecStatusDataBlock{
    bool is_collision; //Whether there was a colision or not
    TP col_tp;
  };
  struct StateDataBlock{
    RBState state;
    bool attached;
    ValueType friction_coefT;
  };

  struct IntersectDetProcDataBlock{
    SphereGeomDataBlock in_sphere ; // Sphere data
    InfPlaneGeomDataBlock in_plane; //Inf plane data
    IntsecStatusDataBlock out_status; // Intersection status data
    StateDataBlock out_state; // Intersection status data

  };

  // struct StateDetProcDataBlock{
  //   SphereGeomDataBlock in_sphere ; // Sphere data
  //   InfPlaneGeomDataBlock in_plane; //Inf plane data
  //   StateDataBlock out_state; // Intersection status data
  // };
  //kusing StateDetProcData = std::vector<StateDetProcDataBlock>;
  using CacheProcData = std::vector<CacheProcDataBlock>;
  using SimProcData = std::vector<SimProcDataBlock>;
  using IntersectDetProcData = std::vector<IntersectDetProcDataBlock>;




  auto const emulateLoss(ValueType fc1, ValueType fc2){
    ValueType fcT;
    fcT=fc1*fc2;
        //auto const fd = std::min(1,fcT);
    auto const fd = 0.15;
    return fd;
  }

  void energyLoss(auto& vel, ValueType fd){
    //ValueType fd = emulateLoss(fc1, fc2);
    vel = vel*(1-fd);
  }


  // auto const attached_kernel=[](auto& state_data_block ){
  //   auto& [s, p, out_state ] =state_data_block;
  //   if(out_state.state == types::RBState::Free){
  //     out_state.attached = false;
  //   }else{
  //     out_state.attached = true;
  //   }
  // };

  bool attached(types::RBState state){
    if(state == types::RBState::Free){
      return false;
    }else{
      return true;
    }
  }
  auto const slide_kernel=[](auto& state_data_block){ //After the benchmarking split that in free kernel and slide+kernel to avoid using to much if
    auto& [s, p, out_status, out_state ] =state_data_block;
    auto T= blaze::inner(s.ds,p.n);
    auto sp1 =s.p +s.ds;
    //Vector3  = sp1+p.p;
    double proj = blaze::dot(s.velocity, p.n);
    auto q = p.p+ proj*p.n;
    //if (T>0 && attached == false){
    if (T>0){
      out_state.state=types::RBState::Free;
    }else if (T<=0){
      out_state.state=types::RBState::Sliding;
      auto const d = (q-sp1)+p.n*s.r;
      s.ds= s.ds+d;
    }
  };

  auto const rest_kernel=[](auto& state_data_block){
    auto& [s, p, out_status, out_state ] =state_data_block;
    if (s.ds == 0 /*&& attached == true*/) {
      out_state.state = types::RBState::Resting;
    }
  };

  //void updateState(SimProcData& sim_data,  )
  void checkState(IntersectDetProcData& data){

     // auto const free_kernel=[](auto& state_data_block){
     //   auto& [sp, r, ds,v,frt1 ,tp , pp, n,frt2 ,rb_id , state, col_tp ] =state_data_block;
     //   auto const T= blaze::inner(ds,n);
     //   if (state =="Resting" || state =="Slidding" && T>0){
     //     state = "Free";
     //   }
     // };
     //Remove attached in the conditions ? check attachment??
      //std::ranges::for_each(data, attached_kernel);
      std::ranges::for_each(data, slide_kernel);
      std::ranges::for_each(data, rest_kernel);
  }

  void computeCache(CacheProcData& data, Params const& params) {

    auto const cache_proc_kernel = [&params](auto& data){
      auto const &[F, dt] = params;
      auto& [rb_id, pos, vel, tp, out_ds, out_a] = data;
      auto traj_accel = mechanics::computeLinearTrajectory(vel, F, dt);
      out_ds = traj_accel.first;
      out_a = traj_accel.second;
    };

    std::ranges::for_each(data, cache_proc_kernel);
  }

  auto const sim_kernel = [](auto& data_block) {
    auto& [p, ds, out_p] = data_block;
    out_p = p + ds;
  };

  void computeSimulation(SimProcData& data) {
    std::ranges::for_each(data, sim_kernel);
  }

  void computeIntersection(IntersectDetProcData& data /*, Vector3 F*/, TP t_0, Duration dt) {
      // Kernel for detection, as a lambda function
      auto const det_kernel = [&](auto &int_data_block) {
        auto& [s, p, out_status, out_state] = int_data_block;
        if (auto col_tp = mechanics::detectCollisionSphereFixedPlane(s.tp,
                                                                     s.p,
                                                                     s.r,
                                                                     s.ds,
                                                                     p.p,
                                                                     p.n, // F
                                                                     t_0,
                                                                     dt)) {
          out_status.is_collision = true;
          out_status.col_tp = col_tp.value();
        }
        else {
          out_status.is_collision = false;
        }
      };
      std::ranges::for_each(data, det_kernel);
    }
  template <typename IntersectContainer_T>
  void makeSortAndUnique(IntersectContainer_T& intersections) {
      // sort
      std::sort(std::begin(intersections), std::end(intersections),
                [](auto const& intblock_a, auto const& intblock_b) {
                  return intblock_a.out_status.col_tp < intblock_b.out_status.col_tp;
                });

               // make unique
      using IntUniqueMemSet = std::set<size_t>;
      IntUniqueMemSet int_unique_set;
      std::erase_if(intersections, [&int_unique_set](auto const &obj) {
        if (not obj.out_status.is_collision)
          return true;
        SphereGeomDataBlock const& s = obj.in_sphere;
        if (int_unique_set.contains(s.cache_index))
          return true;
        int_unique_set.insert(s.cache_index);
        return false;
      });

               // (optional test first)
               // reverse (smallest first)
      std::reverse(std::begin(intersections), std::end(intersections));
    }

  // void setState(StateDetProcData& data ){
  //   auto& const set_state = [&](auto &int_data_block){
  //     auto& [s,p, out_state] = int_data_block;
  //     out_state.state = scenario.rbState();

  //   };
  // }

  template <concepts::SolverFixtureLevel3 Fixture_T>
  void solve([[maybe_unused]] Fixture_T&         scenario,
             [[maybe_unused]] types::NanoSeconds timestep)
  {
    Timer timer;
    auto O = scenario.nonFixedSphereRBs();
    auto F = scenario.fixedInfPlaneRBs();

    CacheProcData cache_data;

    auto const t_0 = types::HighResolutionClock::now();

    // Generate process data blocks
    for (auto o:O) {
      auto const& pos = scenario.globalFramePosition(o);
      auto const& vel = scenario.sphereVelocity(o);
      CacheProcDataBlock db{o, pos, vel, t_0, {}, {}};
      cache_data.push_back(db);
    }

    // Enviroment params
    Params env_params{scenario.getGravity(), timestep};

    // Compute cache properties
    computeCache(cache_data, env_params);

    // Procedure step
    // detect collision
    // Set up data blocks for collision detection
    //    Loop through all spheres and planes and create int data blocks

    IntersectDetProcData int_data;
    //StateDetProcData state_data;
    std::vector<StateDataBlock> st_blocks;
    std::vector<SphereGeomDataBlock> s_g_blocks;
    std::vector<InfPlaneGeomDataBlock> p_g_blocks;

    size_t i = 0;
    for (auto o = O.begin(); o != O.end(); ++o, ++i) { // <
      // Obtain p, ds, tp from cache data block
      // an r from sphere object

      auto const& sp = *o;
      auto const& sc = cache_data[i];

      SphereGeomDataBlock s_geom{sc.in_p, scenario.rbSphereRadius(sp),
                                 sc.out_ds, sc.in_v, sc.in_tp, i,
                                 scenario.rbSphereMaxFrictionCoef(), scenario.rbMass(sp)};
      s_g_blocks.push_back(s_geom);
      //Make it work first then use the attached_kernel in line 99
      ValueType fd;
      fd=emulateLoss(scenario.rbPlaneMaxFrictionCoef(), scenario.rbSphereMaxFrictionCoef());
      StateDataBlock s_state{scenario.rbState(sp), attached(scenario.rbState(sp)), fd} ;
      st_blocks.push_back(s_state);
    }
    timer.~Timer();
    for (auto f : F) {
      InfPlaneGeomDataBlock p_geom{scenario.globalFramePosition(f),
                                   scenario.rbPlaneNormal(f), scenario.rbPlaneMaxFrictionCoef(), f};
      p_g_blocks.push_back(p_geom);
    }

    // Nested loop to set up data blocks for intersection detection
    for (auto s : s_g_blocks) {
      int j =0;
      for (auto p : p_g_blocks) {
        IntersectDetProcDataBlock i_data_block {s, p, {}, st_blocks[j]};
        int_data.push_back(i_data_block);
      }
      j++;
    }

    // Make a call to compute intersections
    computeIntersection(int_data, t_0, timestep);

    // Now, the intersection candidates reside in the int_data
    // so we sort and make unique
    makeSortAndUnique(int_data);
    checkState(int_data);

    // Then, handle collision event
    // 1. Take first element from int data
    // 2. Simulate sphere up to time of impact
    // 3. Compute intersection response (new velocity vector, new direction)
    // 4. Update cache properties (re-compute trajectory with updated velocity, update time point)
    // 5. Check for new collisions after the impact response

    while (not int_data.empty()) {
      // 1. Take first element
      checkState(int_data);
      auto const col = int_data.back();
      //int_data.pop_back();
      //Take the first element state information (i think i should sort them the same way i did with the others)
      //auto const st = state_data.back();
      //state_data.pop_back();


      // 2. Simulate sphere up to time of impact
      auto& c = cache_data.at(col.in_sphere.cache_index);
      //auto& st = state_data.at(col.in_sphere.cache_index);
      SimProcDataBlock s {c.in_p, c.out_ds, {}};
      sim_kernel(s);

      scenario.setPosition(c.rb_id, s.out_p);

      // 3. Compute impact response
      auto new_v = mechanics::computeImpactResponseSphereFixedPlane(
        c.in_v, col.in_plane.n);
      //Apply the energy loss after an impact response
      energyLoss(new_v, col.out_state.friction_coefT);
      //I don't know if this one is appropriate but one thing for sure there 2 more places
      //ou je dois appliquer l'energy loss et check state
      //new_v = mechanics::computeImpactResponseSphereFixedPlane(
      //  new_v, col.in_plane.n);
      scenario.setVelocity(c.rb_id, new_v);


      // 4. Update cache properties (modify in_parameter velocity, then re-compute trajectory)
      c.in_v = new_v;
      c.in_tp = col.out_status.col_tp;
      c.in_p = s.out_p;

      // should use cache computing kernel instead
      auto const &[ext_forces, dt] = env_params;
      auto traj_accel = mechanics::computeLinearTrajectory(c.in_v, ext_forces, dt);
      c.out_ds = traj_accel.first;
      c.out_a = traj_accel.second;
      // 5. check for new collisions
      IntersectDetProcData new_int_candidates;
      //State of the new collisions
      //StateDetProcData new_state_candidates;
      for(auto sts: st_blocks){
        for (auto p : p_g_blocks) {
          if (col.in_plane.rb_id != p.rb_id) {
            SphereGeomDataBlock sp_geom{
                                        c.in_p, col.in_sphere.r, c.out_ds, c.in_v, c.in_tp,
                                        col.in_sphere.cache_index, scenario.rbSphereMaxFrictionCoef(),
                                        scenario.rbMass(col.in_sphere.cache_index)};
            IntersectDetProcDataBlock i_det_proc_data_block{sp_geom, p, {}, sts};
            //StateDetProcDataBlock s_det_proc_data_block{sp_geom,p,sts};
            new_int_candidates.push_back(i_det_proc_data_block);
            //new_state_candidates.push_back(s_det_proc_data_block);
          }
        }
      }


      computeIntersection(new_int_candidates /*, env_params.F*/, t_0, timestep); // feilen skjer her (og kanskje 5)
       // Add new intersection candidates to the list of possible intersections
      for (auto candidate : new_int_candidates) {
        //int k=0;
        if (candidate.out_status.is_collision)
          int_data.push_back(candidate);
          //state_data.push_back(new_state_candidates[k]);
        // Sort again, since we might have appended new candidates
        makeSortAndUnique(int_data);
          //++k;
      }

      // Simulate all spheres up to the end of the timestep
      SimProcData sim_data;
      for (auto /*c*/ cache : cache_data) {
        SimProcDataBlock sph {/*c*/ cache.in_p, /*c*/ cache.out_ds, {}};
        sim_data.push_back(/*s*/ sph);
      }
      computeSimulation(sim_data);
      //Update the values of the ds as computed in the sim_data
      energyLoss(new_v, col.out_state.friction_coefT);
      checkState(int_data);


      // Write result of simulation back to original data objects (in scenario)
      SimProcData::const_iterator sim_proc_data_ptr = sim_data.begin();
      CacheProcData::const_iterator cache_proc_data_ptr = cache_data.begin();
      for (auto o : O) {

        scenario.setPosition(o, *sim_proc_data_ptr->out_p); // ???????
        scenario.setState(o, col.out_state.state );
        ++sim_proc_data_ptr;
        ++cache_proc_data_ptr;
      }
      int_data.pop_back();
    }
  }
}      // namespace dte3607::physengine::solver_dev::level3


#endif // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL3_H
