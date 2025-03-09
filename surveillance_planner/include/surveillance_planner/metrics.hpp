/**
 * @File: metrics.hpp
 * @Date: March 2024
 * @Author: James Swedeen
 *
 * @brief
 * Helper functions that define performance metrics on plans.
 **/

#ifndef SURVEILLANCE_PLANNING_METRICS_HPP
#define SURVEILLANCE_PLANNING_METRICS_HPP

/* C++ Headers */
#include<utility>
#include<vector>
#include<iostream>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Header */
#include<Eigen/Dense>

/* Street Graph Headers */
#include<street_graph/graph.hpp>
#include<street_graph/planning_graph.hpp>

/* Local Headers */
#include<surveillance_planner/hotspot.hpp>
#include<surveillance_planner/action.hpp>
#include<surveillance_planner/agent.hpp>
#include<surveillance_planner/helpers.hpp>

namespace plan
{
/**
 * @totalCostFunction
 *
 * @brief
 * Defines the cost of a plan based on the combination of the two and infinity norms of the ECR and response
 * times over the planning horizon.
 *
 * @parameters
 * plan: The plan to evaluate
 * sim_dt: The time step at which the output will be given
 * start_time: The start time of the plan
 * end_time: The end time of the plan
 * init_agents: The initial agents vector
 * init_hotspots: The set of positions of interest as they are at the start of the simulation
 * event_points: Locations of the events that must be responded to in terms of the index of the vertex in the original graph
 * street_graph: The street graph
 * ecr_two_norm_weight: The wight put on the two norm of the ECR part of the cost
 * ecr_inf_norm_weight: The wight put on the infinity norm of the ECR part of the cost
 * rt_two_norm_weight: The wight put on the two norm of the response time part of the cost
 * rt_inf_norm_weight: The wight put on the infinity norm of the response time part of the cost
 *
 * @templates
 * NUM_AGENTS: The number of agents being planned for
 * NUM_HOTSPOTS: The number of hotspots
 * NUM_EVENT_POINTS: The number of event points
 * MAX_PLAN_VISITS: The max length possible for a plan
 * CONSTRAINT_OPTIONS: Options that control what SOC constraints to use
 * EIG_OPTIONS: Eigen storage options
 * USE_ECR_TWO_NORM: True if the two norm of the ECR should be considered
 * USE_ECR_INF_NORM: True if the infinity norm of the ECR should be considered
 * USE_RT_TWO_NORM: True if the two norm of the response time should be considered
 * USE_RT_INF_NORM: True if the infinity norm of the response time should be considered
 *
 * @return
 * first: True iff the plan dose not violate any constraints
 * second: The cost of the plan
 **/
template<Eigen::Index          NUM_AGENTS,
         Eigen::Index          NUM_HOTSPOTS,
         Eigen::Index          NUM_EVENT_POINTS,
         Eigen::Index          MAX_PLAN_VISITS,
         ConstraintOptions     CONSTRAINT_OPTIONS,
         Eigen::StorageOptions EIG_OPTIONS      = Eigen::ColMajor bitor Eigen::AutoAlign,
         bool                  USE_ECR_TWO_NORM = true,
         bool                  USE_ECR_INF_NORM = true,
         bool                  USE_RT_TWO_NORM  = true,
         bool                  USE_RT_INF_NORM  = true>
inline std::pair<bool,double> totalCostFunction(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&              plan,
                                                const float                                                sim_dt,
                                                const float                                                start_time,
                                                const float                                                end_time,
                                                const AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>&             init_agents,
                                                const HOTSPOTS_VEC_TYPE<NUM_HOTSPOTS,EIG_OPTIONS>&         init_hotspots,
                                                const EVENT_POINTS_VEC_TYPE<NUM_EVENT_POINTS,EIG_OPTIONS>& event_points,
                                                const graph::Graph&                                        street_graph,
                                                const float                                                ecr_two_norm_weight,
                                                const float                                                ecr_inf_norm_weight,
                                                const float                                                rt_two_norm_weight,
                                                const float                                                rt_inf_norm_weight) noexcept;
/**
 * @planToSimulationVector
 *
 * @brief
 * Given a plan this function simulates the problem and produces a time indexed log of everything that happens.
 *
 * @parameters
 * plan: The plan to simulate
 * sim_dt: The time step at which the output will be given
 * init_agents: The initial agents vector
 * init_hotspots: The set of positions of interest as they are at the start of the simulation
 * start_time: The start time of the plan
 * end_time: The end time of the plan
 * depot_location: The location of the depot
 *
 * @templates
 * NUM_AGENTS: The number of agents being planned for
 * NUM_HOTSPOTS: The number of hotspots
 * NUM_EVENT_POINTS: The number of event points
 * MAX_PLAN_VISITS: The max length possible for a plan
 * CONSTRAINT_OPTIONS: Options that control what SOC constraints to use
 * EIG_OPTIONS: Eigen storage options
 *
 * @return
 * A vector where the columns are time indexed and the rows are:
 *   time
 *   first agent position x
 *   first agent position y
 *   first agent position z
 *   ...
 *   last agent position x
 *   last agent position y
 *   last agent position z
 *   first agent state of charge
 *   ...
 *   last agent state of charge
 *   first position of interest expected event rate
 *   ...
 *   last position of interest expected event rate
 **/
template<Eigen::Index          NUM_AGENTS,
         Eigen::Index          NUM_HOTSPOTS,
         Eigen::Index          NUM_EVENT_POINTS,
         Eigen::Index          MAX_PLAN_VISITS,
         ConstraintOptions     CONSTRAINT_OPTIONS,
         Eigen::StorageOptions EIG_OPTIONS      = Eigen::ColMajor bitor Eigen::AutoAlign>
Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,EIG_OPTIONS>
  planToSimulationVector(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&                     plan,
                         const float                                                       sim_dt,
                         const AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>&                    init_agents,
                         const HOTSPOTS_VEC_TYPE<NUM_HOTSPOTS,EIG_OPTIONS>&                init_hotspots,
                         const float                                                       start_time,
                         const float                                                       end_time,
                         const Eigen::Ref<const Eigen::Matrix<float,3,1,EIG_OPTIONS,3,1>>& depot_location,
                         const graph::Graph&                                               street_graph,
                         const EVENT_POINTS_VEC_TYPE<NUM_EVENT_POINTS,EIG_OPTIONS>&        event_points) noexcept;
/**
 * @responseTime
 *
 * @brief
 * Finds the minimum amount of time needed for an agent to get to a particular location without violating constraints.
 *
 * @parameters
 * event_points: Locations of the events that must be responded to in terms of the index of the vertex in the original graph
 * active_actions: The set of actions that currently have an agent in them and the simulation time the agent first started the action
 * street_graph: The street graph
 * cur_time: The current simulation time
 *
 * @templates
 * NUM_AGENTS: The number of agents being planned for
 * NUM_EVENT_POINTS: The number of event points
 * EIG_OPTIONS: Eigen storage options
 *
 * @return
 * A vector indexed by every point given with the value as the minimum response time, or infinity if not possible to respond.
 **/
template<Eigen::Index          NUM_AGENTS,
         Eigen::Index          NUM_EVENT_POINTS,
         Eigen::StorageOptions EIG_OPTIONS>
inline Eigen::Matrix<float,NUM_EVENT_POINTS,1,EIG_OPTIONS,NUM_EVENT_POINTS,1>
  responseTime(const EVENT_POINTS_VEC_TYPE<NUM_EVENT_POINTS,EIG_OPTIONS>&                                  event_points,
               const Eigen::Matrix<std::pair<const Action*,float >,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>& active_actions,
               const graph::Graph&                                                                         street_graph,
               const float                                                                                 cur_time) noexcept;
/**
 * @responseTimeWithConstraints
 *
 * @brief
 * Finds the minimum amount of time needed for an agent to get to a particular location without violating constraints.
 *
 * @parameters
 * event_points: Locations of the events that must be responded to in terms of the index of the vertex in the original graph
 * active_actions: The set of actions that currently have an agent in them and the simulation time the agent first started the action
 * agents: The agents at the correct state
 * street_graph: The street graph
 * cur_time: The current simulation time
 *
 * @templates
 * NUM_AGENTS: The number of agents being planned for
 * NUM_EVENT_POINTS: The number of event points
 * CONSTRAINT_OPTIONS: Options that control what SOC constraints to use
 * EIG_OPTIONS: Eigen storage options
 *
 * @return
 * A vector indexed by every point given with the value as the minimum response time, or infinity if not possible to respond.
 **/
template<Eigen::Index          NUM_AGENTS,
         Eigen::Index          NUM_EVENT_POINTS,
         ConstraintOptions     CONSTRAINT_OPTIONS,
         Eigen::StorageOptions EIG_OPTIONS>
inline Eigen::Matrix<float,NUM_EVENT_POINTS,1,EIG_OPTIONS,NUM_EVENT_POINTS,1>
  responseTimeWithConstraints(const EVENT_POINTS_VEC_TYPE<NUM_EVENT_POINTS,EIG_OPTIONS>&                                 event_points,
                              const Eigen::Matrix<std::pair<const Action*,float>,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>& active_actions,
                              const AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>&                                             agents,
                              const graph::Graph&                                                                        street_graph,
                              const float                                                                                cur_time) noexcept;
} // plan


template<Eigen::Index            NUM_AGENTS,
         Eigen::Index            NUM_HOTSPOTS,
         Eigen::Index            NUM_EVENT_POINTS,
         Eigen::Index            MAX_PLAN_VISITS,
         plan::ConstraintOptions CONSTRAINT_OPTIONS,
         Eigen::StorageOptions   EIG_OPTIONS,
         bool                    USE_ECR_TWO_NORM,
         bool                    USE_ECR_INF_NORM,
         bool                    USE_RT_TWO_NORM,
         bool                    USE_RT_INF_NORM>
inline std::pair<bool,double>
  plan::totalCostFunction(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&              plan,
                          const float                                                sim_dt,
                          const float                                                start_time,
                          const float                                                end_time,
                          const AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>&             init_agents,
                          const HOTSPOTS_VEC_TYPE<NUM_HOTSPOTS,EIG_OPTIONS>&         init_hotspots,
                          const EVENT_POINTS_VEC_TYPE<NUM_EVENT_POINTS,EIG_OPTIONS>& event_points,
                          const graph::Graph&                                        street_graph,
                          const float                                                ecr_two_norm_weight,
                          const float                                                ecr_inf_norm_weight,
                          const float                                                rt_two_norm_weight,
                          const float                                                rt_inf_norm_weight) noexcept
{
  const boost::integer_range<uint32_t> agent_inds(0, NUM_AGENTS);
  const boost::integer_range<uint32_t> hs_inds(   0, NUM_HOTSPOTS);
  const auto                           plan_end = plan.cend();

  #ifndef NDEBUG
    assert(plan[0].dwell_time >= 0);
    uint32_t plan_starts = 0;
    for(auto plan_it = plan.cbegin(); plan_it != plan_end; ++plan_it)
    {
      assert((*plan_it).dwell_time >= 0);
      if(nullptr != (*plan_it).prev_path)
      {
        assert((*std::prev(plan_it)).vertex_ind == (*plan_it).prev_path->from_vertex);
        assert((*plan_it).vertex_ind            == (*plan_it).prev_path->to_vertex);
      }
      else
      {
        ++plan_starts;
      }
    }
    assert(NUM_AGENTS == plan_starts);
  #endif

  //#define PRINT_WHY_FAILED

  /// Find where each agent sub-plan starts
  Eigen::Matrix<typename PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>::const_iterator,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> plan_inds;
  plan_inds[0] = plan.cbegin();
  std::for_each(std::next(agent_inds.begin()), agent_inds.end(),
  [&plan_inds, &plan, &plan_end] (const uint32_t agent_ind) -> void
  {
    const auto sub_plan_end = std::find_if(std::execution::unseq, std::next(plan_inds[agent_ind-1]), plan_end,
                                           [] (const Action& action_it) -> bool { return nullptr == action_it.prev_path; });
    assert((NUM_AGENTS - 1 == agent_ind) or (plan_end != sub_plan_end));
    plan_inds[agent_ind] = sub_plan_end;
  });

  /// Check that we start and end at the depot
  if((not graph::isDepot((*plan_inds[0]).vertex_ind)) or
     std::any_of(std::execution::unseq, std::next(agent_inds.begin()), agent_inds.end(),
     [&plan_inds] (const uint32_t agent_ind) -> bool
     {
       return (not graph::isDepot((*plan_inds[agent_ind]).vertex_ind)) or (not graph::isDepot((*plan_inds[agent_ind - 1]).vertex_ind));
     }) or
     (not graph::isDepot((*plan_inds.template bottomRows<1>()[0]).vertex_ind)))
  {
    #ifdef PRINT_WHY_FAILED
      std::cout << "start and end at depot violated" << std::endl;
    #endif
    return std::pair<bool,double>(false, std::numeric_limits<double>::infinity());
  }

  /// Check that SOC, distance, and time constraints are satisfied
  if(std::any_of(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
     [&] (const uint32_t agent_ind) -> bool
     {
        Agent      sub_agent    = init_agents[agent_ind];
        float      sub_time     = sub_agent.cgetShiftStartTime();
        auto       sub_plan_it  = plan_inds[agent_ind];
        const auto sub_plan_end = ((NUM_AGENTS - 1) == agent_ind) ? plan_end : plan_inds[agent_ind + 1];
        // First charge
        {
          const double charge_time = minutesToSeconds(sub_plan_it->dwell_time);
          if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
          {
            if(charge_time < sub_agent.maxChargeTime())
            {
              #ifdef PRINT_WHY_FAILED
                std::cout << "start charge time error. True: " << charge_time << " should be: " << sub_agent.maxChargeTime() << std::endl;
              #endif
              return true;
            }
            sub_agent.resetTripCounterInPlace();
          }
          else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS)) // Do normal charge
          {
            sub_agent.chargeForInPlace(charge_time);
          }
          sub_time += charge_time;
        }
        ++sub_plan_it;
        // Iterate through every action
        for( ; sub_plan_it != sub_plan_end; ++sub_plan_it)
        {
          // Check SOC and distance constraints
          bool soc_constraint_good      = true;
          bool distance_constraint_good = true;
          if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
          {
            soc_constraint_good = (sub_plan_it->prev_path->traversal_charge_needed <= sub_agent.cgetCurrentStateOfCharge());
          }
          if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
          {
            distance_constraint_good = ((sub_agent.cgetCurrentTripLength() + sub_plan_it->prev_path->length) <= sub_agent.cgetMaxTripLength());
          }
          if constexpr(verboseFlag(CONSTRAINT_OPTIONS))
          {
            if(soc_constraint_good and (not distance_constraint_good))
            {
              std::cout <<
                "SOC constraint is satisfied with error bound: " <<
                std::fabs(sub_plan_it->prev_path->traversal_charge_needed - sub_agent.cgetCurrentStateOfCharge()) <<
                " but distance constraint is violated with error bound: " <<
                std::fabs((sub_agent.cgetCurrentTripLength() + sub_plan_it->prev_path->length) - sub_agent.cgetMaxTripLength()) <<
                "\n";
            }
            if((not soc_constraint_good) and distance_constraint_good)
            {
              std::cout <<
                "Distance constraint is satisfied with error bound: " <<
                std::fabs((sub_agent.cgetCurrentTripLength() + sub_plan_it->prev_path->length) - sub_agent.cgetMaxTripLength()) <<
                " but SOC constraint is violated with error bound: " <<
                std::fabs(sub_plan_it->prev_path->traversal_charge_needed - sub_agent.cgetCurrentStateOfCharge()) <<
                "\n";
            }
          }
          if((not soc_constraint_good) or (not distance_constraint_good))
          {
            #ifdef PRINT_WHY_FAILED
              std::cout << "soc or distance violated" << std::endl;
            #endif
            return true;
          }
          // Propagate
          if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS) or distanceConstraintsFlag(CONSTRAINT_OPTIONS))
          {
            sub_agent.propagateAlongPathInPlace(*sub_plan_it->prev_path);
          }
          {
            const float dwell_time_sec = minutesToSeconds(sub_plan_it->dwell_time);
            if(graph::isDepot(sub_plan_it->vertex_ind))
            {
              if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
              {
                if(dwell_time_sec < sub_agent.maxChargeTime())
                {
                  #ifdef PRINT_WHY_FAILED
                    std::cout << "charge time violation" << std::endl;
                  #endif
                  return true;
                }
                sub_agent.resetTripCounterInPlace();
              }
              else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS)) // Do normal charge
              {
                sub_agent.chargeForInPlace(dwell_time_sec);
              }
            }
            else
            {
              sub_agent.propagateAlongTimeInPlace(dwell_time_sec);
            }
            sub_time += (sub_plan_it->prev_path->traversal_time + dwell_time_sec);
          }
          // Check time constraint
          if((sub_agent.cgetShiftEndTime() + float(1e-4)) < (sub_time - minutesToSeconds(std::prev(sub_plan_end)->dwell_time)))
          {
            #ifdef PRINT_WHY_FAILED
              std::cout << "time violated " << std::setprecision(std::numeric_limits<float>::max_digits10) << end_time << " " << sub_agent.cgetShiftEndTime() << " " << sub_time << std::endl;
            #endif
            return true;
          }
        }
//        if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
//        {
//          if(sub_agent.cgetCurrentStateOfCharge() < (init_agents[agent_ind].cgetCurrentStateOfCharge() - double(1e-4)))
//          {
//            #ifdef PRINT_WHY_FAILED
//              std::cout << "end soc violated, true: " << sub_agent.cgetCurrentStateOfCharge() << " min: " << init_agents[agent_ind].cgetCurrentStateOfCharge() << std::endl;
//            #endif
//            return true;
//          }
//        }
//        if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
//        {
//          if(sub_agent.maxChargeTime() > minutesToSeconds(std::prev(sub_plan_end)->dwell_time))
//          {
//            #ifdef PRINT_WHY_FAILED
//              std::cout << "end soc/distance violated" << std::endl;
//            #endif
//            return true;
//          }
//        }
        return false;
     }))
  {
    return std::pair<bool,double>(false, std::numeric_limits<double>::infinity());
  }

  // Cost counters
  double   ecr_two_norm_sum = 0;
  double   ecr_inf_norm     = 0;
  double   rt_two_norm_sum  = 0;
  double   rt_inf_norm      = 0;
  uint32_t num_points       = 0;

  HOTSPOTS_VEC_TYPE<NUM_HOTSPOTS,EIG_OPTIONS> hs(init_hotspots);

  // Generate active action list
  Eigen::Matrix<std::pair<const Action*,float>,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> active_actions;
  std::for_each(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
                [&active_actions,&plan_inds,&init_agents] (const uint32_t agent_ind) -> void
  {
    active_actions[agent_ind] = std::pair<const Action*,float>(&*plan_inds[agent_ind], init_agents[agent_ind].cgetShiftStartTime());
  });


  if constexpr(constraintsInResponseTimeFlag(CONSTRAINT_OPTIONS))
  {
    // Run though simulation
    float cur_time       = start_time;
    float next_save_time = std::min<float>(sim_dt, end_time - cur_time);

    Eigen::Matrix<bool,        NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> agent_plan_over        = Eigen::Matrix<bool,        NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>::Constant(false);
    Eigen::Matrix<bool,        NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> cur_path_traversed     = Eigen::Matrix<bool,        NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>::Constant(true);
    Eigen::Matrix<Eigen::Index,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> active_dwell_vert_inds = Eigen::Matrix<Eigen::Index,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>::Zero();
    AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>                           agent_before_path      = init_agents;

    // Starting cost evaluations
    {
      Eigen::Matrix<float,NUM_HOTSPOTS,    1,EIG_OPTIONS,NUM_HOTSPOTS,    1> ecrs;
      Eigen::Matrix<float,NUM_EVENT_POINTS,1,EIG_OPTIONS,NUM_EVENT_POINTS,1> res_times;
      if constexpr(USE_ECR_TWO_NORM or USE_ECR_INF_NORM)
      {
        ecrs = hs.unaryExpr([] (const Hotspot& hotspot) -> float { return hotspot.cgetExpectedEventRate(); });
      }
      if constexpr(USE_RT_TWO_NORM or USE_RT_INF_NORM)
      {
        res_times = responseTimeWithConstraints<NUM_AGENTS,NUM_EVENT_POINTS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(event_points, active_actions, agent_before_path, street_graph, cur_time);
      }
      if constexpr(USE_ECR_TWO_NORM)
      {
        ecr_two_norm_sum += ecrs.norm();
      }
      if constexpr(USE_ECR_INF_NORM)
      {
        ecr_inf_norm = std::max<float>(ecr_inf_norm, ecrs.template maxCoeff<Eigen::NaNPropagationOptions::PropagateFast>());
      }
      if constexpr(USE_RT_TWO_NORM)
      {
        rt_two_norm_sum += res_times.norm();
      }
      if constexpr(USE_RT_INF_NORM)
      {
        rt_inf_norm = std::max<float>(rt_inf_norm, res_times.template maxCoeff<Eigen::NaNPropagationOptions::PropagateFast>());
      }
      ++num_points;
    }

    while(cur_time < end_time)
    {
      // Find next action
      uint32_t min_plan_ind       = -1;
      bool     found_min_plan_ind = false;
      float   min_plan_time      = next_save_time - cur_time;
      for(uint32_t agent_it = 0; agent_it < NUM_AGENTS; ++agent_it)
      {
        if(not agent_plan_over[agent_it])
        {
          const float next_action_dt =   active_actions[agent_it].second
                                        + ((cur_path_traversed[agent_it]) ? minutesToSeconds(plan_inds[agent_it]->dwell_time) : float(0))
                                        + ((nullptr == plan_inds[agent_it]->prev_path) ? float(0) : plan_inds[agent_it]->prev_path->traversal_time)
                                        - cur_time;
          if(min_plan_time > next_action_dt)
          {
            min_plan_ind       = agent_it;
            found_min_plan_ind = true;
            min_plan_time      = next_action_dt;
          }
        }
      }
      // Propagate state
      if(0 != min_plan_time)
      {
        std::for_each(std::execution::unseq, hs.begin(), hs.end(),
                      [&active_dwell_vert_inds,min_plan_time] (plan::Hotspot& hs) -> void
        {
          if((active_dwell_vert_inds.array() == hs.cgetGraphIndex()).any())
          {
            hs.propagateAgentPresent(min_plan_time);
          }
          else
          {
            hs.propagateNoAgentPresent(min_plan_time);
          }
        });
      }
      cur_time += min_plan_time;
      if(found_min_plan_ind) // If action end time
      {
        // Move to next action
        if(cur_path_traversed[min_plan_ind])
        {
          // Propagate state during dwell time
          const float dwell_time_sec = minutesToSeconds(active_actions[min_plan_ind].first->dwell_time);
          if(graph::isDepot(active_actions[min_plan_ind].first->vertex_ind))
          {
            if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              agent_before_path[min_plan_ind].resetTripCounterInPlace();
            }
            else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS)) // Do normal charge
            {
              agent_before_path[min_plan_ind].chargeForInPlace(dwell_time_sec);
            }
          }
          else
          {
            agent_before_path[min_plan_ind].propagateAlongTimeInPlace(dwell_time_sec);
          }
          // Update loop variables
          active_dwell_vert_inds[min_plan_ind] = -1;
          ++plan_inds[min_plan_ind];
          agent_plan_over[min_plan_ind] = (plan_end == plan_inds[min_plan_ind]) or (nullptr == plan_inds[min_plan_ind]->prev_path);
          if(not agent_plan_over[min_plan_ind])
          {
            // Update active actions
            active_actions[    min_plan_ind].first  = &*plan_inds[min_plan_ind];
            cur_path_traversed[min_plan_ind]        = false;
            active_actions[    min_plan_ind].second = cur_time;
          }
        }
        else // Do dwell time now
        {
          // Propagate state along path
          if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS) or distanceConstraintsFlag(CONSTRAINT_OPTIONS))
          {
            agent_before_path[min_plan_ind].propagateAlongPathInPlace(*active_actions[min_plan_ind].first->prev_path);
          }
          // Update loop variables
          cur_path_traversed[    min_plan_ind] = true;
          active_dwell_vert_inds[min_plan_ind] = plan_inds[min_plan_ind]->vertex_ind;
        }
      }
      else // Time to save cost info off
      {
        next_save_time += std::min<float>(sim_dt, end_time - cur_time);
        // Update cost variables
        Eigen::Matrix<float,NUM_HOTSPOTS,    1,EIG_OPTIONS,NUM_HOTSPOTS,    1> ecrs;
        Eigen::Matrix<float,NUM_EVENT_POINTS,1,EIG_OPTIONS,NUM_EVENT_POINTS,1> res_times;
        if constexpr(USE_ECR_TWO_NORM or USE_ECR_INF_NORM)
        {
          ecrs = hs.unaryExpr([] (const Hotspot& hotspot) -> float { return hotspot.cgetExpectedEventRate(); });
        }
        if constexpr(USE_RT_TWO_NORM or USE_RT_INF_NORM)
        {
          res_times = responseTimeWithConstraints<NUM_AGENTS,NUM_EVENT_POINTS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(event_points, active_actions, agent_before_path, street_graph, cur_time);
        }
        if constexpr(USE_ECR_TWO_NORM)
        {
          ecr_two_norm_sum += ecrs.norm();
        }
        if constexpr(USE_ECR_INF_NORM)
        {
          ecr_inf_norm = std::max<float>(ecr_inf_norm, ecrs.template maxCoeff<Eigen::NaNPropagationOptions::PropagateFast>());
        }
        if constexpr(USE_RT_TWO_NORM)
        {
          rt_two_norm_sum += res_times.norm();
        }
        if constexpr(USE_RT_INF_NORM)
        {
          rt_inf_norm = std::max<float>(rt_inf_norm, res_times.template maxCoeff<Eigen::NaNPropagationOptions::PropagateFast>());
        }
        ++num_points;
      }
    }
  }
  else // Response time calculation will not consider constraints
  {
    // Run though simulation
    float cur_time       = start_time;
    float next_save_time = std::min<float>(sim_dt, end_time - cur_time);

    Eigen::Matrix<bool,        NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> agent_plan_over        = Eigen::Matrix<bool,        NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>::Constant(false);
    Eigen::Matrix<bool,        NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> cur_path_traversed     = Eigen::Matrix<bool,        NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>::Constant(true);
    Eigen::Matrix<Eigen::Index,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> active_dwell_vert_inds = Eigen::Matrix<Eigen::Index,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>::Zero();

    // Starting cost evaluations
    {
      Eigen::Matrix<float,NUM_HOTSPOTS,    1,EIG_OPTIONS,NUM_HOTSPOTS,    1> ecrs;
      Eigen::Matrix<float,NUM_EVENT_POINTS,1,EIG_OPTIONS,NUM_EVENT_POINTS,1> res_times;
      if constexpr(USE_ECR_TWO_NORM or USE_ECR_INF_NORM)
      {
        ecrs = hs.unaryExpr([] (const Hotspot& hotspot) -> float { return hotspot.cgetExpectedEventRate(); });
      }
      if constexpr(USE_RT_TWO_NORM or USE_RT_INF_NORM)
      {
        res_times = responseTime<NUM_AGENTS,NUM_EVENT_POINTS,EIG_OPTIONS>(event_points, active_actions, street_graph, cur_time);
      }
      if constexpr(USE_ECR_TWO_NORM)
      {
        ecr_two_norm_sum += ecrs.norm();
      }
      if constexpr(USE_ECR_INF_NORM)
      {
        ecr_inf_norm = std::max<float>(ecr_inf_norm, ecrs.template maxCoeff<Eigen::NaNPropagationOptions::PropagateFast>());
      }
      if constexpr(USE_RT_TWO_NORM)
      {
        rt_two_norm_sum += res_times.norm();
      }
      if constexpr(USE_RT_INF_NORM)
      {
        rt_inf_norm = std::max<float>(rt_inf_norm, res_times.template maxCoeff<Eigen::NaNPropagationOptions::PropagateFast>());
      }
      ++num_points;
    }

    while(cur_time < end_time)
    {
      // Find next action
      uint32_t min_plan_ind       = -1;
      bool     found_min_plan_ind = false;
      float    min_plan_time      = next_save_time - cur_time;
      for(uint32_t agent_it = 0; agent_it < NUM_AGENTS; ++agent_it)
      {
        if(not agent_plan_over[agent_it])
        {
          const float next_action_dt =   active_actions[agent_it].second
                                        + ((cur_path_traversed[agent_it]) ? minutesToSeconds(plan_inds[agent_it]->dwell_time) : float(0))
                                        + ((nullptr == plan_inds[agent_it]->prev_path) ? float(0) : plan_inds[agent_it]->prev_path->traversal_time)
                                        - cur_time;
          if(min_plan_time > next_action_dt)
          {
            min_plan_ind       = agent_it;
            found_min_plan_ind = true;
            min_plan_time      = next_action_dt;
          }
        }
      }
      // Propagate state
      if(0 != min_plan_time)
      {
        std::for_each(std::execution::unseq, hs.begin(), hs.end(),
                      [&active_dwell_vert_inds,min_plan_time] (plan::Hotspot& hs) -> void
        {
          if((active_dwell_vert_inds.array() == hs.cgetGraphIndex()).any())
          {
            hs.propagateAgentPresent(min_plan_time);
          }
          else
          {
            hs.propagateNoAgentPresent(min_plan_time);
          }
        });
      }
      cur_time += min_plan_time;
      if(found_min_plan_ind) // If action end time
      {
        // Move to next action
        if(cur_path_traversed[min_plan_ind])
        {
          active_dwell_vert_inds[min_plan_ind] = -1;
          ++plan_inds[min_plan_ind];
          agent_plan_over[min_plan_ind] = (plan_end == plan_inds[min_plan_ind]) or (nullptr == plan_inds[min_plan_ind]->prev_path);
          if(not agent_plan_over[min_plan_ind])
          {
            // Update active actions
            active_actions[    min_plan_ind].first  = &*plan_inds[min_plan_ind];
            cur_path_traversed[min_plan_ind]        = false;
            active_actions[    min_plan_ind].second = cur_time;
          }
        }
        else // Do dwell time now
        {
          cur_path_traversed[    min_plan_ind] = true;
          active_dwell_vert_inds[min_plan_ind] = plan_inds[min_plan_ind]->vertex_ind;
        }
      }
      else // Time to save cost info off
      {
        next_save_time += std::min<float>(sim_dt, end_time - cur_time);
        // Update cost variables
        Eigen::Matrix<float,NUM_HOTSPOTS,    1,EIG_OPTIONS,NUM_HOTSPOTS,    1> ecrs;
        Eigen::Matrix<float,NUM_EVENT_POINTS,1,EIG_OPTIONS,NUM_EVENT_POINTS,1> res_times;
        if constexpr(USE_ECR_TWO_NORM or USE_ECR_INF_NORM)
        {
          ecrs = hs.unaryExpr([] (const Hotspot& hotspot) -> float { return hotspot.cgetExpectedEventRate(); });
        }
        if constexpr(USE_RT_TWO_NORM or USE_RT_INF_NORM)
        {
          res_times = responseTime<NUM_AGENTS,NUM_EVENT_POINTS,EIG_OPTIONS>(event_points, active_actions, street_graph, cur_time);
        }
        if constexpr(USE_ECR_TWO_NORM)
        {
          ecr_two_norm_sum += ecrs.norm();
        }
        if constexpr(USE_ECR_INF_NORM)
        {
          ecr_inf_norm = std::max<float>(ecr_inf_norm, ecrs.template maxCoeff<Eigen::NaNPropagationOptions::PropagateFast>());
        }
        if constexpr(USE_RT_TWO_NORM)
        {
          rt_two_norm_sum += res_times.norm();
        }
        if constexpr(USE_RT_INF_NORM)
        {
          rt_inf_norm = std::max<float>(rt_inf_norm, res_times.template maxCoeff<Eigen::NaNPropagationOptions::PropagateFast>());
        }
        ++num_points;
      }
    }
  }

  // Calculate final cost
  double cost = 0;
  double int_norm_const;
  if constexpr(USE_ECR_TWO_NORM or USE_RT_TWO_NORM)
  {
    int_norm_const = (end_time - start_time) / double(num_points);
  }
  if constexpr(USE_ECR_TWO_NORM)
  {
    cost += (ecr_two_norm_sum * int_norm_const * ecr_two_norm_weight);
  }
  if constexpr(USE_ECR_INF_NORM)
  {
    cost += (ecr_inf_norm * ecr_inf_norm_weight);
  }
  if constexpr(USE_RT_TWO_NORM)
  {
    cost += (rt_two_norm_sum * int_norm_const * rt_two_norm_weight);
  }
  if constexpr(USE_RT_INF_NORM)
  {
    cost += (rt_inf_norm * rt_inf_norm_weight);
  }

  return std::pair<bool,double>(true, cost);
}

template<Eigen::Index            NUM_AGENTS,
         Eigen::Index            NUM_HOTSPOTS,
         Eigen::Index            NUM_EVENT_POINTS,
         Eigen::Index            MAX_PLAN_VISITS,
         plan::ConstraintOptions CONSTRAINT_OPTIONS,
         Eigen::StorageOptions   EIG_OPTIONS>
Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,EIG_OPTIONS>
  plan::planToSimulationVector(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&                     plan,
                               const float                                                       sim_dt,
                               const AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>&                    init_agents,
                               const HOTSPOTS_VEC_TYPE<NUM_HOTSPOTS,EIG_OPTIONS>&                init_hotspots,
                               const float                                                       start_time,
                               const float                                                       end_time,
                               const Eigen::Ref<const Eigen::Matrix<float,3,1,EIG_OPTIONS,3,1>>& depot_location,
                               const graph::Graph&                                               street_graph,
                               const EVENT_POINTS_VEC_TYPE<NUM_EVENT_POINTS,EIG_OPTIONS>&        event_points) noexcept
{
  const uint32_t num_time_steps = std::max<uint32_t>(uint32_t(std::ceil((end_time-start_time)/sim_dt))+2, 2);
  const uint32_t num_rows       = 2 + (NUM_AGENTS * 4) + NUM_HOTSPOTS + NUM_EVENT_POINTS;
  const auto     plan_end       = plan.cend();

  const uint32_t TIME_IND      = 0;
  const uint32_t START_POS_IND = 1;
  const uint32_t START_SOC_IND = 1 + (NUM_AGENTS * 3);
  const uint32_t START_HS_IND  = START_SOC_IND + NUM_AGENTS;
  const uint32_t START_RT_IND  = START_HS_IND + NUM_HOTSPOTS;

  const boost::integer_range<uint32_t> agent_inds(0, NUM_AGENTS);
  const boost::integer_range<uint32_t> hs_inds(   0, NUM_HOTSPOTS);

  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,EIG_OPTIONS> output(num_rows, num_time_steps);
  AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>                         agents(init_agents);
  HOTSPOTS_VEC_TYPE<NUM_HOTSPOTS,EIG_OPTIONS>                     hs(init_hotspots);

  /// Find where each agent sub-plan starts
  Eigen::Matrix<typename PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>::const_iterator,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> plan_inds(NUM_AGENTS);
  plan_inds[0] = plan.cbegin();
  std::for_each(std::next(agent_inds.begin()), agent_inds.end(),
  [&plan_inds, &plan, &plan_end] (const uint32_t agent_ind) -> void
  {
    const auto sub_plan_end = std::find_if(std::execution::unseq, std::next(plan_inds[agent_ind-1]), plan_end,
                                           [] (const Action& action_it) -> bool { return nullptr == action_it.prev_path; });
    assert((NUM_AGENTS - 1 == agent_ind) or (plan_end != sub_plan_end));
    plan_inds[agent_ind] = sub_plan_end;
  });

  /// Run though simulation
  Eigen::Index output_ind     = 1;
  double       cur_time       = start_time;
  double       next_save_time = std::min<double>(sim_dt, end_time - cur_time);

  Eigen::Matrix<bool,        NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> agent_plan_over        = Eigen::Matrix<bool,        NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>::Constant(NUM_AGENTS, 1, false);
  Eigen::Matrix<bool,        NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> cur_path_traversed     = Eigen::Matrix<bool,        NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>::Constant(NUM_AGENTS, 1, true);
  Eigen::Matrix<Eigen::Index,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> active_dwell_vert_inds = Eigen::Matrix<Eigen::Index,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>::Zero(    NUM_AGENTS);
  AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>                           agent_before_path      = agents;

  // Generate active action list
  Eigen::Matrix<std::pair<const Action*,float>,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> active_actions(NUM_AGENTS);
  std::for_each(agent_inds.begin(), agent_inds.end(),
                [&active_actions,&plan_inds,&init_agents] (const uint32_t agent_ind) -> void
  {
    active_actions[agent_ind] = std::pair<const Action*,float>(&*plan_inds[agent_ind], init_agents[agent_ind].cgetShiftStartTime());
  });

  // Copy starting states
  output(TIME_IND, 0) = start_time;
  std::for_each(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
    [&output, START_POS_IND, &plan, &depot_location] (const uint32_t agent_ind) -> void
    {
      const uint32_t base_ind = START_POS_IND + (3 * agent_ind);
      output(Eigen::seq(base_ind, base_ind + 2), 0) = depot_location.template cast<double>();
    });
  output.template block<NUM_AGENTS,      1>(START_SOC_IND, 0) = agents.unaryExpr([] (const Agent&   agent)   -> double { return agent.  cgetCurrentStateOfCharge(); });
  output.template block<NUM_HOTSPOTS,    1>(START_HS_IND,  0) = hs.    unaryExpr([] (const Hotspot& hotspot) -> double { return hotspot.cgetExpectedEventRate();    });
  output.template block<NUM_EVENT_POINTS,1>(START_RT_IND,  0) = responseTimeWithConstraints<NUM_AGENTS,NUM_EVENT_POINTS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(event_points, active_actions, agent_before_path, street_graph, cur_time).template cast<double>();

  // Simulate
  while(cur_time < end_time)
  {
    // Find next action
    Eigen::Index min_plan_ind = -1;
    double       min_plan_time = next_save_time - cur_time;
    for(Eigen::Index agent_it = 0; agent_it < NUM_AGENTS; ++agent_it)
    {
      if(not agent_plan_over[agent_it])
      {
        const double next_action_dt =   double(active_actions[agent_it].second)
                                      + ((cur_path_traversed[agent_it]) ? minutesToSeconds(plan_inds[agent_it]->dwell_time) : double(0))
                                      + ((nullptr == plan_inds[agent_it]->prev_path) ? double(0) : double(plan_inds[agent_it]->prev_path->traversal_time))
                                      - cur_time;
        if(min_plan_time > next_action_dt)
        {
          min_plan_ind  = agent_it;
          min_plan_time = next_action_dt;
        }
      }
    }
    // Propagate state
    if(0 != min_plan_time)
    {
      for(uint32_t agent_it = 0; agent_it < NUM_AGENTS; ++agent_it)
      {
        if(not agent_plan_over[agent_it])
        {
          const double start_prop = cur_time - active_actions[agent_it].second;

          if(cur_path_traversed[agent_it])
          {
            if(graph::isDepot(plan_inds[agent_it]->vertex_ind))
            {
              agents[agent_it].chargeForInPlace(min_plan_time);
            }
            else
            {
              agents[agent_it].propagateAlongTimeInPlace(min_plan_time);
            }
          }
          else // Path not traversed
          {
            agents[agent_it].propagateAlongPathInPlace(*plan_inds[agent_it]->prev_path, start_prop, start_prop + min_plan_time);
          }
        }
      }
      std::for_each(hs.begin(), hs.end(),
      [&active_dwell_vert_inds,min_plan_time] (plan::Hotspot& hs) -> void
      {
        if((active_dwell_vert_inds.array() == hs.cgetGraphIndex()).any())
        {
          hs.propagateAgentPresent(min_plan_time);
        }
        else
        {
          hs.propagateNoAgentPresent(min_plan_time);
        }
      });
    }
    cur_time += min_plan_time;
    if(min_plan_ind != -1) // If action end time
    {
      if(cur_path_traversed[min_plan_ind])
      {
        ++plan_inds[min_plan_ind];
        active_dwell_vert_inds[min_plan_ind] = -1;
        agent_plan_over[min_plan_ind] = (plan_end == plan_inds[min_plan_ind]) or (nullptr == plan_inds[min_plan_ind]->prev_path);
        if(not agent_plan_over[min_plan_ind])
        {
          cur_path_traversed[   min_plan_ind] = false;
          agent_before_path[    min_plan_ind] = agents[min_plan_ind];
          active_actions[       min_plan_ind].first  = &*plan_inds[min_plan_ind];
          active_actions[       min_plan_ind].second = cur_time;
        }
      }
      else // Do dwell time now
      {
        cur_path_traversed[min_plan_ind] = true;
        active_dwell_vert_inds[min_plan_ind] = plan_inds[min_plan_ind]->vertex_ind;
        agents[min_plan_ind] = agent_before_path[min_plan_ind].propagateAlongPath(*plan_inds[min_plan_ind]->prev_path);
      }
    }
    if(next_save_time == cur_time)
    {
      next_save_time += std::min<double>(sim_dt, end_time - cur_time);
      // Save state
      output(TIME_IND, output_ind) = cur_time;

      std::for_each(agent_inds.begin(), agent_inds.end(),
        [&output, &plan_inds, cur_time, output_ind, START_POS_IND, &plan, &depot_location, &cur_path_traversed, &active_actions, &agent_plan_over] (const uint32_t agent_ind) -> void
        {
          const uint32_t base_ind = START_POS_IND + (3 * agent_ind);

          if(agent_plan_over[agent_ind])
          {
            output(Eigen::seq(base_ind, base_ind + 2), output_ind) = output(Eigen::seq(base_ind, base_ind + 2), output_ind-1);
          }
          else
          {
            if(not cur_path_traversed[agent_ind])
            {
              output(Eigen::seq(base_ind, base_ind + 2), output_ind) = plan_inds[agent_ind]->prev_path->stateAtTime(cur_time - active_actions[agent_ind].second).template cast<double>();
            }
            else // Dwelling
            {
              if(graph::isDepot(plan_inds[agent_ind]->vertex_ind))
              {
                output(Eigen::seq(base_ind, base_ind + 2), output_ind) = depot_location.template cast<double>();
              }
              else // Not at root node
              {
                output(Eigen::seq(base_ind, base_ind + 2), output_ind) = plan_inds[agent_ind]->prev_path->cgetSubEdges().back()->cgetToNode()->cgetPosition().template cast<double>();
              }
            }
          }
        });
      output.template block<NUM_AGENTS,      1>(START_SOC_IND, output_ind) = agents.unaryExpr([] (const Agent&   agent)   -> double { return agent.  cgetCurrentStateOfCharge(); });
      output.template block<NUM_HOTSPOTS,    1>(START_HS_IND,  output_ind) = hs.    unaryExpr([] (const Hotspot& hotspot) -> double { return hotspot.cgetExpectedEventRate();    });
      output.template block<NUM_EVENT_POINTS,1>(START_RT_IND,  output_ind) = responseTimeWithConstraints<NUM_AGENTS,NUM_EVENT_POINTS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(event_points, active_actions, agent_before_path, street_graph, cur_time).template cast<double>();
      ++output_ind;
    }
  }

  output.conservativeResize(Eigen::NoChange, output_ind);
  assert(output(TIME_IND, 0)           == start_time);
  assert(output(TIME_IND, Eigen::last) == end_time);

  return output;
}

template<Eigen::Index NUM_AGENTS, Eigen::Index NUM_EVENT_POINTS, Eigen::StorageOptions EIG_OPTIONS>
inline Eigen::Matrix<float,NUM_EVENT_POINTS,1,EIG_OPTIONS,NUM_EVENT_POINTS,1>
  plan::responseTime(const EVENT_POINTS_VEC_TYPE<NUM_EVENT_POINTS,EIG_OPTIONS>&                                  event_points,
                     const Eigen::Matrix<std::pair<const Action*,float>,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>& active_actions,
                     const graph::Graph&                                                                         street_graph,
                     const float                                                                                cur_time) noexcept
{
  const boost::integer_range<uint32_t> event_inds(0, NUM_EVENT_POINTS);
  const boost::integer_range<uint32_t> agent_inds(0, NUM_AGENTS);

  // Agent's next vertex, time it takes to get there
  Eigen::Matrix<std::tuple<uint32_t,float>,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> agents_next_node;
  std::for_each(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
                [&agents_next_node,&street_graph,&active_actions,cur_time] (const uint32_t agent_ind) -> void
  {
    if(nullptr == active_actions[agent_ind].first->prev_path)
    {
      agents_next_node[agent_ind] = std::tuple<uint32_t,float>(0, 0);
    }
    else // Not charging
    {
      const float edge_time = cur_time - active_actions[agent_ind].second;

      if(edge_time < active_actions[agent_ind].first->prev_path->traversal_time)
      {
        const std::pair<const graph::Edge*,float> active_edge = active_actions[agent_ind].first->prev_path->edgeInUseAtTime(edge_time);
        agents_next_node[agent_ind] = std::tuple<uint32_t,float>(
          active_edge.first->cgetToNode()->cgetGraphIndex(),
          active_edge.first->cgetMinTraversalTime() - (edge_time - active_edge.second));
      }
      else // Dwelling
      {
        agents_next_node[agent_ind] = std::tuple<uint32_t,float>(active_actions[agent_ind].first->prev_path->cgetSubEdges().back()->cgetToNode()->cgetGraphIndex(), 0);
      }
    }
  });

  // Find min response time assuming a particular event edge is used
  const auto one_edge_func = [&active_actions,&street_graph,&agent_inds,&agents_next_node]
                             (const uint32_t event_vertex_ind) -> float
  {
    Eigen::Matrix<float,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> pos_res_times;

    // Calculate all possible response times without charging
    std::for_each(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
                  [&] (const uint32_t agent_ind) -> void
    {
      pos_res_times[agent_ind] = std::get<1>(agents_next_node[agent_ind]);
      if(std::get<0>(agents_next_node[agent_ind]) == event_vertex_ind)
      {
        return;
      }
      pos_res_times[agent_ind] += street_graph.minTravelTime(std::get<0>(agents_next_node[agent_ind]), event_vertex_ind);
    });

    return pos_res_times.template minCoeff<Eigen::NaNPropagationOptions::PropagateFast>();
  };

  Eigen::Matrix<float,NUM_EVENT_POINTS,1,EIG_OPTIONS,NUM_EVENT_POINTS,1> output;
  std::for_each(std::execution::unseq, event_inds.begin(), event_inds.end(),
                [&event_points,one_edge_func,&output] (const uint32_t event_ind) -> void
  {
    output[event_ind] = one_edge_func(event_points[event_ind]);
  });

  return output;
}

template<Eigen::Index            NUM_AGENTS,
         Eigen::Index            NUM_EVENT_POINTS,
         plan::ConstraintOptions CONSTRAINT_OPTIONS,
         Eigen::StorageOptions   EIG_OPTIONS>
inline Eigen::Matrix<float,NUM_EVENT_POINTS,1,EIG_OPTIONS,NUM_EVENT_POINTS,1>
  plan::responseTimeWithConstraints(const EVENT_POINTS_VEC_TYPE<NUM_EVENT_POINTS,EIG_OPTIONS>&                                 event_points,
                                    const Eigen::Matrix<std::pair<const Action*,float>,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>& active_actions,
                                    const AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>&                                             agents,
                                    const graph::Graph&                                                                        street_graph,
                                    const float                                                                                cur_time) noexcept
{
  const boost::integer_range<uint32_t> event_inds(0, NUM_EVENT_POINTS);
  const boost::integer_range<uint32_t> agent_inds(0, NUM_AGENTS);

  // Agent's next vertex, time it takes to get there, charge it has when there, and the distance since last charge when there
  Eigen::Matrix<std::tuple<uint32_t,float,float,float>,NUM_AGENTS,1,EIG_OPTIONS> agents_next_node;
  std::for_each(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
                [&agents_next_node,&street_graph,&active_actions,&agents,cur_time] (const uint32_t agent_ind) -> void
  {
    if((cur_time < agents[agent_ind].cgetShiftStartTime()) or (cur_time > agents[agent_ind].cgetShiftEndTime()))
    {
      agents_next_node[agent_ind] = std::tuple<uint32_t,float,float,float>(0, std::numeric_limits<float>::infinity(), agents[agent_ind].cgetCurrentStateOfCharge(), agents[agent_ind].cgetCurrentTripLength());
      return;
    }
    if(nullptr == active_actions[agent_ind].first->prev_path)
    {
      agents_next_node[agent_ind] = std::tuple<uint32_t,float,float,float>(0, 0, agents[agent_ind].cgetCurrentStateOfCharge(), agents[agent_ind].cgetCurrentTripLength());
    }
    else // Not charging
    {
      const double edge_time = cur_time - active_actions[agent_ind].second;

      if(edge_time < active_actions[agent_ind].first->prev_path->traversal_time)
      {
        const std::tuple<const graph::Edge*,float,float,float> active_edge = active_actions[agent_ind].first->prev_path->edgeInUseAtTimePlus(edge_time);
        agents_next_node[agent_ind] = std::tuple<uint32_t,float,float,float>(
          std::get<0>(active_edge)->cgetToNode()->cgetGraphIndex(),
          std::get<0>(active_edge)->cgetMinTraversalTime() - (edge_time - std::get<1>(active_edge)),
          std::min<float>(agents[agent_ind].cgetCurrentStateOfCharge() - std::get<2>(active_edge),
                          agents[agent_ind].cgetMaxStateOfCharge()     - active_actions[agent_ind].first->prev_path->end_traversal_charge),
          agents[agent_ind].cgetCurrentTripLength() + std::get<3>(active_edge)
        );
      }
      else // Dwelling
      {
        agents_next_node[agent_ind] = std::tuple<uint32_t,float,float,float>(
          active_actions[agent_ind].first->prev_path->cgetSubEdges().back()->cgetToNode()->cgetGraphIndex(),
          0,
          agents[agent_ind].cgetCurrentStateOfCharge(),
          agents[agent_ind].cgetCurrentTripLength()
        );
      }
    }
  });

  // Find min response time assuming a particular event edge is used
  const auto one_edge_func = [&active_actions,&agents,&street_graph,&agent_inds,&agents_next_node]
                             (const uint32_t event_vertex_ind) -> float
  {
    Eigen::Matrix<float,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> pos_res_times;
    // Calculate all possible response times without charging
    std::for_each(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
                  [&] (const uint32_t agent_ind) -> void
    {
      if(std::get<0>(agents_next_node[agent_ind]) == event_vertex_ind)
      {
        pos_res_times[agent_ind] = std::get<1>(agents_next_node[agent_ind]);
        return;
      }

      // Find to event path info
      float to_event_time;
      float to_event_charge;
      float to_event_charge_needed;
      float to_event_distance;
      std::tie(to_event_time, to_event_charge, to_event_charge_needed, to_event_distance) = street_graph.minTravelTimeFull(std::get<0>(agents_next_node[agent_ind]), event_vertex_ind);

      float at_event_soc;
      float at_event_distance;
      if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
      {
        at_event_soc = std::min<float>(std::get<2>(agents_next_node[agent_ind]) - to_event_charge,
                                       agents[agent_ind].cgetMaxStateOfCharge());
      }
      if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
      {
        at_event_distance = std::get<3>(agents_next_node[agent_ind]) + to_event_distance;
      }

      // Find to depot path info
      if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
      {
        float to_depot_charge_needed;
        float to_depot_distance;

        std::tie(std::ignore, std::ignore, to_depot_charge_needed, to_depot_distance) = street_graph.minLengthFull(event_vertex_ind, 0);

        // Check SOC constraints
        if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
        {
          if((to_event_charge_needed > std::get<2>(agents_next_node[agent_ind])) or
             (to_depot_charge_needed > at_event_soc))
          {
            pos_res_times[agent_ind] = std::numeric_limits<float>::infinity();
            return;
          }
        }
        // Check distance constraints
        if((at_event_distance + to_depot_distance) > agents[agent_ind].cgetMaxTripLength())
        {
          pos_res_times[agent_ind] = std::numeric_limits<float>::infinity();
          return;
        }
        pos_res_times[agent_ind] = std::get<1>(agents_next_node[agent_ind]) + to_event_time;
        return;
      }
      if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
      {
        float to_depot_charge_needed;
        float to_depot_distance;

        std::tie(std::ignore, std::ignore, to_depot_charge_needed, to_depot_distance) = street_graph.minChargeNecessaryFull(event_vertex_ind, 0);

        // Check distance constraints
        if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
        {
          if((at_event_distance + to_depot_distance) > agents[agent_ind].cgetMaxTripLength())
          {
            pos_res_times[agent_ind] = std::numeric_limits<float>::infinity();
            return;
          }
        }
        // Check SOC constraints
        if((to_event_charge_needed > std::get<2>(agents_next_node[agent_ind])) or
           (to_depot_charge_needed > at_event_soc))
        {
          pos_res_times[agent_ind] = std::numeric_limits<float>::infinity();
          return;
        }
      }
      pos_res_times[agent_ind] = std::get<1>(agents_next_node[agent_ind]) + to_event_time;
    });

    return pos_res_times.template minCoeff<Eigen::NaNPropagationOptions::PropagateFast>();
  };

  Eigen::Matrix<float,NUM_EVENT_POINTS,1,EIG_OPTIONS,NUM_EVENT_POINTS,1> output;
  std::for_each(std::execution::unseq, event_inds.begin(), event_inds.end(),
                [&event_points,one_edge_func,&output] (const uint32_t event_ind) -> void
  {
    output[event_ind] = one_edge_func(event_points[event_ind]);
  });

  return output;
}

#endif
/* metrics.hpp */
