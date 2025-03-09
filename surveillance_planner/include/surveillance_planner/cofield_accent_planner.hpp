/**
 * @File: cofield_accent_planner.hpp
 * @Date: March 2024
 * @Author: James Swedeen
 *
 * @brief
 * Plans paths for agents by picking from the valid actions at each time point and picking the one goes in the
 * direction of max accent in the cofield.
 *
 * @cite
 * Cooper, John R. "Optimal multi-agent search and rescue using potential field theory." AIAA Scitech 2020 Forum. 2020.
 **/

#ifndef SURVEILLANCE_PLANNING_COFIELD_ACCENT_HPP
#define SURVEILLANCE_PLANNING_COFIELD_ACCENT_HPP

/* C++ Headers */
#include<memory>
#include<vector>
#include<chrono>
#include<algorithm>
#include<execution>
#include<iostream>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Street Graph Headers */
#include<street_graph/planning_graph.hpp>

/* Local Headers */
#include<surveillance_planner/hotspot.hpp>
#include<surveillance_planner/agent.hpp>
#include<surveillance_planner/action.hpp>
#include<surveillance_planner/helpers.hpp>

namespace plan
{
/**
 * @generateCofieldAccentPlan
 *
 * @brief
 * Planner that picks actions in the direction of greatest cofield accent from the set of all valid actions.
 *
 * @parameters
 * graph: The graph to plan over
 * agents: The agents to plan paths for
 * hotspots: A set of places to survey
 * dwell_time: The time to spend dwelling at hotspots
 * start_time: The start time of the plan
 * end_time: The end time of the plan
 * searching_weight: Weight applied to the POI terms in the cofield
 * agent_avoidance_weight: Weight applied to pushing the agents away from each other
 * hotspot_grpf_width_parameter: The width of the GRPB weights on hotspots
 *
 * @templates
 * NUM_AGENTS: The number of agents being planned for
 * NUM_HOTSPOTS: The number of hotspots
 * MAX_PLAN_VISITS: The max length possible for a plan
 * CONSTRAINT_OPTIONS: Options that control what SOC constraints to use
 * EIG_OPTIONS: Eigen storage options
 *
 * @return
 * Vector with each index being an action.
 **/
template<Eigen::Index          NUM_AGENTS,
         Eigen::Index          NUM_HOTSPOTS,
         Eigen::Index          MAX_PLAN_VISITS,
         ConstraintOptions     CONSTRAINT_OPTIONS,
         Eigen::StorageOptions EIG_OPTIONS = Eigen::ColMajor bitor Eigen::AutoAlign>
std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  generateCofieldAccentPlan(const graph::PlanningGraph&                        graph,
                            const AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>&     agents,
                            const HOTSPOTS_VEC_TYPE<NUM_HOTSPOTS,EIG_OPTIONS>& hotspots,
                            const uint32_t                                     dwell_time,
                            const float                                        start_time,
                            const float                                        end_time,
                            const float                                        searching_weight,
                            const float                                        agent_avoidance_weight,
                            const float                                        hotspot_grpf_width_parameter);
} // plan


template<Eigen::Index            NUM_AGENTS,
         Eigen::Index            NUM_HOTSPOTS,
         Eigen::Index            MAX_PLAN_VISITS,
         plan::ConstraintOptions CONSTRAINT_OPTIONS,
         Eigen::StorageOptions   EIG_OPTIONS>
std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::generateCofieldAccentPlan(const graph::PlanningGraph&                        graph,
                                  const AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>&     agents,
                                  const HOTSPOTS_VEC_TYPE<NUM_HOTSPOTS,EIG_OPTIONS>& hotspots,
                                  const uint32_t                                     dwell_time,
                                  const float                                        start_time,
                                  const float                                        end_time,
                                  const float                                        searching_weight,
                                  const float                                        agent_avoidance_weight,
                                  const float                                        hotspot_grpf_width_parameter)
{
  // Checks for validity
  assert(start_time <= end_time);

  // Helper definitions
  const boost::integer_range<uint32_t> agent_inds( 0, NUM_AGENTS);
  const boost::integer_range<uint32_t> hs_inds(    0, NUM_HOTSPOTS);
  const boost::integer_range<uint32_t> vertex_inds(0, graph.numberVertices());
  const float                          dwell_time_sec = minutesToSeconds(dwell_time);

  // Make mutable states
  AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>     m_agents(agents);
  HOTSPOTS_VEC_TYPE<NUM_HOTSPOTS,EIG_OPTIONS> m_hotspots(hotspots);

  // Plan
  std::vector<std::vector<Action>> output(NUM_AGENTS);

  // Run though simulation
  float cur_time = start_time;

  Eigen::Matrix<uint32_t,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> active_dwell_vert_inds = Eigen::Matrix<uint32_t,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>::Zero();
  Eigen::Matrix<bool,    NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> cur_path_traversed     = Eigen::Matrix<bool,    NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>::Constant(true);
  Eigen::Matrix<float,   NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> cur_action_start_time;
  Eigen::Index                                                  num_actions_in_plan          = NUM_AGENTS;
  Eigen::Index                                                  num_agent_need_path_to_depot = 0;

  // Keep track of action end times
  Eigen::Matrix<float,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> action_end_times;
  // Make first action selection
  std::for_each(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
  [&] (const uint32_t agent_it) -> void
  {
    if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
    {
      output[agent_it].emplace_back(0, nullptr, secondsToMinutes(m_agents[agent_it].maxChargeTime()));
    }
    else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
    {
      output[agent_it].emplace_back(0, nullptr, secondsToMinutes(m_agents[agent_it].minChargeTime()));
    }
    else // No battery constraints
    {
      output[agent_it].emplace_back(0, nullptr, 0);
    }
    action_end_times[     agent_it] = m_agents[agent_it].cgetShiftStartTime() + minutesToSeconds(output[agent_it].back().dwell_time);
    cur_action_start_time[agent_it] = m_agents[agent_it].cgetShiftStartTime();
  });

  // Planning loop
  while(cur_time < end_time)
  {
    // Find next action end
    const uint32_t next_action_end_ind = std::distance(action_end_times.cbegin(),
                                                     std::min_element(std::execution::unseq, action_end_times.cbegin(), action_end_times.cend()));
    // Propagate to next action end
    const float time_step = action_end_times[next_action_end_ind] - cur_time;
    assert(time_step >= 0);
    if(0 != time_step)
    {
      std::for_each(std::execution::unseq, m_hotspots.begin(), m_hotspots.end(),
      [&] (plan::Hotspot& hs) -> void
      {
        if((active_dwell_vert_inds.array() == hs.cgetGraphIndex()).any())
        {
          hs.propagateAgentPresent(time_step);
        }
        else
        {
          hs.propagateNoAgentPresent(time_step);
        }
      });
    }
    cur_time += time_step;
    if(cur_path_traversed[next_action_end_ind])
    { // Finished with dwell time
      if(graph::isDepot(output[next_action_end_ind].back().vertex_ind))
      {
        if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
        {
          m_agents[next_action_end_ind].resetTripCounterInPlace();
        }
        else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
        {
          m_agents[next_action_end_ind].chargeForInPlace(minutesToSeconds(output[next_action_end_ind].back().dwell_time));
        }
      }
      else
      {
        if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
        {
          m_agents[next_action_end_ind].propagateAlongTimeInPlace(minutesToSeconds(output[next_action_end_ind].back().dwell_time));
        }
      }
      active_dwell_vert_inds[next_action_end_ind] = -1;
    }
    else // Do dwell time now
    {
      cur_path_traversed[    next_action_end_ind] = true;
      action_end_times[      next_action_end_ind] += minutesToSeconds(output[next_action_end_ind].back().dwell_time);
      m_agents[              next_action_end_ind].propagateAlongPathInPlace(*output[next_action_end_ind].back().prev_path);
      active_dwell_vert_inds[next_action_end_ind] = output[next_action_end_ind].back().vertex_ind;
    }
    // Chose new action
    if(-1 == active_dwell_vert_inds[next_action_end_ind])
    {
      // Find valid actions
      std::vector<Action> valid_actions;
      valid_actions.reserve(graph.numberPaths());

      const uint32_t cur_vertex_ind = output[next_action_end_ind].back().vertex_ind;
      if constexpr(Eigen::Dynamic != MAX_PLAN_VISITS)
      {
        const bool to_depot_path_only       = num_agent_need_path_to_depot       == (MAX_PLAN_VISITS - num_actions_in_plan);
        const bool no_away_from_depot_paths = (num_agent_need_path_to_depot + 1) == (MAX_PLAN_VISITS - num_actions_in_plan);

        if(no_away_from_depot_paths and graph::isDepot(cur_vertex_ind)) { goto CHOOSE_AN_ACTION; }

        if(to_depot_path_only)
        {
          std::for_each(graph.cgetPointToPointPaths()(cur_vertex_ind, 0).cbegin(),
                        graph.cgetPointToPointPaths()(cur_vertex_ind, 0).cend(),
          [&] (const graph::Path& to_vertex_path) -> void
          {
            // Check if path is follow able
            bool soc_constraint_good      = true;
            bool distance_constraint_good = true;
            if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              soc_constraint_good = (to_vertex_path.traversal_charge_needed <= m_agents[next_action_end_ind].cgetCurrentStateOfCharge());
            }
            if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              distance_constraint_good = (to_vertex_path.length <= m_agents[next_action_end_ind].tripLengthLeft());
            }
            if constexpr(verboseFlag(CONSTRAINT_OPTIONS))
            {
              if(soc_constraint_good and (not distance_constraint_good))
              {
                std::cout <<
                  "SOC constraint is satisfied with error bound: " <<
                  std::fabs(to_vertex_path.traversal_charge_needed - m_agents[next_action_end_ind].cgetCurrentStateOfCharge()) <<
                  " but distance constraint is violated with error bound: " <<
                  std::fabs(to_vertex_path.length - m_agents[next_action_end_ind].tripLengthLeft()) <<
                  "\n";
              }
              if((not soc_constraint_good) and distance_constraint_good)
              {
                std::cout <<
                  "Distance constraint is satisfied with error bound: " <<
                  std::fabs(to_vertex_path.length - m_agents[next_action_end_ind].tripLengthLeft()) <<
                  " but SOC constraint is violated with error bound: " <<
                  std::fabs(to_vertex_path.traversal_charge_needed - m_agents[next_action_end_ind].cgetCurrentStateOfCharge()) <<
                  "\n";
              }
            }
            if((not soc_constraint_good) or (not distance_constraint_good)) { return; }
            if((cur_time + to_vertex_path.traversal_time) > m_agents[next_action_end_ind].cgetShiftEndTime()) { return; }
            uint32_t charge_time_min = 0;
            if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              charge_time_min = secondsToMinutes(m_agents[next_action_end_ind].maxChargeTime());
            }
            else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              const float next_soc = std::min<float>(m_agents[next_action_end_ind].cgetCurrentStateOfCharge() - to_vertex_path.traversal_charge,
                                                     m_agents[next_action_end_ind].cgetMaxStateOfCharge()     - to_vertex_path.end_traversal_charge);
              charge_time_min = secondsToMinutes(m_agents[next_action_end_ind].minChargeTime(next_soc));
            }
            //if((cur_time + to_vertex_path.traversal_time + minutesToSeconds(charge_time_min)) > m_agents[next_action_end_ind].cgetShiftEndTime()) { return; }
            valid_actions.emplace_back(0, &to_vertex_path, charge_time_min);
          });
          goto CHOOSE_AN_ACTION;
        }
      }
      std::for_each(vertex_inds.begin(), vertex_inds.end(),
      [&] (const uint32_t to_vertex_ind) -> void
      {
        if(cur_vertex_ind == to_vertex_ind) { return; }
        std::for_each(graph.cgetPointToPointPaths()(cur_vertex_ind, to_vertex_ind).cbegin(),
                      graph.cgetPointToPointPaths()(cur_vertex_ind, to_vertex_ind).cend(),
        [&] (const graph::Path& to_vertex_path) -> void
        {
          // Check if path is follow able
          bool soc_constraint_good      = true;
          bool distance_constraint_good = true;
          if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
          {
            soc_constraint_good = (to_vertex_path.traversal_charge_needed <= m_agents[next_action_end_ind].cgetCurrentStateOfCharge());
          }
          if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
          {
            distance_constraint_good = (to_vertex_path.length <= m_agents[next_action_end_ind].tripLengthLeft());
          }
          if constexpr(verboseFlag(CONSTRAINT_OPTIONS))
          {
            if(soc_constraint_good and (not distance_constraint_good))
            {
              std::cout <<
                "SOC constraint is satisfied with error bound: " <<
                std::fabs(to_vertex_path.traversal_charge_needed - m_agents[next_action_end_ind].cgetCurrentStateOfCharge()) <<
                " but distance constraint is violated with error bound: " <<
                std::fabs(to_vertex_path.length - m_agents[next_action_end_ind].tripLengthLeft()) <<
                "\n";
            }
            if((not soc_constraint_good) and distance_constraint_good)
            {
              std::cout <<
                "Distance constraint is satisfied with error bound: " <<
                std::fabs(to_vertex_path.length - m_agents[next_action_end_ind].tripLengthLeft()) <<
                " but SOC constraint is violated with error bound: " <<
                std::fabs(to_vertex_path.traversal_charge_needed - m_agents[next_action_end_ind].cgetCurrentStateOfCharge()) <<
                "\n";
            }
          }
          if((not soc_constraint_good) or (not distance_constraint_good)) { return; }
          if(graph::isDepot(to_vertex_ind))
          {
            if((cur_time + to_vertex_path.traversal_time) > m_agents[next_action_end_ind].cgetShiftEndTime()) { return; }
            uint32_t charge_time_min = 0;
            if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              charge_time_min = secondsToMinutes(m_agents[next_action_end_ind].maxChargeTime());
            }
            else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              const float next_soc = std::min<float>(m_agents[next_action_end_ind].cgetCurrentStateOfCharge() - to_vertex_path.traversal_charge,
                                                      m_agents[next_action_end_ind].cgetMaxStateOfCharge()     - to_vertex_path.end_traversal_charge);
              charge_time_min = secondsToMinutes(m_agents[next_action_end_ind].minChargeTime(next_soc));
            }
            valid_actions.emplace_back(to_vertex_ind, &to_vertex_path, charge_time_min);
          }
          else // It's a hotspot
          {
            const float next_time = cur_time + to_vertex_path.traversal_time + dwell_time_sec;
            if(next_time > m_agents[next_action_end_ind].cgetShiftEndTime()) { return; }
            float next_soc;
            if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              next_soc = std::min<float>(m_agents[next_action_end_ind].cgetCurrentStateOfCharge() - to_vertex_path.traversal_charge,
                                         m_agents[next_action_end_ind].cgetMaxStateOfCharge()     - to_vertex_path.end_traversal_charge) -
                         (m_agents[next_action_end_ind].cgetAuxiliaryPower() * dwell_time_sec);
            }

            // if valid path exists to this vertex and back to depot
            if(not std::any_of(std::execution::unseq,
                               graph.cgetPointToPointPaths()(to_vertex_ind, 0).cbegin(),
                               graph.cgetPointToPointPaths()(to_vertex_ind, 0).cend(),
            [&] (const graph::Path& to_depot_path) -> bool
            {
//              float charge_time = 0;
              if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
              {
                if(to_depot_path.traversal_charge_needed > next_soc) { return false; }

//                if constexpr(not distanceConstraintsFlag(CONSTRAINT_OPTIONS))
//                {
//                  const float depot_soc = std::min<float>(next_soc - to_depot_path.traversal_charge,
//                                                          m_agents[next_action_end_ind].cgetMaxStateOfCharge() - to_depot_path.end_traversal_charge);
//                  charge_time = minutesToSeconds(secondsToMinutes(m_agents[next_action_end_ind].minChargeTime(depot_soc, agents[next_action_end_ind].cgetCurrentStateOfCharge())));
//                }
              }
              if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
              {
                if((to_vertex_path.length + to_depot_path.length) > m_agents[next_action_end_ind].tripLengthLeft()) { return false; }

//                charge_time = minutesToSeconds(secondsToMinutes(m_agents[next_action_end_ind].maxChargeTime()));
              }

//              const double time_needed = next_time + to_depot_path.traversal_time + charge_time;
              const float time_needed = next_time + to_depot_path.traversal_time;
              return not (time_needed > m_agents[next_action_end_ind].cgetShiftEndTime());
            })) { return; }

            valid_actions.emplace_back(to_vertex_ind, &to_vertex_path, dwell_time);
          }
        });
      });
      CHOOSE_AN_ACTION:
      // Choose an action
      const uint32_t num_valid_actions = valid_actions.size();
      if(0 == num_valid_actions)
      {
        assert(graph::isDepot(output[next_action_end_ind].back().vertex_ind));

        // No more valid actions possible
        action_end_times[next_action_end_ind] = std::numeric_limits<float>::infinity();
      }
      else if(1 == num_valid_actions)
      {
        output[               next_action_end_ind].emplace_back(valid_actions.front());
        action_end_times[     next_action_end_ind] += output[next_action_end_ind].back().prev_path->traversal_time;
        cur_path_traversed[   next_action_end_ind] = false;
        cur_action_start_time[next_action_end_ind] = cur_time;
        ++num_actions_in_plan;
        if constexpr(Eigen::Dynamic != MAX_PLAN_VISITS)
        {
          if(graph::isDepot(output[next_action_end_ind].back().vertex_ind))
          {
            --num_agent_need_path_to_depot;
          }
          else
          {
            if(graph::isDepot(std::prev(output[next_action_end_ind].cend(), 2)->vertex_ind)) { ++num_agent_need_path_to_depot; }
          }
        }
      }
      else
      {
        // Find cofield gradient
        Eigen::Matrix<float,2,NUM_AGENTS,EIG_OPTIONS,2,NUM_AGENTS> agent_positions;
        std::for_each(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
        [&] (const uint32_t agent_ind) -> void
        {
          if(nullptr == output[agent_ind].back().prev_path)
          {
            agent_positions.col(agent_ind) = graph.cgetPointToPointPaths()(0, 1).front().cgetSubEdges().front()->cgetFromNode()->cgetPosition().topRows<2>();
          }
          else // Not charging
          {
            agent_positions.col(agent_ind) = output[agent_ind].back().prev_path->stateAtTime(std::min<float>(cur_time - cur_action_start_time[agent_ind], output[agent_ind].back().prev_path->traversal_time)).template topRows<2>();
          }
        });
        Eigen::Matrix<float,2,NUM_AGENTS,EIG_OPTIONS,2,NUM_AGENTS> agent_sub_gradients;
        std::for_each(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
        [&agent_sub_gradients,&agent_positions,next_action_end_ind] (const uint32_t agent_ind) -> void
        {
          if(agent_ind == next_action_end_ind)
          {
            agent_sub_gradients.col(agent_ind).setZero();
          }
          else
          {
            const Eigen::Matrix<float,2,1,EIG_OPTIONS,2,1> pos_diff = agent_positions.col(next_action_end_ind) - agent_positions.col(agent_ind);
            const float                                    norm_sqr = pos_diff.array().square().sum();
            if(norm_sqr == 0)
            {
              agent_sub_gradients.col(agent_ind).setZero(); // TODO: Decide if I like this
            }
            else
            {
              agent_sub_gradients.col(agent_ind) = -pos_diff / std::pow(norm_sqr, float(2));
            }
          }
        });
        Eigen::Matrix<float,2,NUM_HOTSPOTS,EIG_OPTIONS,2,NUM_HOTSPOTS> poi_sub_gradients;
        std::for_each(std::execution::unseq, hs_inds.begin(), hs_inds.end(),
        [&] (const uint32_t poi_ind) -> void
        {
          const Eigen::Matrix<float,2,1,EIG_OPTIONS,2,1> pose_diff      = agent_positions.col(next_action_end_ind) - graph.cgetPointToPointPaths()(m_hotspots[poi_ind].cgetGraphIndex(), 0).front().cgetSubEdges().front()->cgetFromNode()->cgetPosition().template topRows<2>();
          const float                                    pose_diff_norm = pose_diff.squaredNorm();
          poi_sub_gradients.col(poi_ind) = -((m_hotspots[poi_ind].cgetExpectedEventRate() * std::exp(-pose_diff_norm / hotspot_grpf_width_parameter)) / hotspot_grpf_width_parameter) * pose_diff;
        });
        const Eigen::Matrix<float,2,1,EIG_OPTIONS,2,1> cofield_gradient = ((searching_weight       * (poi_sub_gradients.  rowwise().sum())) -
                                                                           (agent_avoidance_weight * (agent_sub_gradients.rowwise().sum()))).normalized();
        // Pick direction most inline with gradient
        const uint32_t                                                     num_valid_actions = valid_actions.size();
        const boost::integer_range<uint32_t>                               valid_action_inds(0, num_valid_actions);
        Eigen::Matrix<float,2,Eigen::Dynamic,EIG_OPTIONS,2,Eigen::Dynamic> valid_action_dirs(2, num_valid_actions);
        std::for_each(std::execution::unseq, valid_action_inds.begin(), valid_action_inds.end(),
        [&] (const uint32_t valid_action_ind) -> void
        {
          if(graph::isDepot(valid_actions[valid_action_ind].vertex_ind))
          {
            valid_action_dirs.col(valid_action_ind) = (graph.cgetPointToPointPaths()(0, 1).front().cgetSubEdges().front()->cgetFromNode()->cgetPosition().topRows<2>() - agent_positions.col(next_action_end_ind).template topRows<2>()).normalized();
          }
          else
          {
            valid_action_dirs.col(valid_action_ind) = (graph.cgetPointToPointPaths()(valid_actions[valid_action_ind].vertex_ind, 0).front().cgetSubEdges().front()->cgetFromNode()->cgetPosition().topRows<2>() - agent_positions.col(next_action_end_ind).template topRows<2>()).normalized();
          }
        });

        const uint32_t best_direction = *std::max_element(std::execution::unseq, valid_action_inds.begin(), valid_action_inds.end(),
        [&cofield_gradient,&valid_action_dirs] (const uint32_t i, const uint32_t j) -> bool
        {
          return (valid_action_dirs.col(i).transpose() * cofield_gradient) < (valid_action_dirs.col(j).transpose() * cofield_gradient);
        });
        output[                next_action_end_ind].emplace_back(valid_actions[best_direction]);
        action_end_times[      next_action_end_ind] += output[next_action_end_ind].back().prev_path->traversal_time;
        cur_path_traversed[    next_action_end_ind] = false;
        cur_action_start_time[ next_action_end_ind] = cur_time;
        ++num_actions_in_plan;
        if constexpr(Eigen::Dynamic != MAX_PLAN_VISITS)
        {
          if(graph::isDepot(output[next_action_end_ind].back().vertex_ind))
          {
            --num_agent_need_path_to_depot;
          }
          else
          {
            if(graph::isDepot(std::prev(output[next_action_end_ind].cend(), 2)->vertex_ind)) { ++num_agent_need_path_to_depot; }
          }
        }
      }
    }
  }

  // Convert to Eigen
  std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> eig_output = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(num_actions_in_plan);
  Eigen::Index                                            eig_out_ind = 0;
  for(uint32_t agent_ind = 0; agent_ind < NUM_AGENTS; ++agent_ind)
  {
    uint32_t sub_len = output[agent_ind].size();
    for(uint32_t sub_ind = 0; sub_ind < sub_len; ++sub_ind, ++eig_out_ind)
    {
      (*eig_output)[eig_out_ind] = output[agent_ind][sub_ind];
    }
  }

  return eig_output;
}

#endif
/* cofield_accent_planner.hpp */
