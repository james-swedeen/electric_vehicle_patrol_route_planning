/**
 * @File: random_planner.hpp
 * @Date: March 2024
 * @Author: James Swedeen
 *
 * @brief
 * Plans paths for agents by picking from the valid actions at each time point and picking one at random.
 **/

#ifndef SURVEILLANCE_PLANNING_RANDOM_PLANNER_HPP
#define SURVEILLANCE_PLANNING_RANDOM_PLANNER_HPP

/* C++ Headers */
#include<vector>
#include<list>
#include<chrono>
#include<functional>
#include<random>
#include<algorithm>
#include<execution>
#include<iostream>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Street Graph Headers */
#include<street_graph/planning_graph.hpp>

/* Local Headers */
#include<surveillance_planner/agent.hpp>
#include<surveillance_planner/action.hpp>
#include<surveillance_planner/helpers.hpp>

namespace plan
{
/**
 * @generateRandomPlan
 *
 * @brief
 * Planner that picks actions at random from the set of all valid actions.
 *
 * @parameters
 * graph: The graph to plan over
 * agents: The agents to plan paths for
 * dwell_time: The time to spend dwelling at hotspots in minutes
 * random_seed: Seed for random action selection
 *
 * @templates
 * NUM_AGENTS: The number of agents being planned for
 * MAX_PLAN_VISITS: The max length possible for a plan
 * CONSTRAINT_OPTIONS: Options that control what SOC constraints to use
 * EIG_OPTIONS: Eigen storage options
 *
 * @return
 * Vector with each index being an action.
 **/
template<Eigen::Index          NUM_AGENTS,
         Eigen::Index          MAX_PLAN_VISITS,
         ConstraintOptions     CONSTRAINT_OPTIONS,
         Eigen::StorageOptions EIG_OPTIONS = Eigen::ColMajor bitor Eigen::AutoAlign>
std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  generateRandomPlan(const graph::PlanningGraph&                    graph,
                     const AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>& agents,
                     const uint32_t                                 dwell_time,
                     const unsigned                                 random_seed = std::chrono::system_clock::now().time_since_epoch().count());
} // plan


template<Eigen::Index            NUM_AGENTS,
         Eigen::Index            MAX_PLAN_VISITS,
         plan::ConstraintOptions CONSTRAINT_OPTIONS,
         Eigen::StorageOptions   EIG_OPTIONS>
std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::generateRandomPlan(const graph::PlanningGraph&                    graph,
                           const AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>& agents,
                           const uint32_t                                 dwell_time,
                           const unsigned                                 random_seed)
{
  // Helper definitions
  const boost::integer_range<uint32_t> agent_inds( 0, NUM_AGENTS);
  const boost::integer_range<uint32_t> vertex_inds(0, graph.numberVertices());
  const float                          dwell_time_sec = minutesToSeconds(dwell_time);
  std::default_random_engine           rand_gen(random_seed);

  // Plan
  std::vector<std::vector<Action>> output(NUM_AGENTS);
  uint32_t                         num_actions_in_plan    = NUM_AGENTS;
  uint32_t                         num_plans_end_at_depot = NUM_AGENTS;

  AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>                     cur_agents(agents);
  Eigen::Matrix<float,NUM_AGENTS,1,EIG_OPTIONS>               cur_times = agents.unaryExpr([] (const Agent& agent) -> float { return agent.cgetShiftStartTime(); });
  Eigen::Matrix<std::vector<Action>,NUM_AGENTS,1,EIG_OPTIONS> valid_actions(NUM_AGENTS);

  // Find initial valid actions
  std::for_each(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
                [&] (const uint32_t agent_ind) -> void
  {
    if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
    {
      const float charge_time_sec = cur_agents[agent_ind].maxChargeTime();
      output[agent_ind].emplace_back(0, nullptr, secondsToMinutes(charge_time_sec));
      cur_agents[agent_ind].resetTripCounterInPlace();
      cur_times[agent_ind] += charge_time_sec;
    }
    else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
    {
      const float min_charge_time_sec = cur_agents[agent_ind].minChargeTime();
      output[agent_ind].emplace_back(0, nullptr, secondsToMinutes(min_charge_time_sec));
      cur_agents[agent_ind].chargeForInPlace(min_charge_time_sec);
      cur_times[agent_ind] += min_charge_time_sec;
    }
    else // No battery constraints
    {
      cur_agents[agent_ind].emplace_back(0, nullptr, 0);
    }

    valid_actions[agent_ind].reserve(graph.numberPaths());
    std::for_each(vertex_inds.begin(), vertex_inds.end(),
    [&] (const uint32_t to_vertex_ind) -> void
    {
      if(output[agent_ind].back().vertex_ind == to_vertex_ind) { return; }
      std::for_each(graph.cgetPointToPointPaths()(output[agent_ind].back().vertex_ind, to_vertex_ind).cbegin(),
                    graph.cgetPointToPointPaths()(output[agent_ind].back().vertex_ind, to_vertex_ind).cend(),
      [&] (const graph::Path& to_vertex_path) -> void
      {
        // Check if path is follow able
        bool soc_constraint_good      = true;
        bool distance_constraint_good = true;
        float next_dist;
        if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
        {
          soc_constraint_good = (to_vertex_path.traversal_charge_needed <= cur_agents[agent_ind].cgetCurrentStateOfCharge());
        }
        if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
        {
          next_dist = cur_agents[agent_ind].cgetCurrentTripLength() + to_vertex_path.length;
          distance_constraint_good = (next_dist <= cur_agents[agent_ind].cgetMaxTripLength());
        }
        if constexpr(verboseFlag(CONSTRAINT_OPTIONS))
        {
          if(soc_constraint_good and (not distance_constraint_good))
          {
            std::cout <<
              "SOC constraint is satisfied with error bound: " <<
              std::fabs(to_vertex_path.traversal_charge_needed - cur_agents[agent_ind].cgetCurrentStateOfCharge()) <<
              " but distance constraint is violated with error bound: " <<
              std::fabs(to_vertex_path.length - cur_agents[agent_ind].tripLengthLeft()) <<
              "\n";
          }
          if((not soc_constraint_good) and distance_constraint_good)
          {
            std::cout <<
              "Distance constraint is satisfied with error bound: " <<
              std::fabs(to_vertex_path.length - cur_agents[agent_ind].tripLengthLeft()) <<
              " but SOC constraint is violated with error bound: " <<
              std::fabs(to_vertex_path.traversal_charge_needed - cur_agents[agent_ind].cgetCurrentStateOfCharge()) <<
              "\n";
          }
        }
        if((not soc_constraint_good) or (not distance_constraint_good)) { return; }
        // Now make sure we can make it to the depot
        if(graph::isDepot(to_vertex_ind))
        {
          const float next_time = cur_times[agent_ind] + to_vertex_path.traversal_time;
          if(next_time > cur_agents[agent_ind].cgetShiftEndTime()) { return; }

          uint32_t charge_time = 0;
          if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
          {
            charge_time = secondsToMinutes(agents[agent_ind].maxChargeTime());
          }
          else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS)) // Charge until full
          {
            const float next_soc = std::min<float>(cur_agents[agent_ind].cgetCurrentStateOfCharge() - to_vertex_path.traversal_charge,
                                                   cur_agents[agent_ind].cgetMaxStateOfCharge()     - to_vertex_path.end_traversal_charge);
            charge_time = secondsToMinutes(cur_agents[agent_ind].minChargeTime(next_soc));
          }
          valid_actions[agent_ind].emplace_back(to_vertex_ind, &to_vertex_path, charge_time);
        }
        else
        {
          float next_soc;
          if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
          {
            next_soc = std::min<float>(cur_agents[agent_ind].cgetCurrentStateOfCharge() - to_vertex_path.traversal_charge,
                                       cur_agents[agent_ind].cgetMaxStateOfCharge()     - to_vertex_path.end_traversal_charge)
                       - (cur_agents[agent_ind].cgetAuxiliaryPower() * dwell_time_sec);
          }
          const float next_time = cur_times[agent_ind] + to_vertex_path.traversal_time + dwell_time_sec;
          if(next_time > cur_agents[agent_ind].cgetShiftEndTime()) { return; }

          // if valid path exists to this vertex and back to depot
          if(not std::any_of(std::execution::unseq, graph.cgetPointToPointPaths()(to_vertex_ind, 0).cbegin(),
                             graph.cgetPointToPointPaths()(to_vertex_ind, 0).cend(),
          [&] (const graph::Path& to_depot_path) -> bool
          {
            const float time_needed = next_time + to_depot_path.traversal_time;
            return not (time_needed > cur_agents[agent_ind].cgetShiftEndTime());
            float charge_time = 0;
            if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              if(to_depot_path.traversal_charge_needed > next_soc) { return false; }

              if constexpr(not distanceConstraintsFlag(CONSTRAINT_OPTIONS))
              {
                const float depot_soc = std::min<float>(next_soc - to_depot_path.traversal_charge,
                                                        cur_agents[agent_ind].cgetMaxStateOfCharge() - to_depot_path.end_traversal_charge);
                charge_time = minutesToSeconds(secondsToMinutes(cur_agents[agent_ind].minChargeTime(depot_soc, agents[agent_ind].cgetCurrentStateOfCharge())));
              }
            }
            if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              const float depot_dist = next_dist + to_depot_path.length;
              if(depot_dist > cur_agents[agent_ind].cgetMaxTripLength()) { return false; }

              charge_time = minutesToSeconds(secondsToMinutes(cur_agents[agent_ind].maxChargeTime()));
            }
          })) { return; }

          valid_actions[agent_ind].emplace_back(to_vertex_ind, &to_vertex_path, dwell_time);
        }
      });
    });
  });

  /// Pick actions loop
  bool to_depot_path_only = false;
  while(true)
  {
    // Check for ending at depot
    if constexpr(Eigen::Dynamic != MAX_PLAN_VISITS)
    {
      if(not to_depot_path_only)
      {
        const uint32_t num_agent_need_path_to_depot = NUM_AGENTS - num_plans_end_at_depot;
        const uint32_t num_actions_left             = MAX_PLAN_VISITS - num_actions_in_plan;
        if(num_agent_need_path_to_depot == num_actions_left)
        {
          // Take all non-to-depot paths out of consideration
          to_depot_path_only = true;
          std::for_each(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
                        [&] (const uint32_t agent_ind) -> void
          {
            valid_actions[agent_ind].erase(std::remove_if(std::execution::unseq, valid_actions[agent_ind].begin(), valid_actions[agent_ind].end(),
                                                          [] (const Action& action_it) -> bool { return not graph::isDepot(action_it.vertex_ind); }),
                                           valid_actions[agent_ind].end());
          });
        }
        if((num_agent_need_path_to_depot + 1) == num_actions_left)
        {
          std::for_each(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
                        [&] (const uint32_t agent_ind) -> void
          {
            if(graph::isDepot(output[agent_ind].back().vertex_ind))
            {
              valid_actions[agent_ind].clear();
            }
          });
        }
      }
    }

    // Choose action
    const uint32_t num_valid_actions = valid_actions.unaryExpr([] (const std::vector<Action>& sub_val_ac) -> uint32_t { return sub_val_ac.size(); }).sum();
    if(0 == num_valid_actions) { break; }

    std::uniform_int_distribution<uint32_t> choose_dist(0, num_valid_actions-1);
    const uint32_t                          choice = choose_dist(rand_gen);

    uint32_t      chosen_agent_ind = 0;
    const Action* chosen_action;
    bool          end_cond = false;
    for(uint32_t agent_ind = 0; (agent_ind < NUM_AGENTS) and not end_cond; ++agent_ind)
    {
      const auto sub_val_actions_end = valid_actions[agent_ind].cend();
      for(auto action_it = valid_actions[agent_ind].cbegin(); (action_it != sub_val_actions_end) and not end_cond; ++action_it)
      {
        end_cond = (chosen_agent_ind++ == choice);
        if(end_cond)
        {
          chosen_agent_ind = agent_ind;
          chosen_action    = &*action_it;
        }
      }
    }
    assert(end_cond);

    // Add action to plan
    if(graph::isDepot(output[chosen_agent_ind].back().vertex_ind)) { --num_plans_end_at_depot; }
    output[chosen_agent_ind].emplace_back(*chosen_action);
    ++num_actions_in_plan;

    // Propagate state
    if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS) or distanceConstraintsFlag(CONSTRAINT_OPTIONS))
    {
      cur_agents[chosen_agent_ind].propagateAlongPathInPlace(*chosen_action->prev_path);
    }
    {
      const double chosen_dwell_time_sec = minutesToSeconds(chosen_action->dwell_time);
      if(graph::isDepot(chosen_action->vertex_ind))
      {
        ++num_plans_end_at_depot;

        if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
        {
          cur_agents[chosen_agent_ind].resetTripCounterInPlace();
        }
        else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS)) // Do normal charge
        {
          cur_agents[chosen_agent_ind].chargeForInPlace(chosen_dwell_time_sec);
        }
      }
      else
      {
        if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
        {
          cur_agents[chosen_agent_ind].propagateAlongTimeInPlace(chosen_dwell_time_sec);
        }
      }
      cur_times[chosen_agent_ind] += chosen_action->prev_path->traversal_time + chosen_dwell_time_sec;
    }

    // Remake agent's valid actions
    valid_actions[chosen_agent_ind].clear();
    if(not to_depot_path_only)
    {
      std::for_each(vertex_inds.begin(), vertex_inds.end(),
      [&] (const uint32_t to_vertex_ind) -> void
      {
        if(output[chosen_agent_ind].back().vertex_ind == to_vertex_ind) { return; }
        std::for_each(graph.cgetPointToPointPaths()(output[chosen_agent_ind].back().vertex_ind, to_vertex_ind).cbegin(),
                      graph.cgetPointToPointPaths()(output[chosen_agent_ind].back().vertex_ind, to_vertex_ind).cend(),
        [&] (const graph::Path& to_vertex_path) -> void
        {
          // Check if path is follow able
          bool soc_constraint_good      = true;
          bool distance_constraint_good = true;
          float next_dist;
          if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
          {
            soc_constraint_good = (to_vertex_path.traversal_charge_needed <= cur_agents[chosen_agent_ind].cgetCurrentStateOfCharge());
          }
          if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
          {
            next_dist = cur_agents[chosen_agent_ind].cgetCurrentTripLength() + to_vertex_path.length;
            distance_constraint_good = (next_dist <= cur_agents[chosen_agent_ind].cgetMaxTripLength());
          }
          if constexpr(verboseFlag(CONSTRAINT_OPTIONS))
          {
            if(soc_constraint_good and (not distance_constraint_good))
            {
              std::cout <<
                "SOC constraint is satisfied with error bound: " <<
                std::fabs(to_vertex_path.traversal_charge_needed - cur_agents[chosen_agent_ind].cgetCurrentStateOfCharge()) <<
                " but distance constraint is violated with error bound: " <<
                std::fabs(to_vertex_path.length - cur_agents[chosen_agent_ind].tripLengthLeft()) <<
                "\n";
            }
            if((not soc_constraint_good) and distance_constraint_good)
            {
              std::cout <<
                "Distance constraint is satisfied with error bound: " <<
                std::fabs(to_vertex_path.length - cur_agents[chosen_agent_ind].tripLengthLeft()) <<
                " but SOC constraint is violated with error bound: " <<
                std::fabs(to_vertex_path.traversal_charge_needed - cur_agents[chosen_agent_ind].cgetCurrentStateOfCharge()) <<
                "\n";
            }
          }
          if((not soc_constraint_good) or (not distance_constraint_good)) { return; }
          // Now make sure we can make it to the depot
          if(graph::isDepot(to_vertex_ind))
          {
            const float next_time = cur_times[chosen_agent_ind] + to_vertex_path.traversal_time;
            if(next_time > cur_agents[chosen_agent_ind].cgetShiftEndTime()) { return; }

	          uint32_t charge_time = 0;
            if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              charge_time = secondsToMinutes(agents[chosen_agent_ind].maxChargeTime());
            }
            else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS)) // Change until full
            {
              const float next_soc = std::min<float>(cur_agents[chosen_agent_ind].cgetCurrentStateOfCharge() - to_vertex_path.traversal_charge,
                                                     cur_agents[chosen_agent_ind].cgetMaxStateOfCharge()     - to_vertex_path.end_traversal_charge);
              charge_time = secondsToMinutes(cur_agents[chosen_agent_ind].minChargeTime(next_soc));
            }
            valid_actions[chosen_agent_ind].emplace_back(to_vertex_ind, &to_vertex_path, charge_time);
          }
          else
          {
            float next_soc;
            if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              next_soc = std::min<float>(cur_agents[chosen_agent_ind].cgetCurrentStateOfCharge() - to_vertex_path.traversal_charge,
                                         cur_agents[chosen_agent_ind].cgetMaxStateOfCharge()     - to_vertex_path.end_traversal_charge)
                         - (cur_agents[chosen_agent_ind].cgetAuxiliaryPower() * dwell_time_sec);
            }
            const float next_time = cur_times[chosen_agent_ind] + to_vertex_path.traversal_time + dwell_time_sec;
            if(next_time > cur_agents[chosen_agent_ind].cgetShiftEndTime()) { return; }

            // if valid path exists to this vertex and back to depot
            if(not std::any_of(std::execution::unseq,
			       graph.cgetPointToPointPaths()(to_vertex_ind, 0).cbegin(),
                               graph.cgetPointToPointPaths()(to_vertex_ind, 0).cend(),
            [&] (const graph::Path& to_depot_path) -> bool
            {
              const float time_needed = next_time + to_depot_path.traversal_time;
              return not (time_needed > cur_agents[chosen_agent_ind].cgetShiftEndTime());
            })) { return; }

            valid_actions[chosen_agent_ind].emplace_back(to_vertex_ind, &to_vertex_path, dwell_time);
          }
        });
      });
    }
  }

  // Convert to Eigen
  std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> eig_output  = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(num_actions_in_plan);
  Eigen::Index                                            eig_out_ind = 0;
  for(uint32_t agent_ind = 0; agent_ind < NUM_AGENTS; ++agent_ind)
  {
    uint32_t sub_len = output[agent_ind].size();
    assert(nullptr == output[agent_ind].front().prev_path);
    for(uint32_t sub_ind = 0; sub_ind < sub_len; ++sub_ind, ++eig_out_ind)
    {
      (*eig_output)[eig_out_ind] = output[agent_ind][sub_ind];
    }
  }

  return eig_output;
}

#endif
/* random_planner.hpp */
