/**
 * @File: greedy_planner.hpp
 * @Date: May 2024
 * @Author: James Swedeen
 *
 * @brief
 * Plans paths for agents by picking from the valid actions at each time point and picking the optimal one.
 **/

#ifndef SURVEILLANCE_PLANNING_GREEDY_PLANNER_HPP
#define SURVEILLANCE_PLANNING_GREEDY_PLANNER_HPP

/* C++ Headers */
#include<functional>
#include<vector>
#include<list>
#include<chrono>
#include<algorithm>
#include<execution>
#include<iostream> // TODO

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
 * @generateGreedyPlan
 *
 * @brief
 * Planner that picks actions at random from the set of all valid actions.
 *
 * @parameters
 * graph: The graph to plan over
 * agents: The agents to plan paths for
 * dwell_time: The time to spend dwelling at hotspots
 * cost_func: The objective function, also a boolean that determines if the plan satisfies all constraints
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
  generateGreedyPlan(const graph::PlanningGraph&                                                                                   graph,
                     const AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>&                                                                agents,
                     const uint32_t                                                                                                dwell_time,
                     const std::function<std::pair<bool,double>(const Eigen::Ref<const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>&)>& cost_func);
} // plan


template<Eigen::Index            NUM_AGENTS,
         Eigen::Index            MAX_PLAN_VISITS,
         plan::ConstraintOptions CONSTRAINT_OPTIONS,
         Eigen::StorageOptions   EIG_OPTIONS>
std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::generateGreedyPlan(const graph::PlanningGraph&                                                                                   graph,
                           const AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>&                                                                agents,
                           const uint32_t                                                                                                dwell_time,
                           const std::function<std::pair<bool,double>(const Eigen::Ref<const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>&)>& cost_func)
{
  // Helper definitions
  const boost::integer_range<uint32_t> agent_inds( 0, NUM_AGENTS);
  const boost::integer_range<uint32_t> vertex_inds(0, graph.numberVertices());
  const uint32_t                       reserve_size   = NUM_AGENTS * graph.numberPaths();
  const float                          dwell_time_sec = minutesToSeconds(dwell_time);

  // Plan
  std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> output = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(NUM_AGENTS);
  AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>                 cur_agents(agents);
  Eigen::Index                                            next_must_be_agent           = -1;
  Eigen::Index                                            num_actions_left             = MAX_PLAN_VISITS - NUM_AGENTS;
  Eigen::Index                                            num_agent_need_path_to_depot = 0;
  Eigen::Matrix<Eigen::Index,NUM_AGENTS,1,EIG_OPTIONS>    agent_plan_last_inds;
  std::iota(agent_plan_last_inds.begin(), agent_plan_last_inds.end(), 0);

  // Agents start at the depot
  if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
  {
    std::for_each(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
    [&output, &cur_agents] (const uint32_t agent_ind) -> void
    {
      (*output)[agent_ind] = Action(0, nullptr, secondsToMinutes(cur_agents[agent_ind].maxChargeTime()));
      cur_agents[agent_ind].resetTripCounterInPlace();
    });
  }
  else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS)) // Just go till max charge
  {
    std::for_each(std::execution::unseq, agent_inds.begin(), agent_inds.end(),
    [&output, &cur_agents] (const uint32_t agent_ind) -> void
    {
      const double dwell_time = secondsToMinutes(cur_agents[agent_ind].minChargeTime());
      (*output)[agent_ind] = Action(0, nullptr, dwell_time);
      cur_agents[agent_ind].chargeForInPlace(minutesToSeconds(dwell_time));
    });
  }

  while(true)
  {
    // Generate all one step valid actions: the action to add, the agent ind that gets the action, cost of resulting plan
    std::vector<std::tuple<Action,uint32_t,double>> valid_steps;
    std::mutex                                      steps_mux;
    valid_steps.reserve(reserve_size);
    const auto one_agent_func = [&] (const uint32_t agent_ind) -> void
    {
      const uint32_t cur_vertex_ind = (*output)[agent_plan_last_inds[agent_ind]].vertex_ind;

      if constexpr(Eigen::Dynamic != MAX_PLAN_VISITS)
      {
        const bool to_depot_path_only       = num_agent_need_path_to_depot       == num_actions_left;
        const bool no_away_from_depot_paths = (num_agent_need_path_to_depot + 1) == num_actions_left;

        if(no_away_from_depot_paths and graph::isDepot(cur_vertex_ind)) { return; }

        if(to_depot_path_only)
        {
          std::for_each(std::execution::par_unseq,
                        graph.cgetPointToPointPaths()(cur_vertex_ind, 0).cbegin(),
                        graph.cgetPointToPointPaths()(cur_vertex_ind, 0).cend(),
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

            // If to depot then valid action made
            const Eigen::Index                                      num_actions_in_plan = output->rows();
            std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> next_step_plan(std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(num_actions_in_plan + 1));
            Action                                                  action_to_add;

            if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              action_to_add = Action(0, &to_vertex_path, secondsToMinutes(agents[agent_ind].maxChargeTime()));
            }
            else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS)) // Charge until full
            {
              const float next_soc = std::min<float>(cur_agents[agent_ind].cgetCurrentStateOfCharge() - to_vertex_path.traversal_charge,
                                                     cur_agents[agent_ind].cgetMaxStateOfCharge()     - to_vertex_path.end_traversal_charge);
              action_to_add = Action(0, &to_vertex_path, secondsToMinutes(agents[agent_ind].minChargeTime(next_soc)));
            }
            else
            {
              action_to_add = Action(0, &to_vertex_path, 0);
            }
            next_step_plan->topRows(agent_plan_last_inds[agent_ind] + 1) = output->topRows(agent_plan_last_inds[agent_ind] + 1);
            (*next_step_plan)[agent_plan_last_inds[agent_ind] + 1] = action_to_add;
            next_step_plan->bottomRows(num_actions_in_plan - agent_plan_last_inds[agent_ind] - 1) = output->bottomRows(num_actions_in_plan - agent_plan_last_inds[agent_ind] - 1);

            const std::pair<bool,double> next_step_plan_cost = cost_func(*next_step_plan);
            if(next_step_plan_cost.first)
            {
              std::lock_guard<std::mutex> lock(steps_mux);
              valid_steps.emplace_back(std::move(action_to_add), agent_ind, next_step_plan_cost.second);
            }
          });
          return;
        }
      }

      std::for_each(std::execution::par_unseq, vertex_inds.begin(), vertex_inds.end(),
      [&] (const uint32_t to_vertex_ind) -> void
      {
        if(cur_vertex_ind == to_vertex_ind) { return; }
        std::for_each(std::execution::par_unseq,
                      graph.cgetPointToPointPaths()(cur_vertex_ind, to_vertex_ind).cbegin(),
                      graph.cgetPointToPointPaths()(cur_vertex_ind, to_vertex_ind).cend(),
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

          // If to depot then valid action made
          if(graph::isDepot(to_vertex_ind))
          {
            const Eigen::Index                                      num_actions_in_plan = output->rows();
            std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> next_step_plan(std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(num_actions_in_plan + 1));
            Action                                                  action_to_add;

            if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              action_to_add = Action(to_vertex_ind, &to_vertex_path, secondsToMinutes(agents[agent_ind].maxChargeTime()));
            }
            else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS)) // Charge until full
            {
              const double next_soc = std::min<double>(cur_agents[agent_ind].cgetCurrentStateOfCharge() - to_vertex_path.traversal_charge,
                                                       cur_agents[agent_ind].cgetMaxStateOfCharge()     - to_vertex_path.end_traversal_charge);

              action_to_add = Action(to_vertex_ind, &to_vertex_path, secondsToMinutes(agents[agent_ind].minChargeTime(next_soc)));
            }
            else
            {
              action_to_add = Action(to_vertex_ind, &to_vertex_path, 0);
            }
            next_step_plan->topRows(agent_plan_last_inds[agent_ind] + 1) = output->topRows(agent_plan_last_inds[agent_ind] + 1);
            (*next_step_plan)[agent_plan_last_inds[agent_ind] + 1] = action_to_add;
            next_step_plan->bottomRows(num_actions_in_plan - agent_plan_last_inds[agent_ind] - 1) = output->bottomRows(num_actions_in_plan - agent_plan_last_inds[agent_ind] - 1);

            const std::pair<bool,double> next_step_plan_cost = cost_func(*next_step_plan);
            if(next_step_plan_cost.first)
            {
              std::lock_guard<std::mutex> lock(steps_mux);
              valid_steps.emplace_back(std::move(action_to_add), agent_ind, next_step_plan_cost.second);
            }
          }
          else // Need path to depot too
          {
            float next_soc;
            if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
            {
              next_soc = std::min<float>(cur_agents[agent_ind].cgetCurrentStateOfCharge() - to_vertex_path.traversal_charge,
                                         cur_agents[agent_ind].cgetMaxStateOfCharge()     - to_vertex_path.end_traversal_charge)
                         - (cur_agents[agent_ind].cgetAuxiliaryPower() * dwell_time_sec);
            }
            // If valid path exists to this vertex and back to depot
            std::for_each(std::execution::unseq, graph.cgetPointToPointPaths()(to_vertex_ind, 0).cbegin(), graph.cgetPointToPointPaths()(to_vertex_ind, 0).cend(),
            [&] (const graph::Path& to_depot_path) -> void
            {
              uint32_t depot_time = 0;
              if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
              {
                if(to_depot_path.traversal_charge_needed > next_soc) { return; }

                if constexpr(not distanceConstraintsFlag(CONSTRAINT_OPTIONS))
                {
                  const float depot_soc = std::min<float>(next_soc - to_depot_path.traversal_charge,
                                                          cur_agents[agent_ind].cgetMaxStateOfCharge() - to_depot_path.end_traversal_charge);
                  depot_time = secondsToMinutes(cur_agents[agent_ind].minChargeTime(depot_soc, agents[agent_ind].cgetCurrentStateOfCharge()));
                }
              }
              if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
              {
                const float depot_dist = next_dist + to_depot_path.length;
                if(depot_dist > cur_agents[agent_ind].cgetMaxTripLength()) { return; }

                depot_time = secondsToMinutes(cur_agents[agent_ind].maxChargeTime());
              }

              const Eigen::Index                                      num_actions_in_plan = output->rows();
              std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> next_step_plan(std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(num_actions_in_plan + 2));
              Action                                                  action_to_add(to_vertex_ind, &to_vertex_path, dwell_time);

              next_step_plan->topRows(agent_plan_last_inds[agent_ind] + 1) = output->topRows(agent_plan_last_inds[agent_ind] + 1);
              (*next_step_plan)[agent_plan_last_inds[agent_ind] + 1] = action_to_add;
              (*next_step_plan)[agent_plan_last_inds[agent_ind] + 2] = Action(0, &to_depot_path, depot_time);
              next_step_plan->bottomRows(num_actions_in_plan - agent_plan_last_inds[agent_ind] - 1) = output->bottomRows(num_actions_in_plan - agent_plan_last_inds[agent_ind] - 1);

              const std::pair<bool,double> next_step_plan_cost = cost_func(*next_step_plan);
              if(next_step_plan_cost.first)
              {
                std::lock_guard<std::mutex> lock(steps_mux);
                valid_steps.emplace_back(std::move(action_to_add), agent_ind, next_step_plan_cost.second);
              }
            });
          }
        });
      });
    };
    if(-1 == next_must_be_agent)
    {
      std::for_each(std::execution::par_unseq, agent_inds.begin(), agent_inds.end(), one_agent_func);
    }
    else
    {
      one_agent_func(next_must_be_agent);
    }
    // Choose action
    const auto min_it = std::min_element(std::execution::unseq, valid_steps.cbegin(), valid_steps.cend(),
                        [] (const std::tuple<Action,uint32_t,double>& i,
                            const std::tuple<Action,uint32_t,double>& j) -> bool
                        {
                          if(std::get<2>(i) == std::get<2>(j))
                          {
                            if(std::get<1>(i) == std::get<1>(j))
                            {
                              if(std::get<0>(i).vertex_ind == std::get<0>(j).vertex_ind)
                              {
                                return std::get<0>(i).prev_path < std::get<0>(j).prev_path;
                              }
                              return std::get<0>(i).vertex_ind < std::get<0>(j).vertex_ind;
                            }
                            return std::get<1>(i) < std::get<1>(j);
                          }
                          return std::get<2>(i) < std::get<2>(j);
                        });
    if(valid_steps.cend() == min_it) { break; }
    // Propagate state
    --num_actions_left;
    cur_agents[std::get<1>(*min_it)].propagateAlongPathInPlace(*std::get<0>(*min_it).prev_path);
    if(graph::isDepot(std::get<0>(*min_it).vertex_ind))
    {
      if constexpr(Eigen::Dynamic != MAX_PLAN_VISITS)
      {
        --num_agent_need_path_to_depot;
      }
      next_must_be_agent = -1;
      if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
      {
        cur_agents[std::get<1>(*min_it)].resetTripCounterInPlace();
      }
      else if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS)) // Do normal charge
      {
        cur_agents[std::get<1>(*min_it)].chargeForInPlace(minutesToSeconds(std::get<0>(*min_it).dwell_time));
      }
    }
    else
    {
      if constexpr(Eigen::Dynamic != MAX_PLAN_VISITS)
      {
        if(graph::isDepot((*output)[agent_plan_last_inds[std::get<1>(*min_it)]].vertex_ind)) { ++num_agent_need_path_to_depot; }
      }
      next_must_be_agent = std::get<1>(*min_it);
      if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
      {
        cur_agents[std::get<1>(*min_it)].propagateAlongTimeInPlace(minutesToSeconds(std::get<0>(*min_it).dwell_time));
      }
    }
    agent_plan_last_inds.bottomRows(NUM_AGENTS - std::get<1>(*min_it)).array() += 1;

    // Add action to plan
    {
      const Eigen::Index num_actions_in_plan = output->rows();
      output->conservativeResize(num_actions_in_plan + 1);
      for(uint32_t rev_out_ind = num_actions_in_plan; rev_out_ind > agent_plan_last_inds[std::get<1>(*min_it)]; --rev_out_ind)
      {
        (*output)[rev_out_ind] = std::move((*output)[rev_out_ind - 1]);
      }
      (*output)[agent_plan_last_inds[std::get<1>(*min_it)]] = std::get<0>(*min_it);
    }

    #ifndef NDEBUG
      uint32_t ind = 0;
      assert((*output)[0].dwell_time >= 0);
      uint32_t output_starts = 0;
      for(auto output_it = output->cbegin(); output_it != output->cend(); ++output_it)
      {
        if(++ind > output->rows()) { break; }
        assert((*output_it).dwell_time >= 0);
        if(nullptr != (*output_it).prev_path)
        {
          assert((*std::prev(output_it)).vertex_ind == (*output_it).prev_path->from_vertex);
          assert((*output_it).vertex_ind            == (*output_it).prev_path->to_vertex);
        }
        else
        {
          ++output_starts;
        }
      }
      assert(NUM_AGENTS == output_starts);
    #endif
  }

  return output;
}

#endif
/* greedy_planner.hpp */
