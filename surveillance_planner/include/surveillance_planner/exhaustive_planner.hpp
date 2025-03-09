/**
 * @File: exhaustive_planner.hpp
 * @Date: July 2024
 * @Author: James Swedeen
 *
 * @brief
 * Plans paths for agents by checking all possible paths and picking the optimal.
 **/

#ifndef SURVEILLANCE_PLANNING_EXHAUSTIVE_PLANNER_HPP
#define SURVEILLANCE_PLANNING_EXHAUSTIVE_PLANNER_HPP

/* C++ Headers */
#include<functional>
#include<limits>
#include<vector>
#include<chrono>
#include<thread>
#include<future>
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
 * @generateExhaustivePlan
 *
 * @brief
 * Planner that picks actions at random from the set of all valid actions.
 *
 * @parameters
 * graph: The graph to plan over
 * agents: The agents to plan paths for
 * start_time: The start time of the plan
 * end_time: The end time of the plan
 * cost_func: The objective function, also a boolean that determines if the plan satisfies all constraints
 * max_active_thread_count: The max number of threads to be active at one time
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
  generateExhaustivePlan(const graph::PlanningGraph&                                                           graph,
                         const AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>&                                        agents,
                         const float                                                                           start_time,
                         const float                                                                           end_time,
                         const std::function<std::pair<bool,double>(const std::vector<std::vector<Action>>&)>& cost_func,
                         const size_t                                                                          max_active_thread_count = 64);
} // plan


template<Eigen::Index            NUM_AGENTS,
         Eigen::Index            MAX_PLAN_VISITS,
         plan::ConstraintOptions CONSTRAINT_OPTIONS,
         Eigen::StorageOptions   EIG_OPTIONS>
std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::generateExhaustivePlan(const graph::PlanningGraph&                                                           graph,
                               const AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>&                                        agents,
                               const float                                                                           start_time,
                               const float                                                                           end_time,
                               const std::function<std::pair<bool,double>(const std::vector<std::vector<Action>>&)>& cost_func,
                               const size_t                                                                          max_active_thread_count)
{
  // Helper definitions
  const uint32_t                       num_paths    = graph.numberPaths();
  const uint32_t                       num_vertices = graph.numberVertices();
  const boost::integer_range<uint32_t> agent_inds( 0, NUM_AGENTS);
  const boost::integer_range<uint32_t> vertex_inds(0, num_vertices);

  // Plan
  std::vector<std::vector<Action>> best_found_sol;
  double                           best_found_cost = std::numeric_limits<double>::infinity();
  std::mutex                       best_found_mux;

  std::forward_list<std::vector<std::unique_ptr<std::tuple<std::vector<std::vector<Action>>,
                                                           AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,
                                                           Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>>> possible_plans; // Plan, agents after following plan, current plan end times

  std::list<std::tuple<std::thread,bool,std::vector<std::unique_ptr<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>>,std::unique_ptr<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>>> workers;

  // Agents start at the depot
  possible_plans.emplace_front();
  if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
  {
    std::vector<std::vector<Action>>                            output(NUM_AGENTS);
    AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>                     cur_agents(agents);
    Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> end_times;

    std::for_each(agent_inds.begin(), agent_inds.end(),
    [&output, &cur_agents, start_time, &end_times] (const uint32_t agent_ind) -> void
    {
      //const double dwell_time_sec = std::min<double>(cur_agents[agent_ind].maxChargeTime(), start_time - cur_agents[agent_ind].cgetShiftEndTime());
      const double   dwell_time_sec = cur_agents[agent_ind].maxChargeTime();
      const uint32_t dwell_time     = secondsToMinutes(dwell_time_sec);
      output[agent_ind].emplace_back(0, nullptr, dwell_time);
      cur_agents[agent_ind].resetTripCounterInPlace();
      end_times[agent_ind] = cur_agents[agent_ind].cgetShiftStartTime() + dwell_time_sec;
    });
    possible_plans.front().emplace_back(std::make_unique<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>(std::move(output), std::move(cur_agents), std::move(end_times)));
  }
  else // Check all possibilities
  {
    const uint32_t max_dwell_time = secondsToMinutes(end_time - start_time);
    Eigen::Matrix<uint32_t,Eigen::Dynamic,1> dwell_times(max_dwell_time + 1);
    std::iota(dwell_times.begin(), dwell_times.end(), 0);

    std::vector<Eigen::Index> dwell_times_inds(NUM_AGENTS, 0);
    while(true)
    {
      // Make plan
      std::vector<std::vector<Action>>                            output(NUM_AGENTS);
      AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>                     cur_agents(agents);
      Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> end_times;

      std::for_each(std::execution::par_unseq, agent_inds.begin(), agent_inds.end(),
      [&output, &cur_agents, &end_times, &dwell_times, &dwell_times_inds] (const uint32_t agent_ind) -> void
      {
        const double dwell_time_sec = minutesToSeconds(dwell_times[dwell_times_inds[agent_ind]]);
        output[agent_ind].emplace_back(0, nullptr, dwell_times[dwell_times_inds[agent_ind]]);
        cur_agents[agent_ind].chargeForInPlace(dwell_time_sec);
        end_times[agent_ind] = cur_agents[agent_ind].cgetShiftStartTime() + dwell_time_sec;
      });
      if(std::none_of(agent_inds.begin(), agent_inds.end(),
                      [&] (const uint32_t agent_ind) -> bool { return end_times[agent_ind] > cur_agents[agent_ind].cgetShiftEndTime(); }))
      {
        possible_plans.front().emplace_back(std::make_unique<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>(std::move(output), std::move(cur_agents), std::move(end_times)));
      }
      // Increment indexes
      uint32_t agent_ind = 0;
      while(true)
      {
        ++dwell_times_inds[agent_ind];
        if(dwell_times_inds[agent_ind] == dwell_times.rows())
        {
          dwell_times_inds[agent_ind] = 0;
          ++agent_ind;
          if(agent_ind == NUM_AGENTS)
          {
            break;
          }
        }
        else
        {
          break;
        }
      }
      if(agent_ind == NUM_AGENTS)
      {
        break;
      }
    }
  }
  while((not possible_plans.empty()) or (not workers.empty()))
  {
    if(not possible_plans.empty())
    {
      const size_t thread_spots_available = max_active_thread_count - workers.size();
      const auto   new_possible_plans_end = (possible_plans.front().size() <= thread_spots_available) ? possible_plans.front().begin() : std::prev(possible_plans.front().end(), thread_spots_available);

      std::for_each(new_possible_plans_end, possible_plans.front().end(),
      [&] (std::unique_ptr<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>& pos_plan_super) -> void
      {
        if constexpr(Eigen::Dynamic != MAX_PLAN_VISITS)
        {
          if(MAX_PLAN_VISITS <= std::accumulate<std::vector<std::vector<Action>>::const_iterator,uint32_t>(std::get<0>(*pos_plan_super).cbegin(), std::get<0>(*pos_plan_super).cend(), 0,
                                                                                                              [] (const uint32_t prev_val, const std::vector<Action>& sub_plan) -> uint32_t { return prev_val + sub_plan.size(); }))
          {
            return;
          }
        }

        workers.emplace_back();
        std::get<1>(workers.back()) = false;
        std::get<3>(workers.back()).reset(pos_plan_super.release());
        std::get<0>(workers.back()) = std::thread(
        [&] (const std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>* const                   pos_plan,
             bool&                                                                                                                                                                           finished_flag,
             std::vector<std::unique_ptr<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>>& output) -> void
        {
          Eigen::Matrix<std::list<std::unique_ptr<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>>,Eigen::Dynamic,Eigen::Dynamic> next_possible_plans_mat(NUM_AGENTS, num_vertices);
          // Generate next generation
          std::for_each(agent_inds.begin(), agent_inds.end(),
          [&] (const uint32_t agent_ind) -> void
          {
            const uint32_t cur_vertex_ind = std::get<0>(*pos_plan)[agent_ind].back().vertex_ind;

            std::for_each(vertex_inds.begin(), vertex_inds.end(),
            [&] (const uint32_t to_vertex_ind) -> void
            {
              std::for_each(graph.cgetPointToPointPaths()(cur_vertex_ind, to_vertex_ind).cbegin(),
                            graph.cgetPointToPointPaths()(cur_vertex_ind, to_vertex_ind).cend(),
              [&] (const graph::Path& to_vertex_path) -> void
              {
                const double path_end_time = std::get<2>(*pos_plan)[agent_ind] + to_vertex_path.traversal_time;
                // Check if path is follow able
                bool soc_constraint_good      = true;
                bool distance_constraint_good = true;
                if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
                {
                  soc_constraint_good = (to_vertex_path.traversal_charge_needed <= std::get<1>(*pos_plan)[agent_ind].cgetCurrentStateOfCharge());
                }
                if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
                {
                  distance_constraint_good = (to_vertex_path.length <= std::get<1>(*pos_plan)[agent_ind].tripLengthLeft());
                }
                if constexpr(verboseFlag(CONSTRAINT_OPTIONS))
                {
                  if(soc_constraint_good and (not distance_constraint_good))
                  {
                    std::cout <<
                      "SOC constraint is satisfied with error bound: " <<
                      std::fabs(to_vertex_path.traversal_charge_needed - std::get<1>(*pos_plan)[agent_ind].cgetCurrentStateOfCharge()) <<
                      " but distance constraint is violated with error bound: " <<
                      std::fabs(to_vertex_path.length - std::get<1>(*pos_plan)[agent_ind].tripLengthLeft()) <<
                      "\n";
                  }
                  if((not soc_constraint_good) and distance_constraint_good)
                  {
                    std::cout <<
                      "Distance constraint is satisfied with error bound: " <<
                      std::fabs(to_vertex_path.length - std::get<1>(*pos_plan)[agent_ind].tripLengthLeft()) <<
                      " but SOC constraint is violated with error bound: " <<
                      std::fabs(to_vertex_path.traversal_charge_needed - std::get<1>(*pos_plan)[agent_ind].cgetCurrentStateOfCharge()) <<
                      "\n";
                  }
                }
                if((not soc_constraint_good) or (not distance_constraint_good)) { return; }
                const double min_to_depot_time = (0 == to_vertex_ind) ? 0 : graph.minTravelTimePath(to_vertex_ind, 0).traversal_time;
                const bool   time_constraint_violated = (path_end_time + min_to_depot_time) > std::get<1>(*pos_plan)[agent_ind].cgetShiftEndTime();
                bool to_depot_charge_constraint_violated = false;
                if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
                {
                  to_depot_charge_constraint_violated = (0 != to_vertex_ind) and (graph.minChargeNecessaryPath(to_vertex_ind, 0).traversal_charge_needed > std::get<1>(*pos_plan)[agent_ind].cgetCurrentStateOfCharge());
                }
                bool to_depot_dist_constraint_violated = false;
                if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
                {
                  to_depot_dist_constraint_violated = (0 != to_vertex_ind) and (graph.minLengthPath(to_vertex_ind, 0).length > std::get<1>(*pos_plan)[agent_ind].tripLengthLeft());
                }
                if(to_depot_charge_constraint_violated or time_constraint_violated or to_depot_dist_constraint_violated) { return; }

                // Propagate along path
                std::vector<std::vector<Action>>        cur_plan(                         std::get<0>(*pos_plan));
                AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS> cur_agents(                       std::get<1>(*pos_plan));
                Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> cur_end_times(std::get<2>(*pos_plan));

                cur_agents[   agent_ind].propagateAlongPathInPlace(to_vertex_path);
                cur_end_times[agent_ind] += to_vertex_path.traversal_time;

                // Generate next step plans
                if(graph::isDepot(to_vertex_ind))
                {
                  if constexpr(distanceConstraintsFlag(CONSTRAINT_OPTIONS))
                  {
                    const uint32_t dwell_time     = secondsToMinutes(cur_agents[agent_ind].maxChargeTime());
                    const double   dwell_time_sec = minutesToSeconds(dwell_time);
                    cur_end_times[agent_ind] += dwell_time_sec;
                    if(cur_end_times[agent_ind] > cur_agents[agent_ind].cgetShiftEndTime()) { return; }
                    cur_plan[agent_ind].emplace_back(0, &to_vertex_path, dwell_time);
                    cur_agents[agent_ind].resetTripCounterInPlace();

                    next_possible_plans_mat(agent_ind, to_vertex_ind).emplace_back(std::make_unique<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>(std::move(cur_plan), std::move(cur_agents), std::move(cur_end_times)));
                  }
                  else // check all dwell times
                  {
                    const uint32_t max_dwell_time = secondsToMinutes(cur_agents[agent_ind].cgetShiftEndTime() - cur_end_times[agent_ind]);
                    Eigen::Matrix<uint32_t,Eigen::Dynamic,1> dwell_times(max_dwell_time + 1);
                    std::iota(dwell_times.begin(), dwell_times.end(), 0);

                    const uint32_t                 num_dwell_times = dwell_times.rows();
                    boost::integer_range<uint32_t> dwell_times_inds(0, num_dwell_times);

                    std::vector<std::unique_ptr<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>> temp_possible_plans(num_dwell_times);
                    std::for_each(dwell_times_inds.begin(), dwell_times_inds.end(),
                    [&] (const uint32_t dwell_times_ind) -> void
                    {
                      const double dwell_time_sec = minutesToSeconds(dwell_times[dwell_times_ind]);

                      if((cur_end_times[agent_ind] + dwell_time_sec) > cur_agents[agent_ind].cgetShiftEndTime()) { return; }

                      std::vector<std::vector<Action>>                            sub_cur_plan(     cur_plan);
                      AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>                     sub_cur_agents(   cur_agents);
                      Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> sub_cur_end_times(cur_end_times);

                      sub_cur_plan[agent_ind].emplace_back(0, &to_vertex_path, dwell_times[dwell_times_ind]);
                      sub_cur_agents[agent_ind].chargeForInPlace(dwell_time_sec);
                      sub_cur_end_times[agent_ind] += dwell_time_sec;

                      temp_possible_plans[dwell_times_ind] = std::make_unique<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>(std::move(sub_cur_plan), std::move(sub_cur_agents), std::move(sub_cur_end_times));
                    });
                    std::list<std::unique_ptr<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>> temp_possible_plans_real;
                    std::copy_if(std::make_move_iterator(temp_possible_plans.begin()),
                                 std::make_move_iterator(temp_possible_plans.end()),
                                 std::back_inserter(temp_possible_plans_real),
                                 [] (const std::unique_ptr<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>& pos_plan_ptr) -> bool
                                 { return nullptr != pos_plan_ptr.get(); });
                    next_possible_plans_mat(agent_ind, to_vertex_ind).splice(next_possible_plans_mat(agent_ind, to_vertex_ind).end(), std::move(temp_possible_plans_real));
                  }
                }
                else // Not the depot
                {
                  const uint32_t max_dwell_time = secondsToMinutes(cur_agents[agent_ind].cgetShiftEndTime() - cur_end_times[agent_ind]);
                  Eigen::Matrix<uint32_t,Eigen::Dynamic,1> dwell_times(max_dwell_time + 1);
                  std::iota(dwell_times.begin(), dwell_times.end(), 0);

                  const uint32_t                 num_dwell_times = dwell_times.rows();
                  boost::integer_range<uint32_t> dwell_times_inds(0, num_dwell_times);

                  std::vector<std::unique_ptr<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>> temp_possible_plans(num_dwell_times);
                  std::for_each(dwell_times_inds.begin(), dwell_times_inds.end(),
                  [&] (const uint32_t dwell_times_ind) -> void
                  {
                    const double dwell_time_sec = minutesToSeconds(dwell_times[dwell_times_ind]);

                    if((cur_end_times[agent_ind] + dwell_time_sec + min_to_depot_time) > cur_agents[agent_ind].cgetShiftEndTime()) { return; }

                    std::vector<std::vector<Action>>                            sub_cur_plan(     cur_plan);
                    AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>                     sub_cur_agents(   cur_agents);
                    Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1> sub_cur_end_times(cur_end_times);

                    sub_cur_plan[agent_ind].emplace_back(to_vertex_ind, &to_vertex_path, dwell_times[dwell_times_ind]);
                    if constexpr(socConstraintsFlag(CONSTRAINT_OPTIONS))
                    {
                      sub_cur_agents[agent_ind].propagateAlongTimeInPlace(dwell_time_sec);
                    }
                    sub_cur_end_times[agent_ind] += dwell_time_sec;

                    temp_possible_plans[dwell_times_ind] = std::make_unique<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>(std::move(sub_cur_plan), std::move(sub_cur_agents), std::move(sub_cur_end_times));
                  });
                  std::list<std::unique_ptr<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>> temp_possible_plans_real;
                  std::copy_if(std::make_move_iterator(temp_possible_plans.begin()),
                               std::make_move_iterator(temp_possible_plans.end()),
                               std::back_inserter(temp_possible_plans_real),
                               [] (const std::unique_ptr<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>& pos_plan_ptr) -> bool
                               { return nullptr != pos_plan_ptr.get(); });
                  next_possible_plans_mat(agent_ind, to_vertex_ind).splice(next_possible_plans_mat(agent_ind, to_vertex_ind).end(), std::move(temp_possible_plans_real));
                }
              });
            });
          });
          // Move next possible plans into correct location
          {
            uint32_t output_len = 0;
            std::for_each(agent_inds.begin(), agent_inds.end(),
            [&] (const uint32_t agent_ind) -> void
            {
              std::for_each(vertex_inds.begin(), vertex_inds.end(),
              [&] (const uint32_t to_vertex_ind) -> void
              {
                output_len += next_possible_plans_mat(agent_ind, to_vertex_ind).size();
              });
            });
            output.reserve(output_len);
            std::for_each(agent_inds.begin(), agent_inds.end(),
            [&] (const uint32_t agent_ind) -> void
            {
              std::for_each(vertex_inds.begin(), vertex_inds.end(),
              [&] (const uint32_t to_vertex_ind) -> void
              {
                std::copy(std::make_move_iterator(next_possible_plans_mat(agent_ind, to_vertex_ind).begin()),
                          std::make_move_iterator(next_possible_plans_mat(agent_ind, to_vertex_ind).end()),
                          std::back_inserter(output));
              });
            });
          }
          // Process cost function
          if(std::all_of(std::get<0>(*pos_plan).cbegin(), std::get<0>(*pos_plan).cend(),
             [&] (const std::vector<Action>& agent_plan) -> bool
             { return graph::isDepot(agent_plan.back().vertex_ind); }) and
             std::all_of(agent_inds.begin(), agent_inds.end(),
             [&] (const uint32_t agent_ind) -> bool
             {
               return std::get<1>(*pos_plan)[agent_ind].cgetCurrentStateOfCharge() > (agents[agent_ind].cgetCurrentStateOfCharge() - double(1e-8));
             }))
          {
            const std::pair<bool,double> cost = cost_func(std::get<0>(*pos_plan));
            assert(cost.first);
            {
              std::lock_guard<std::mutex> lock(best_found_mux);
              if(cost.second < best_found_cost)
              {
                best_found_sol  = std::get<0>(*pos_plan);
                best_found_cost = cost.second;
              }
            }
          }
          finished_flag = true;
        }, std::get<3>(workers.back()).get(), std::ref(std::get<1>(workers.back())), std::ref(std::get<2>(workers.back())));
      });

      // Bookkeeping
      possible_plans.front().erase(new_possible_plans_end, possible_plans.front().end());
      if(possible_plans.front().empty())
      {
        possible_plans.pop_front();
      }
    }

    // Update workers
    workers.remove_if([&possible_plans] (std::tuple<std::thread,
                                                    bool,
                                                    std::vector<std::unique_ptr<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>>,
                                                    std::unique_ptr<std::tuple<std::vector<std::vector<Action>>,AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS>,Eigen::Matrix<double,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>>>>& worker) -> bool
    {
      if(std::get<1>(worker))
      {
        std::get<0>(worker).join();
        possible_plans.emplace_front(std::move(std::get<2>(worker)));
        return true;
      }
      return false;
    });
  }

  assert(not std::isinf(best_found_cost));

  // Convert to Eigen
  const uint32_t num_actions_in_plan = std::accumulate<std::vector<std::vector<Action>>::const_iterator,uint32_t>(best_found_sol.cbegin(), best_found_sol.cend(), 0,
                                                                                                              [] (const uint32_t prev_val, const std::vector<Action>& sub_plan) -> uint32_t { return prev_val + sub_plan.size(); });
  std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> eig_output  = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(num_actions_in_plan, 1);
  Eigen::Index                                            eig_out_ind = 0;
  for(uint32_t agent_ind = 0; agent_ind < NUM_AGENTS; ++agent_ind)
  {
    uint32_t sub_len = best_found_sol[agent_ind].size();
    for(uint32_t sub_ind = 0; sub_ind < sub_len; ++sub_ind, ++eig_out_ind)
    {
      (*eig_output)[eig_out_ind] = best_found_sol[agent_ind][sub_ind];
    }
  }

  return eig_output;
}

#endif
/* exhaustive_planner.hpp */
