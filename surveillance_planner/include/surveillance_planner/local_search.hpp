/**
 * @File: local_search.hpp
 * @Date: November 2024
 * @Author: James Swedeen
 *
 * @brief
 * Implements the variable neighborhood decent algorithm.
 **/

#ifndef SURVEILLANCE_PLANNING_LOCAL_SEARCH_HPP
#define SURVEILLANCE_PLANNING_LOCAL_SEARCH_HPP

/* C++ Headers */
#include<chrono>
#include<vector>
#include<memory>
#include<execution>
#include<algorithm>
#include<mutex>
#include<iostream> // TODO

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Street Graph Headers */
#include<street_graph/planning_graph.hpp>

/* Local Headers */
#include<surveillance_planner/action.hpp>
#include<surveillance_planner/helpers.hpp>

namespace plan
{
/**
 * @localSearch
 *
 * @brief
 * Performs a local search on the provided plan.
 *
 * @parameters
 * cur_plan: The plan to mutate
 * cur_cost: The cost of the current plan
 * max_dwell_time_hotspot: The max dwell time allowed at a hotspot
 * max_dwell_time_depot: The max dwell time allowed at the depot
 * graph: The graph to plan over
 * cost_func: The objective function, also a boolean that determines if the plan satisfies all constraints
 * max_run_time: The maximum length of time this function will run in nanoseconds 
 *
 * @templates
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 *
 * @return
 * first: The resulting plan of the local search
 * second: The cost of the resulting plan
 **/
template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
inline std::pair<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,double>
  localSearch(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&                                                                 cur_plan,
              const uint32_t                                                                                                max_dwell_time_hotspot,
              const uint32_t                                                                                                max_dwell_time_depot,
              const graph::PlanningGraph&                                                                                   graph,
              const std::function<std::pair<bool,double>(const Eigen::Ref<const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>&)>& cost_func,
	      const std::chrono::duration<int64_t,std::nano>                                                                max_run_time = std::chrono::duration<int64_t,std::nano>::max()) noexcept;
template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
inline std::pair<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,double>
  localSearch(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&                                                                 cur_plan,
              const double                                                                                                  cur_cost,
              const uint32_t                                                                                                max_dwell_time_hotspot,
              const uint32_t                                                                                                max_dwell_time_depot,
              const graph::PlanningGraph&                                                                                   graph,
              const std::function<std::pair<bool,double>(const Eigen::Ref<const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>&)>& cost_func,
	      const std::chrono::duration<int64_t,std::nano>                                                                max_run_time = std::chrono::duration<int64_t,std::nano>::max()) noexcept;
} // plan


template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
inline std::pair<std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,double>
  plan::localSearch(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&                                                                 cur_plan,
                    const uint32_t                                                                                                max_dwell_time_hotspot,
                    const uint32_t                                                                                                max_dwell_time_depot,
                    const graph::PlanningGraph&                                                                                   graph,
                    const std::function<std::pair<bool,double>(const Eigen::Ref<const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>&)>& cost_func,
	      	    const std::chrono::duration<int64_t,std::nano>                                                                max_run_time) noexcept
{
  const std::pair<bool,double> cost = cost_func(cur_plan);
  assert(cost.first);
  return localSearch<MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan,
                                                  cost.second,
                                                  max_dwell_time_hotspot,
                                                  max_dwell_time_depot,
                                                  graph,
                                                  cost_func,
						  max_run_time);
}

template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
inline std::pair<std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,double>
  plan::localSearch(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&                                                                 cur_plan,
                    const double                                                                                                  cur_cost,
                    const uint32_t                                                                                                max_dwell_time_hotspot,
                    const uint32_t                                                                                                max_dwell_time_depot,
                    const graph::PlanningGraph&                                                                                   graph,
                    const std::function<std::pair<bool,double>(const Eigen::Ref<const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>&)>& cost_func,
	      	    const std::chrono::duration<int64_t,std::nano>                                                                max_run_time) noexcept
{
  const std::chrono::high_resolution_clock::time_point    start_time   = std::chrono::high_resolution_clock::now();
  const uint32_t                                          graph_size   = graph.numberVertices();
  std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> new_plan     = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(cur_plan);
  double                                                  new_cost     = cur_cost;
  uint32_t                                                new_plan_len = cur_plan.size();
  auto                                                    new_plan_end = cur_plan.cend();
        boost::integer_range<uint32_t>                    new_plan_inds(    0, new_plan_len);
  const boost::integer_range<uint32_t>                    graph_vertex_inds(0, graph_size);

  while(max_run_time >= (std::chrono::high_resolution_clock::now() - start_time))
  {
    Eigen::Matrix<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,Eigen::Dynamic,1,EIG_OPTIONS> next_plans;

    /// Switch time moves
    {
      const uint32_t                       temp_len = new_plan_len*2;
      const boost::integer_range<uint32_t> temp_inds(0, temp_len);

      next_plans.resize(temp_len);
      std::for_each(std::execution::unseq, new_plan_inds.begin(), new_plan_inds.end(),
      [&] (const uint32_t plan_ind) -> void
      {
        const Eigen::Index next_plans_ind = plan_ind * 2;

        // One step more time
        next_plans[next_plans_ind] = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(*new_plan);
        (*next_plans[next_plans_ind])[plan_ind].dwell_time = (*new_plan)[plan_ind].dwell_time + 1;
        // One step less time
        if((*new_plan)[plan_ind].dwell_time > 0)
        {
          next_plans[next_plans_ind+1] = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(*new_plan);
          (*next_plans[next_plans_ind+1])[plan_ind].dwell_time = (*new_plan)[plan_ind].dwell_time - 1;
        }
      });
      /// Check to be finished
      {
        Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> costs(temp_len);
        std::for_each(std::execution::par_unseq, temp_inds.begin(), temp_inds.end(),
        [&] (const uint32_t ind) -> void
        {
          if(nullptr != next_plans[ind].get())
          {
            costs[ind] = cost_func(*next_plans[ind]).second;
          }
          else
          {
            costs[ind] = std::numeric_limits<double>::infinity();
          }
        });
        Eigen::Index best_ind;
        const double best_next_cost = costs.template minCoeff<Eigen::NaNPropagationOptions::PropagateFast>(&best_ind);
        if(best_next_cost < new_cost)
        {
          // Update info
          new_plan     = std::move(next_plans[best_ind]);
          new_cost     = best_next_cost;
          new_plan_end = new_plan->cend();
          continue;
        }
      }
    }

    /// Remove visit moves
    {
      next_plans.resize(new_plan_len);
      std::for_each(std::execution::unseq, new_plan_inds.begin(), new_plan_inds.end(),
      [&] (const uint32_t to_remove_ind) -> void
      {
        if((nullptr != (*new_plan)[to_remove_ind].prev_path) and
           ((new_plan_len == (to_remove_ind + 1)) or
            (nullptr      == (*new_plan)[to_remove_ind + 1].prev_path) or
            ((*new_plan)[to_remove_ind - 1].vertex_ind != (*new_plan)[to_remove_ind + 1].vertex_ind)))
        {
          if((new_plan_len                           != (to_remove_ind + 1)) and
             (nullptr                                != (*new_plan)[to_remove_ind + 1].prev_path) and
             ((*new_plan)[to_remove_ind - 1].vertex_ind != (*new_plan)[to_remove_ind + 1].prev_path->from_vertex))
          {
            // Copy all but the removed one
            next_plans[to_remove_ind] = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(new_plan_len - 1);
            next_plans[to_remove_ind]->topRows(to_remove_ind) = new_plan->topRows(to_remove_ind);
            next_plans[to_remove_ind]->bottomRows(new_plan_len - to_remove_ind - 2) = new_plan->bottomRows(new_plan_len - to_remove_ind - 2);

            (*next_plans[to_remove_ind])[to_remove_ind] = Action((*new_plan)[to_remove_ind + 1].vertex_ind,
                                                                 &graph.minTravelTimePath((*new_plan)[to_remove_ind - 1].vertex_ind, (*new_plan)[to_remove_ind + 1].vertex_ind),
                                                                 (*new_plan)[to_remove_ind + 1].dwell_time);
          }
          else
          {
            // Copy all but the removed one
            next_plans[to_remove_ind] = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(new_plan_len - 1);
            (*next_plans[to_remove_ind]).topRows(to_remove_ind) = new_plan->topRows(to_remove_ind);
            (*next_plans[to_remove_ind]).bottomRows(new_plan_len - to_remove_ind - 1) = new_plan->bottomRows(new_plan_len - to_remove_ind - 1);
          }
        }
      });
      /// Check to be finished
      {
        Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> costs(new_plan_len);
        std::for_each(std::execution::par_unseq, new_plan_inds.begin(), new_plan_inds.end(),
        [&] (const uint32_t ind) -> void
        {
          if(nullptr != next_plans[ind].get())
          {
            costs[ind] = cost_func(*next_plans[ind]).second;
          }
          else
          {
            costs[ind] = std::numeric_limits<double>::infinity();
          }
        });

        Eigen::Index best_ind;
        const double best_next_cost = costs.template minCoeff<Eigen::NaNPropagationOptions::PropagateFast>(&best_ind);
        if(best_next_cost < new_cost)
        {
          // Update info
          new_plan      = std::move(next_plans[best_ind]);
          new_cost      = best_next_cost;
          new_plan_len  = new_plan->size();
          new_plan_end  = new_plan->cend();
          new_plan_inds = boost::integer_range<uint32_t>(0, new_plan_len);
          continue;
        }
      }
    }

    /// Add visit moves
    bool do_adds = true;
    if constexpr(Eigen::Dynamic != MAX_PLAN_VISITS)
    {
      do_adds = new_plan_len < MAX_PLAN_VISITS;
    }
    if(do_adds)
    {
      const uint32_t                       temp_len = graph_size * (new_plan_len - 2);
      const boost::integer_range<uint32_t> temp_inds(0, temp_len);

      next_plans.resize(temp_len);
      const uint32_t temp_plan_size = new_plan_len + 1;

      std::for_each(std::execution::unseq, std::next(new_plan_inds.begin()), std::prev(new_plan_inds.end()),
      [&] (const uint32_t insert_ind) -> void
      {
        // Check feasibility
        if((nullptr == (*new_plan)[insert_ind].    prev_path) or // if start of agent plan
           (nullptr == (*new_plan)[insert_ind + 1].prev_path)) // if end of agent plan
        { return; }

        const uint32_t start_ind_for_insert_ind = (insert_ind - 1) * graph_size;

        std::for_each(std::execution::unseq, graph_vertex_inds.begin(), graph_vertex_inds.end(),
        [&] (const uint32_t vertex_ind) -> void
        {
          // Check feasibility
          if(((*new_plan)[insert_ind].vertex_ind == vertex_ind) or // If doubling vertex visit
             ((*new_plan)[insert_ind - 1].vertex_ind == vertex_ind))
          { return; }

          next_plans[start_ind_for_insert_ind + vertex_ind] = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(temp_plan_size);
          next_plans[start_ind_for_insert_ind + vertex_ind]->topRows(insert_ind) = new_plan->topRows(insert_ind);
          next_plans[start_ind_for_insert_ind + vertex_ind]->bottomRows(new_plan_len - insert_ind) = new_plan->bottomRows(new_plan_len - insert_ind);

          const uint32_t next_plans_ind = start_ind_for_insert_ind + vertex_ind;

          // Insert the action
          if(graph::isDepot((*new_plan)[insert_ind].vertex_ind))
          {
            (*next_plans[next_plans_ind])[insert_ind] = Action(vertex_ind, &graph.minTravelTimePath((*new_plan)[insert_ind - 1].vertex_ind, vertex_ind), max_dwell_time_depot);
          }
          else // It's a hotspot visit
          {
            (*next_plans[next_plans_ind])[insert_ind] = Action(vertex_ind, &graph.minTravelTimePath((*new_plan)[insert_ind - 1].vertex_ind, vertex_ind), max_dwell_time_hotspot);
          }
          assert((*next_plans[next_plans_ind])[insert_ind - 1].vertex_ind == (*next_plans[next_plans_ind])[insert_ind].prev_path->from_vertex);
          // Update next action to have a path that starts in the right place
          if((temp_plan_size != (insert_ind + 1)) and
             (nullptr        != (*next_plans[next_plans_ind])[insert_ind + 1].prev_path) and
             (vertex_ind     != (*next_plans[next_plans_ind])[insert_ind + 1].prev_path->from_vertex))
          {
            (*next_plans[next_plans_ind])[insert_ind + 1].prev_path = &graph.minTravelTimePath(vertex_ind, (*next_plans[next_plans_ind])[insert_ind + 1].vertex_ind);
          }
          assert((temp_plan_size == (vertex_ind + 1)) or (nullptr == (*next_plans[next_plans_ind])[insert_ind + 1].prev_path) or (vertex_ind == (*next_plans[next_plans_ind])[insert_ind + 1].prev_path->from_vertex));
        });
      });
      /// Check to be finished
      {
        Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> costs(temp_len);
        std::for_each(std::execution::par_unseq, temp_inds.begin(), temp_inds.end(),
        [&] (const uint32_t ind) -> void
        {
          if(nullptr != next_plans[ind].get())
          {
            costs[ind] = cost_func(*next_plans[ind]).second;
          }
          else
          {
            costs[ind] = std::numeric_limits<double>::infinity();
          }
        });

        Eigen::Index best_ind;
        const double best_next_cost = costs.template minCoeff<Eigen::NaNPropagationOptions::PropagateFast>(&best_ind);
        if(best_next_cost < new_cost)
        {
          // Update info
          new_plan      = std::move(next_plans[best_ind]);
          new_cost      = best_next_cost;
          new_plan_len  = new_plan->size();
          new_plan_end  = new_plan->cend();
          new_plan_inds = boost::integer_range<uint32_t>(0, new_plan_len);
          continue;
        }
      }
    }

    /// Switch path moves
    {
      Eigen::Matrix<double,Eigen::Dynamic,1> next_costs(new_plan_len-2);
      next_plans.resize(new_plan_len-2);
      std::for_each(std::execution::unseq, std::next(new_plan_inds.begin()), std::prev(new_plan_inds.end()),
      [&] (const Eigen::Index to_mod_ind) -> void
      {
        if((nullptr == (*new_plan)[to_mod_ind].prev_path) or
           (1 == graph.cgetPointToPointPaths()((*new_plan)[to_mod_ind - 1].vertex_ind, (*new_plan)[to_mod_ind].vertex_ind).size()))
        {
          next_costs[to_mod_ind-1] = std::numeric_limits<double>::infinity();
          return;
        }

        const std::vector<graph::Path>& ptp_paths  = graph.cgetPointToPointPaths()((*new_plan)[to_mod_ind].prev_path->from_vertex, (*new_plan)[to_mod_ind].prev_path->to_vertex);

        const uint32_t                       ptp_paths_len = ptp_paths.size();
        const boost::integer_range<uint32_t> ptp_paths_inds(0, ptp_paths_len);

        Eigen::Matrix<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,Eigen::Dynamic,1,EIG_OPTIONS> sub_next_plans(ptp_paths_len);
        // Copy all but the mod one
        std::for_each(std::execution::unseq, ptp_paths_inds.begin(), ptp_paths_inds.end(),
        [&] (const uint32_t ptp_path_ind) -> void
        {
          if(&ptp_paths[ptp_path_ind] != (*new_plan)[to_mod_ind].prev_path)
          {
            sub_next_plans[ptp_path_ind] = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(*new_plan);
            (*sub_next_plans[ptp_path_ind])[to_mod_ind].prev_path = &ptp_paths[ptp_path_ind];
          }
        });

        // Find best of that set
        Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> costs(ptp_paths_len);
        std::for_each(std::execution::par_unseq, ptp_paths_inds.begin(), ptp_paths_inds.end(),
        [&] (const uint32_t ind) -> void
        {
          if(nullptr != sub_next_plans[ind].get())
          {
            costs[ind] = cost_func(*sub_next_plans[ind]).second;
          }
          else
          {
            costs[ind] = std::numeric_limits<double>::infinity();
          }
        });

        Eigen::Index best_ind;
        next_costs[to_mod_ind-1] = costs.template minCoeff<Eigen::NaNPropagationOptions::PropagateFast>(&best_ind);
        next_plans[to_mod_ind-1] = std::move(sub_next_plans[best_ind]);
      });
      /// Check to be finished
      {
        Eigen::Index best_ind;
        const double best_next_cost = next_costs.template minCoeff<Eigen::NaNPropagationOptions::PropagateFast>(&best_ind);
        if(best_next_cost < new_cost)
        {
          // Update info
          new_plan     = std::move(next_plans[best_ind]);
          new_cost     = best_next_cost;
          new_plan_end = new_plan->cend();
          continue;
        }
      }
    }

    break; // No improvement made
  }

  return std::pair<std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,double>(std::move(new_plan), new_cost);
}

#endif
/* local_search.hpp */
