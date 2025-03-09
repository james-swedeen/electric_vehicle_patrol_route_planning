/**
 * @File: planner_test.cpp
 * @Date: March 2024
 * @Author: James Swedeen
 *
 * @brief
 * Simple demo for testing planners that plotting.
 **/

/* C++ Headers */
#include<unistd.h>
#include<random>
#include<atomic>

/* Plotting Headers */
#include<matplotlibcpp/matplotlibcpp.hpp>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Plotting Messages Headers */
#include<graph_surveillance_planning_msgs/msg/plot_plan.hpp>

/* Local Headers */
#include<surveillance_planner/hotspot.hpp>
#include<surveillance_planner/random_planner.hpp>
#include<surveillance_planner/greedy_planner.hpp>
#include<surveillance_planner/cofield_accent_planner.hpp>
#include<surveillance_planner/metrics.hpp>
#include<surveillance_planner/levenshtein_distance.hpp>
#include<surveillance_planner/simulated_annealing.hpp>
#include<surveillance_planner/firefly_planner.hpp>
#include<surveillance_planner/exhaustive_planner.hpp>
#include<surveillance_planner/local_search.hpp>


std::vector<double> toVec(const Eigen::Ref<const Eigen::Matrix<double,Eigen::Dynamic,1>>& input)
{
  std::vector<double> output(input.rows());

  for(Eigen::Index col_it = 0; col_it < input.rows(); ++col_it)
  {
    output[col_it] = input[col_it];
  }

  return output;
}

inline static constexpr const double                  start_time         = 0;
inline static constexpr const double                  end_time           = 48*60*60;
inline static constexpr const double                  shift_length       = 12.0 * 60.0 * 60.0;
inline static constexpr const Eigen::Index            num_agent_active   = 15; // 12
inline static constexpr const Eigen::Index            NUM_AGENTS         = std::ceil((end_time - start_time) / (shift_length / double(num_agent_active))) + (num_agent_active - 1);
inline static constexpr const Eigen::Index            NUM_HOTSPOTS       = 75;
inline static constexpr const Eigen::Index            NUM_EVENT_POINTS   = 30;
inline static constexpr const Eigen::Index            MAX_PLAN_VISITS    = Eigen::Dynamic; // 3000; // 2 << 11;
inline static constexpr const Eigen::StorageOptions   EIG_OPTIONS        = Eigen::StorageOptions(Eigen::ColMajor bitor Eigen::AutoAlign);
inline static constexpr const plan::ConstraintOptions CONSTRAINT_OPTIONS = plan::ConstraintOptions(plan::ConstraintOptions::USE_SOC_CONSTRAINTS bitor plan::ConstraintOptions::USE_CONSTRAINTS_IN_RESPONSE_TIME_CALCULATION);// bitor plan::ConstraintOptions::USE_DISTANCE_CONSTRAINTS bitor plan::ConstraintOptions::VERBOSE);

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("planner_test");

  std::cout << "num agent: " << NUM_AGENTS << std::endl;

  // Plotting helpers
  rclcpp::Publisher<graph_surveillance_planning_msgs::msg::PlotPlan>::SharedPtr plan_pub = node->create_publisher<graph_surveillance_planning_msgs::msg::PlotPlan>("/plan", 3);

  const bool use_field_accent_planner        = false;
  const bool use_exhaustive_planner          = false;
  const bool use_random_planner              = false;
  const bool use_local_search_planner        = false;
  const bool use_greedy_planner              = true;
  const bool use_greedy_and_local_planner    = false;
  const bool use_firefly_planner             = false;
  const bool use_simulated_annealing_planner = false;

  const size_t num_rand_runs   = 1;
  const double cost_time_step  = 15.0*60.0;

  std::cout << "Loading Problem " << MAX_PLAN_VISITS << std::endl;
  // Generate problem
  const std::string base_dir("~/ros_ws/src/graph_surveillance_planning/street_graph/config/");
  const auto g_start_time = std::chrono::high_resolution_clock::now();
//  graph::Graph      graph(base_dir+"los_angeles_3500_nodes.csv", base_dir+"los_angeles_3500_edges.csv", 2500);
  graph::Graph      graph(base_dir+"new_york_15000_nodes.csv", base_dir+"new_york_15000_edges.csv", 10000);
//  graph::Graph      graph(base_dir+"seattle_5500_nodes.csv", base_dir+"seattle_5500_edges.csv", 2500);
  const auto g_end_time = std::chrono::high_resolution_clock::now();
  std::cout << "Took " << std::chrono::duration_cast<std::chrono::milliseconds>(g_end_time - g_start_time).count() << " milliseconds" << std::endl;

  const double max_charge      = 85.0*0.8;// / double(2); // max: 102 kWh
  const double max_trip_length = 455.444 * 1000.0;
  const double charging_rate   = 190.0 / double(60*60); // slow: 19.2 kW fast: ~190 kW
  const double plan_dt         = 60*30;

  const double poi_field_width_param   = 1.2213e+5;
  const double poi_field_search_weight = 77.888;
  const double poi_field_avoid_weight  = 1053.3;

  const boost::integer_range<uint32_t> orig_node_inds(0, graph.numberOriginalNodes());
//  {
//    std::vector<std::pair<size_t,size_t>> orig_node_ind_pairs;
//    orig_node_ind_pairs.reserve(graph.numberOriginalNodes() * graph.numberOriginalNodes());
//    std::for_each(orig_node_inds.begin(), orig_node_inds.end(),
//    [&] (const size_t start_ind) -> void
//    {
//      std::for_each(orig_node_inds.begin(), orig_node_inds.end(),
//      [&] (const size_t end_ind) -> void
//      {
//        orig_node_ind_pairs.emplace_back(start_ind, end_ind);
//      });
//    });
//    const auto min_max_it = std::minmax_element(std::execution::par_unseq, orig_node_ind_pairs.cbegin(), orig_node_ind_pairs.cend(),
//                            [&] (const std::pair<size_t,size_t>& i, const std::pair<size_t,size_t>& j) -> bool
//                            {
//                              return graph.minCharge(i.first, i.second) < graph.minCharge(j.first, j.second);
//                            });
//    std::cout << "Min PtP Charge Usage:     " << graph.minCharge(min_max_it.first-> first, min_max_it.first-> second) << std::endl;
//    std::cout << "Max min PtP Charge Usage: " << graph.minCharge(min_max_it.second->first, min_max_it.second->second) << std::endl;
//  }

  // Make agents
  std::cout << "Making Agents" << std::endl;
  plan::AGENTS_VEC_TYPE<NUM_AGENTS,EIG_OPTIONS> agents;
  {
    Eigen::Index agents_ind       = 0;
    const double shift_start_diff = shift_length / double(num_agent_active);
    double       shift_start_time = start_time - (double(num_agent_active - 1) * shift_start_diff);
    while(shift_start_time < end_time)
    {
      agents[agents_ind++] = plan::Agent(max_charge,
                                         charging_rate,
                                         max_trip_length,
                                         max_charge,
                                         0,
                                         (shift_start_time < start_time) ? start_time : shift_start_time,
                                         ((shift_start_time + shift_length) > end_time) ? end_time : shift_start_time + shift_length);
      shift_start_time += shift_start_diff;
      assert((end_time - start_time) >= agents[agents_ind-1].maxChargeTime());
    }
    assert(agents_ind == NUM_AGENTS);
  }

  // Pick hotspot
  std::cout << "Picking Hotspots" << std::endl;
  std::vector<uint32_t> hotspot_inds;
  hotspot_inds.reserve(NUM_HOTSPOTS);
  if(NUM_HOTSPOTS >= 2)
  {
    hotspot_inds.emplace_back(*std::min_element(std::execution::par_unseq, orig_node_inds.begin(), orig_node_inds.end(),
                               [&] (const uint32_t i, const uint32_t j) -> bool
                               {
                                 return graph.cgetOriginalNodes()[i]->cgetPosition()[2] < graph.cgetOriginalNodes()[j]->cgetPosition()[2];
                               }));
    hotspot_inds.emplace_back(*std::max_element(std::execution::par_unseq, orig_node_inds.begin(), orig_node_inds.end(),
                               [&] (const uint32_t i, const uint32_t j) -> bool
                               {
                                 return graph.cgetOriginalNodes()[i]->cgetPosition()[2] < graph.cgetOriginalNodes()[j]->cgetPosition()[2];
                               }));
  }
  size_t seed = 540;
  do
  {
    std::sample(orig_node_inds.begin(), orig_node_inds.end(), std::back_inserter(hotspot_inds), NUM_HOTSPOTS - hotspot_inds.size(), std::mt19937{seed++});
    std::sort(std::execution::par_unseq, hotspot_inds.begin(), hotspot_inds.end());
    hotspot_inds.erase(std::unique(std::execution::par_unseq, hotspot_inds.begin(), hotspot_inds.end()), hotspot_inds.end());
  }
  while(hotspot_inds.size() < NUM_HOTSPOTS);

  // Make planning graph
  std::cout << "Making Planning Graph" << std::endl;
  const auto pg_start_time = std::chrono::high_resolution_clock::now();
  graph::PlanningGraph planning_graph(graph, hotspot_inds, max_charge, shift_length, max_trip_length, plan::socConstraintsFlag(CONSTRAINT_OPTIONS), plan::distanceConstraintsFlag(CONSTRAINT_OPTIONS));
  const auto pg_end_time = std::chrono::high_resolution_clock::now();
  std::cout << "Took " << std::chrono::duration_cast<std::chrono::milliseconds>(pg_end_time - pg_start_time).count() << " milliseconds" << std::endl;

//  exit(EXIT_SUCCESS);

  std::cout << "Original Node Count:   " << graph.numberOriginalNodes()   << std::endl;
  std::cout << "Simplified Node Count: " << graph.numberSimplifiedNodes() << std::endl;
  std::cout << "Original Edge Count:   " << graph.numberOriginalEdges()   << std::endl;
  std::cout << "Simplified Edge Count: " << graph.numberSimplifiedEdges() << std::endl;
  std::cout << "Path Count:            " << planning_graph.numberPaths()  << std::endl;

  // Make hotspots
  plan::HOTSPOTS_VEC_TYPE<NUM_HOTSPOTS,EIG_OPTIONS> hotspots;
  for(uint32_t hs_ind = 0; hs_ind < hotspot_inds.size(); ++hs_ind)
  {
    hotspots[hs_ind] = plan::Hotspot(hs_ind + 1,
                                     0.041);
  }

  const uint32_t init_max_dwell_time    = 11;
  const uint32_t max_dwell_time_depot   = 204;
  const uint32_t max_dwell_time_hotspot = 14;

  // Approximate vehicle efficiency
//  double energy_total_kwh = 0;
//  std::for_each(graph.cgetSimplifiedEdges().cbegin(), graph.cgetSimplifiedEdges().cend(),
//                [&energy_total_kwh] (const std::unique_ptr<graph::EdgeBase>& edge) -> void
//  {
//    energy_total_kwh += edge->cgetTraversalCharge();
//  });
//  const double avg_efficiency = (energy_total_kwh / graph.totalEdgeLength())*double(1000000);
//  std::cout << "Average Efficiency: " << avg_efficiency << " Wh/km" << std::endl;

  // Make event points
  plan::EVENT_POINTS_VEC_TYPE<NUM_EVENT_POINTS,EIG_OPTIONS> event_points;
  std::sample(orig_node_inds.begin(), orig_node_inds.end(), event_points.begin(), NUM_EVENT_POINTS, std::mt19937{43});

  // Define objective function
  const double ecr_two_norm_weight = 1.0e-3;
  const double ecr_inf_norm_weight = 10;
  const double rt_two_norm_weight  = 1.0e-6;
  const double rt_inf_norm_weight  = 1.0e-2;

  std::mutex temp_mux;
  double ecr_two_norm_sum = 0;
  double ecr_inf_norm_sum = 0;
  double rt_two_norm_sum  = 0;
  double rt_inf_norm_sum  = 0;
  size_t count;
  const auto objective_func = [&] (const Eigen::Ref<const plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& cur_plan) -> std::pair<bool,double>
                              { // TODO: Remove extra stuff
//                                std::lock_guard<std::mutex> lock(temp_mux);
//                                ++count;
                                const std::pair<bool,double> output_old = plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,true,true,true,true>(cur_plan,
                                                                                    cost_time_step,
                                                                                    start_time,
                                                                                    end_time,
                                                                                    agents,
                                                                                    hotspots,
                                                                                    event_points,
                                                                                    graph,
                                                                                    ecr_two_norm_weight, ecr_inf_norm_weight, rt_two_norm_weight, rt_inf_norm_weight);
//                                if(not output_old.first) { return output_old; }
//                                ecr_two_norm_sum += plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,true,false,false,false>(cur_plan,
//                                                                                    cost_time_step,
//                                                                                    start_time,
//                                                                                    end_time,
//                                                                                    agents,
//                                                                                    hotspots,
//                                                                                    event_points,
//                                                                                    graph,
//                                                                                    ecr_two_norm_weight, ecr_inf_norm_weight, rt_two_norm_weight, rt_inf_norm_weight).second;
//                                ecr_inf_norm_sum += plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,true,false,false>(cur_plan,
//                                                                                    cost_time_step,
//                                                                                    start_time,
//                                                                                    end_time,
//                                                                                    agents,
//                                                                                    hotspots,
//                                                                                    event_points,
//                                                                                    graph,
//                                                                                    ecr_two_norm_weight, ecr_inf_norm_weight, rt_two_norm_weight, rt_inf_norm_weight).second;
//                                rt_two_norm_sum += plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,false,true,false>(cur_plan,
//                                                                                    cost_time_step,
//                                                                                    start_time,
//                                                                                    end_time,
//                                                                                    agents,
//                                                                                    hotspots,
//                                                                                    event_points,
//                                                                                    graph,
//                                                                                    ecr_two_norm_weight, ecr_inf_norm_weight, rt_two_norm_weight, rt_inf_norm_weight).second;
//                                rt_inf_norm_sum += plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,false,false,true>(cur_plan,
//                                                                                    cost_time_step,
//                                                                                    start_time,
//                                                                                    end_time,
//                                                                                    agents,
//                                                                                    hotspots,
//                                                                                    event_points,
//                                                                                    graph,
//                                                                                    ecr_two_norm_weight, ecr_inf_norm_weight, rt_two_norm_weight, rt_inf_norm_weight).second;
                                return output_old;
                              };

  const auto objective_func_old = [&] (const std::vector<std::vector<plan::Action>>& cur_plan) -> std::pair<bool,double>
                              {
                                // Convert to Eigen
                                const uint32_t num_actions_in_plan =
                                  std::accumulate<std::vector<std::vector<plan::Action>>::const_iterator,long>(cur_plan.cbegin(), cur_plan.cend(), 0,
                                                                                                               [] (const long prev_val, const std::vector<plan::Action>& sub_plan) -> long { return prev_val + sub_plan.size(); });

                                plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS> eig_output(num_actions_in_plan);
                                Eigen::Index                                 eig_out_ind = 0;
                                for(uint32_t agent_ind = 0; agent_ind < NUM_AGENTS; ++agent_ind)
                                {
                                  uint32_t sub_len = cur_plan[agent_ind].size();
                                  for(uint32_t sub_ind = 0; sub_ind < sub_len; ++sub_ind, ++eig_out_ind)
                                  {
                                    eig_output[eig_out_ind] = cur_plan[agent_ind][sub_ind];
                                  }
                                }

                                return objective_func(eig_output);
                              };
  // Plan
  std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan;

  double exhaustive_planner_time_sec;
  double exhaustive_planner_cost;
  if(use_exhaustive_planner)
  {
    std::cout << "Exhaustive Planning" << std::endl;
    {
      const auto plan_start_time = std::chrono::high_resolution_clock::now();
      plan = plan::generateExhaustivePlan<NUM_AGENTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(planning_graph,
                                                                                  agents,
                                                                                  start_time,
                                                                                  end_time,
                                                                                  objective_func_old);
      const auto plan_end_time = std::chrono::high_resolution_clock::now();
      exhaustive_planner_time_sec = double(std::chrono::duration_cast<std::chrono::nanoseconds>(plan_end_time - plan_start_time).count()) * double(1.0e-9);
      std::cout << "Took " << exhaustive_planner_time_sec << " seconds" << std::endl;

      std::cout << "Running Objective Function" << std::endl;
      const auto obj_start_time = std::chrono::high_resolution_clock::now();
      const std::pair<bool,double> obj_func_result = objective_func(*plan);
      const auto obj_end_time = std::chrono::high_resolution_clock::now();
      assert(obj_func_result.first);
      std::cout << "Objective function value: " << obj_func_result.second << std::endl;
      exhaustive_planner_cost = obj_func_result.second;
      std::cout << "Took " << std::chrono::duration_cast<std::chrono::milliseconds>(obj_end_time - obj_start_time).count() << " milliseconds" << std::endl;
      std::cout << "plan length: " << plan->size() << std::endl;
      if(MAX_PLAN_VISITS == plan->size())
      {
        std::cout << "Warning: Max plan length was met." << std::endl;
      }
    }
  }

  Eigen::Matrix<double,Eigen::Dynamic,1> random_planner_times_sec(num_rand_runs);
  Eigen::Matrix<double,Eigen::Dynamic,1> random_planner_cost_sec( num_rand_runs);
  Eigen::Matrix<double,Eigen::Dynamic,1> random_planner_costs(    num_rand_runs);
  if(use_random_planner)
  {
    std::cout << "Random Planning" << std::endl;
    for(size_t rand_it = 0; rand_it < num_rand_runs; ++rand_it)
    {
      const auto plan_start_time = std::chrono::high_resolution_clock::now();
      plan = plan::generateRandomPlan<NUM_AGENTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(planning_graph,
                                                                                                 agents,
                                                                                                 init_max_dwell_time,
                                                                                                 42+rand_it);
      const auto plan_end_time = std::chrono::high_resolution_clock::now();
      random_planner_times_sec[rand_it] = double(std::chrono::duration_cast<std::chrono::nanoseconds>(plan_end_time - plan_start_time).count()) * double(1.0e-9);

      const auto cost_start_time = std::chrono::high_resolution_clock::now();
      const std::pair<bool,double> obj_func_result = objective_func(*plan);
      const auto cost_end_time = std::chrono::high_resolution_clock::now();
      random_planner_cost_sec[rand_it] = double(std::chrono::duration_cast<std::chrono::nanoseconds>(cost_end_time - cost_start_time).count()) * double(1.0e-9);
      assert(obj_func_result.first);
      random_planner_costs[rand_it] = obj_func_result.second;
      std::cout << "plan length: " << plan->size() << std::endl;
      if(MAX_PLAN_VISITS == plan->size())
      {
        std::cout << "Warning: Max plan length was met." << std::endl;
      }
    }
    std::cout << "Average objective function value: " << random_planner_costs.mean()         << std::endl;
    std::cout << "Average time:                     " << random_planner_times_sec.mean()     << std::endl;
    std::cout << "Average Objective time:           " << random_planner_cost_sec.mean()      << std::endl;
    std::cout << "Min objective function value:     " << random_planner_costs.minCoeff()     << std::endl;
    std::cout << "Min time:                         " << random_planner_times_sec.minCoeff() << std::endl;
    std::cout << "Max objective function value:     " << random_planner_costs.maxCoeff()     << std::endl;
    std::cout << "Max time:                         " << random_planner_times_sec.maxCoeff() << std::endl;
  }

  Eigen::Matrix<double,Eigen::Dynamic,1> local_search_planner_times_sec(num_rand_runs);
  Eigen::Matrix<double,Eigen::Dynamic,1> local_search_planner_costs(    num_rand_runs);
  if(use_local_search_planner)
  {
    std::cout << "Local Search Planning" << std::endl;
    for(size_t rand_it = 0; rand_it < num_rand_runs; ++rand_it)
    {
      const auto plan_start_time = std::chrono::high_resolution_clock::now();
      plan = plan::generateRandomPlan<NUM_AGENTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(planning_graph,
                                                                                                 agents,
                                                                                                 init_max_dwell_time,
                                                                                                 42+rand_it);
      while(true)
      {
        std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> new_plan = plan::localSearch<MAX_PLAN_VISITS,EIG_OPTIONS>(*plan,
                                                                                                                                max_dwell_time_hotspot,
                                                                                                                                max_dwell_time_depot,
                                                                                                                                planning_graph,
                                                                                                                                objective_func).first;
        std::swap(new_plan, plan);
        if((new_plan->rows() == plan->rows()) and (*new_plan == *plan)) { break; }
        std::cout << "loop" << std::endl;
      }
      const auto plan_end_time = std::chrono::high_resolution_clock::now();
      local_search_planner_times_sec[rand_it] = double(std::chrono::duration_cast<std::chrono::nanoseconds>(plan_end_time - plan_start_time).count()) * double(1.0e-9);

      const std::pair<bool,double> obj_func_result = objective_func(*plan);
      assert(obj_func_result.first);
      local_search_planner_costs[rand_it] = obj_func_result.second;
      std::cout << "plan length: " << plan->size() << std::endl;
      if(MAX_PLAN_VISITS == plan->size())
      {
        std::cout << "Warning: Max plan length was met." << std::endl;
      }
    }
    std::cout << "Average objective function value: " << local_search_planner_costs.mean()         << std::endl;
    std::cout << "Average time:                     " << local_search_planner_times_sec.mean()     << std::endl;
    std::cout << "Min objective function value:     " << local_search_planner_costs.minCoeff()     << std::endl;
    std::cout << "Min time:                         " << local_search_planner_times_sec.minCoeff() << std::endl;
    std::cout << "Max objective function value:     " << local_search_planner_costs.maxCoeff()     << std::endl;
    std::cout << "Max time:                         " << local_search_planner_times_sec.maxCoeff() << std::endl;
  }

  double greedy_planner_time_sec = 100;
  double greedy_planner_cost;
  if(use_greedy_planner)
  {
    std::cout << "Greedy Planning" << std::endl;
    {
      const auto plan_start_time = std::chrono::high_resolution_clock::now();
      plan = plan::generateGreedyPlan<NUM_AGENTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(planning_graph,
                                                                                                 agents,
                                                                                                 init_max_dwell_time,
                                                                                                 objective_func);
      const auto plan_end_time = std::chrono::high_resolution_clock::now();
      greedy_planner_time_sec = double(std::chrono::duration_cast<std::chrono::nanoseconds>(plan_end_time - plan_start_time).count()) * double(1.0e-9);
      std::cout << "Took " << greedy_planner_time_sec << " seconds" << std::endl;
      const std::pair<bool,double> obj_func_result = objective_func(*plan);
      assert(obj_func_result.first);
      std::cout << "Objective function value: " << obj_func_result.second << std::endl;
      greedy_planner_cost = obj_func_result.second;
      std::cout << "plan length: " << plan->size() << std::endl;
      if(MAX_PLAN_VISITS == plan->size())
      {
        std::cout << "Warning: Max plan length was met." << std::endl;
      }
    }
  }

  Eigen::Matrix<std::vector<double>,Eigen::Dynamic,1>                                fa_planner_time_sec(num_rand_runs);
  Eigen::Matrix<std::deque<Eigen::Matrix<double,Eigen::Dynamic,1>>,Eigen::Dynamic,1> fa_planner_cost(    num_rand_runs);
  if(use_firefly_planner)
  {
    std::cout << "Planning with Firefly" << std::endl;

    for(size_t rand_it = 0; rand_it < num_rand_runs; ++rand_it)
    {
      fa_planner_time_sec[rand_it].reserve(999999);

      static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_K_MEANS_SELECTION bitor plan::FaFlags::USE_ELITISM_SELECTION bitor plan::FaFlags::USE_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH);

      plan::FaParams<plan::mo::MutationsFlags::USE_RECOMMENDATIONS> fa_params(R"({
  "accept_cost_threshold": 0.0,
  "base_attractiveness_multiplier": 0.0,
  "cooling_rate": 0.0,
  "distance_multiplier": 0.0,
  "distance_power": 0.0,
  "field_accent_avoid_weight": 9999.999999999996,
  "field_accent_search_weight": 499.9999999999997,
  "field_accent_width": 46168.29327812841,
  "group_number": 0,
  "initial_temperature": 0.0,
  "initialization_max_dwell_time": 11,
  "local_search_max_time_sec": 59.455358329342296,
  "local_search_probability": 0.12002047492784901,
  "max_dwell_time_depot": 334,
  "max_dwell_time_hotspot": 11,
  "min_temperature": 0.0,
  "mop_add_depot_visit": 0.9386350810164745,
  "mop_add_hotspot_visit": 0.9999999999999993,
  "mop_remove_depot_visit": 7.104980349886509e-16,
  "mop_remove_hotspot_visit": 1.0,
  "mop_remove_visits_windowed": 0.26886277090791544,
  "mop_sim": 3.7089610701782246e-16,
  "mop_swap_multiple_visits": 0.011334924425652123,
  "mop_swap_path": 1.3115941951477971e-15,
  "mop_swap_visits": 3.6454000238890817e-16,
  "mop_tweak_dwell_time": 1.0,
  "number_cluster_iterations": 0,
  "number_clusters": 1,
  "number_elite": 0,
  "population_size": 3,
  "sa_max_time_sec": 0.0,
  "tweak_dwell_time_max_change": 5,
  "use_firefly_operator": 4,
  "use_selection": 3
})");

      const auto plan_start_time = std::chrono::high_resolution_clock::now();
      auto       last_start_time = std::chrono::high_resolution_clock::now();
      plan = plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(
               fa_params,
               planning_graph,
               [&] (const unsigned rand_seed) -> std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
               {
                 return plan::generateRandomPlan<NUM_AGENTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(planning_graph,
                                                                                                            agents,
                                                                                                            init_max_dwell_time,
                                                                                                            rand_seed);
               },
               objective_func,
               [&] (const size_t generation_count, const double best_cost_found, const Eigen::Ref<const plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& best_plan_found) -> bool
               {
//                 std::cout << generation_count << std::endl;
//                 return generation_count >= 30;
                 //return (greedy_planner_time_sec * double(2)) <= (double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9));
                  return double(10 * 60) <= (double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9));
               },
               [&] (const Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS>& cur_population) -> void
               {
                 auto log_start_time = std::chrono::high_resolution_clock::now();

                 if(fa_planner_time_sec[rand_it].empty())
                 {
                   fa_planner_time_sec[rand_it].emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(log_start_time - last_start_time).count()) * double(1.0e-9));
                 }
                 else
                 {
                   fa_planner_time_sec[rand_it].emplace_back((double(std::chrono::duration_cast<std::chrono::nanoseconds>(log_start_time - last_start_time).count()) * double(1.0e-9)) + fa_planner_time_sec[rand_it].back());
                 }
                 fa_planner_cost[rand_it].emplace_back(cur_population);

                 last_start_time = std::chrono::high_resolution_clock::now();
               },
               std::vector<std::function<std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>()>>({
                 [&] () -> std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
                 {
                   return plan::generateGreedyPlan<NUM_AGENTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(planning_graph,
                                                                                                              agents,
                                                                                                              init_max_dwell_time,
                                                                                                              objective_func);
                 }
               }),
               42+rand_it);
      const auto plan_end_time = std::chrono::high_resolution_clock::now();
      std::cout << "Took " << double(std::chrono::duration_cast<std::chrono::nanoseconds>(plan_end_time - plan_start_time).count()) * double(1.0e-9) << " seconds" << std::endl;

      std::cout << "Running Objective Function" << std::endl;
      const auto obj_start_time = std::chrono::high_resolution_clock::now();
      const std::pair<bool,double> obj_func_result = objective_func(*plan);
      const auto obj_end_time = std::chrono::high_resolution_clock::now();
      assert(obj_func_result.first);
      std::cout << "Objective function value: " << obj_func_result.second << std::endl;
      std::cout << "Took " << std::chrono::duration_cast<std::chrono::milliseconds>(obj_end_time - obj_start_time).count() << " milliseconds" << std::endl;
      std::cout << "plan length: " << plan->size() << std::endl;
      if(MAX_PLAN_VISITS == plan->size())
      {
        std::cout << "Warning: Max plan length was met." << std::endl;
      }
    }
    std::cout << "Average objective function value: " << fa_planner_cost.unaryExpr([] (const std::deque<Eigen::Matrix<double,Eigen::Dynamic,1>>& i) -> double { return i.back().minCoeff(); }).mean()          << std::endl;
    std::cout << "Average time:                     " << fa_planner_time_sec.unaryExpr([] (const std::vector<double>& i) -> double { return i.back(); }).mean()      << std::endl;
  }

  double field_accent_planner_time_sec;
  double field_accent_planner_cost;
  if(use_field_accent_planner)
  {
//    {
//      double best_cost_found = std::numeric_limits<double>::infinity();
//      std::mutex best_cost_mux;
//      std::list<std::pair<std::thread,bool>> threads;
//      for(double poi_field_search_weight = 1; poi_field_search_weight < 100; poi_field_search_weight += 1)
//      {
//      for(double poi_field_avoid_weight = 1; poi_field_avoid_weight < 1000; poi_field_avoid_weight += 1)
//      {
//      for(double poi_field_width_param = 100; poi_field_width_param < 10000; poi_field_width_param += 1)
//      {
//        threads.emplace_back();
//        threads.back().second = false;
//        threads.back().first = std::thread([&] (bool& end_flag) -> void
//        {
//          std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan = plan::generateCofieldAccentPlan<NUM_AGENTS,NUM_HOTSPOTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(
//                   planning_graph,
//                   agents,
//                   hotspots,
//                   opt_dwell_time_hotspot,
//                   start_time,
//                   end_time,
//                   poi_field_search_weight,
//                   poi_field_avoid_weight,
//                   poi_field_width_param);
//          const std::pair<bool,double> obj_func_result = objective_func(*plan);
//          std::lock_guard<std::mutex> lock(best_cost_mux);
//          if(best_cost_found > obj_func_result.second)
//          {
//            best_cost_found = obj_func_result.second;
//            std::cout << "poi_field_search_weight: " << poi_field_search_weight
//                      << " poi_field_avoid_weight: " << poi_field_avoid_weight
//                      << " poi_field_width_param: " << poi_field_width_param
//                      << std::endl;
//          }
//          end_flag = true;
//        }, std::ref(threads.back().second));
//        while(threads.size() > 100)
//        {
//          threads.remove_if([] (std::pair<std::thread,bool>& th) -> bool { if(th.second) { th.first.join(); return true; } return false; });
//        }
//      }
//      }
//      }
//    }
    std::cout << "Field Accent Planning" << std::endl;
    {
      const auto plan_start_time = std::chrono::high_resolution_clock::now();
      plan = plan::generateCofieldAccentPlan<NUM_AGENTS,NUM_HOTSPOTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(
               planning_graph,
               agents,
               hotspots,
               init_max_dwell_time,
               start_time,
               end_time,
               poi_field_search_weight,
               poi_field_avoid_weight,
               poi_field_width_param);
      const auto plan_end_time = std::chrono::high_resolution_clock::now();
      field_accent_planner_time_sec = double(std::chrono::duration_cast<std::chrono::nanoseconds>(plan_end_time - plan_start_time).count()) * double(1.0e-9);
      std::cout << "Took " << field_accent_planner_time_sec << " seconds" << std::endl;

      std::cout << "Running Objective Function" << std::endl;
      const auto obj_start_time = std::chrono::high_resolution_clock::now();
      const std::pair<bool,double> obj_func_result = objective_func(*plan);
      const auto obj_end_time = std::chrono::high_resolution_clock::now();
      assert(obj_func_result.first);
      std::cout << "Objective function value: " << obj_func_result.second << std::endl;
      field_accent_planner_cost = obj_func_result.second;
      std::cout << "Took " << std::chrono::duration_cast<std::chrono::microseconds>(obj_end_time - obj_start_time).count() << " microseconds" << std::endl;
      std::cout << "plan length: " << plan->size() << std::endl;
      if(MAX_PLAN_VISITS == plan->size())
      {
        std::cout << "Warning: Max plan length was met." << std::endl;
      }
    }
  }

  double greedy_and_local_planner_time_sec = 100;
  double greedy_and_local_planner_cost;
  if(use_greedy_and_local_planner)
  {
    std::cout << "Greedy and Local Planning" << std::endl;
    {
      const auto plan_start_time = std::chrono::high_resolution_clock::now();
      plan = plan::generateGreedyPlan<NUM_AGENTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(planning_graph,
                                                                                                 agents,
                                                                                                 init_max_dwell_time,
                                                                                                 objective_func);
      while(true)
      {
        std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> new_plan = plan::localSearch<MAX_PLAN_VISITS,EIG_OPTIONS>(*plan,
                                                                                                                                max_dwell_time_hotspot,
                                                                                                                                max_dwell_time_depot,
                                                                                                                                planning_graph,
                                                                                                                                objective_func).first;
        std::swap(new_plan, plan);
        if((new_plan->rows() == plan->rows()) and (*new_plan == *plan)) { break; }
        std::cout << "loop" << std::endl;
      }
      const auto plan_end_time = std::chrono::high_resolution_clock::now();
      greedy_and_local_planner_time_sec = double(std::chrono::duration_cast<std::chrono::nanoseconds>(plan_end_time - plan_start_time).count()) * double(1.0e-9);
      std::cout << "Took " << greedy_and_local_planner_time_sec << " seconds" << std::endl;
      const std::pair<bool,double> obj_func_result = objective_func(*plan);
      assert(obj_func_result.first);
      std::cout << "Objective function value: " << obj_func_result.second << std::endl;
      greedy_and_local_planner_cost = obj_func_result.second;
      std::cout << "plan length: " << plan->size() << std::endl;
      if(MAX_PLAN_VISITS == plan->size())
      {
        std::cout << "Warning: Max plan length was met." << std::endl;
      }
    }
  }

  std::vector<std::vector<double>>                                sa_planner_time_sec(num_rand_runs);
  std::vector<std::deque<Eigen::Matrix<double,Eigen::Dynamic,1>>> sa_planner_cost(    num_rand_runs);
  if(use_simulated_annealing_planner)
  {
    std::cout << "Planning with Simulated Annealing" << std::endl;

    for(size_t rand_it = 0; rand_it < num_rand_runs; ++rand_it)
    {
      sa_planner_time_sec[rand_it].reserve(999999);

      static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_K_MEANS_SELECTION bitor plan::SaFlags::USE_ELITISM_SELECTION bitor plan::SaFlags::USE_LOCAL_SEARCH);

      plan::SaParams<plan::mo::MutationsFlags::USE_RECOMMENDATIONS> sa_params(R"({
                "accept_cost_threshold": 4.492767687969428,
                "base_attractiveness_multiplier": 0.0,
                "cooling_rate": 0.5208223062620193,
                "distance_multiplier": 0.0,
                "distance_power": 0.0,
                "field_accent_avoid_weight": 4206.805902608776,
                "field_accent_search_weight": 499.99999999864946,
                "field_accent_width": 160000.0,
                "group_number": 10,
                "initial_temperature": 1499.9999994596185,
                "initialization_max_dwell_time": 11,
                "local_search_max_time_sec": 60,
                "local_search_probability": 0.0,
                "max_dwell_time_depot": 399,
                "max_dwell_time_hotspot": 399,
                "min_temperature": 39.82447654286113,
                "mop_add_depot_visit": 0.999999999958731,
                "mop_add_hotspot_visit": 0.9999999990701445,
                "mop_remove_depot_visit": 1.0,
                "mop_remove_hotspot_visit": 6.40416609100157e-11,
                "mop_remove_visits_windowed": 1.5485579920406174e-09,
                "mop_sim": 0.9999999995631943,
                "mop_swap_multiple_visits": 1.0,
                "mop_swap_path": 2.8694021456207137e-11,
                "mop_swap_visits": 0.9999999999812859,
                "mop_tweak_dwell_time": 0.9999999998540277,
                "number_cluster_iterations": 0,
                "number_clusters": 3,
                "number_elite": 22,
                "population_size": 50,
                "sa_max_time_sec": 192.9669244178827,
                "tweak_dwell_time_max_change": 30,
                "use_firefly_operator": 3,
                "use_selection": 3})");

      const auto plan_start_time = std::chrono::high_resolution_clock::now();
      auto       last_start_time = std::chrono::high_resolution_clock::now();
      plan = plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(
               sa_params,
               planning_graph,
               [&] (const unsigned rand_seed) -> std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
               {
                 return plan::generateRandomPlan<NUM_AGENTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(planning_graph,
                                                                                                            agents,
                                                                                                            init_max_dwell_time,
                                                                                                            rand_seed);
               },
               objective_func,
               [&] (const size_t generation_count, const double best_cost_found, const Eigen::Ref<const plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& best_plan_found) -> bool
               {
//                 std::cout << generation_count << std::endl;
//                 return generation_count >= 10;
                 //return (greedy_planner_time_sec * double(2)) <= (double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9));
                 return double(10 * 60) <= (double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9));
               },
               [&] (const Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS>& cur_population) -> void
               {
                 auto log_start_time = std::chrono::high_resolution_clock::now();

                 if(sa_planner_time_sec[rand_it].empty())
                 {
                   sa_planner_time_sec[rand_it].emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(log_start_time - last_start_time).count()) * double(1.0e-9));
                 }
                 else
                 {
                   sa_planner_time_sec[rand_it].emplace_back((double(std::chrono::duration_cast<std::chrono::nanoseconds>(log_start_time - last_start_time).count()) * double(1.0e-9)) + sa_planner_time_sec[rand_it].back());
                 }
                 sa_planner_cost[rand_it].emplace_back(cur_population);

                 last_start_time = std::chrono::high_resolution_clock::now();
               },
               std::vector<std::function<std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>()>>({
                 [&] () -> std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
                 {
                   return plan::generateGreedyPlan<NUM_AGENTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(planning_graph,
                                                                                                              agents,
                                                                                                              init_max_dwell_time,
                                                                                                              objective_func);
                 }
               }),
               42+rand_it);
      const auto plan_end_time = std::chrono::high_resolution_clock::now();
      std::cout << "Took " << double(std::chrono::duration_cast<std::chrono::nanoseconds>(plan_end_time - plan_start_time).count()) * double(1.0e-9) << " seconds" << std::endl;

      std::cout << "Running Objective Function" << std::endl;
      const auto obj_start_time = std::chrono::high_resolution_clock::now();
      const std::pair<bool,double> obj_func_result = objective_func(*plan);
      const auto obj_end_time = std::chrono::high_resolution_clock::now();
      assert(obj_func_result.first);
      std::cout << "Objective function value: " << obj_func_result.second << std::endl;
      std::cout << "Took " << std::chrono::duration_cast<std::chrono::milliseconds>(obj_end_time - obj_start_time).count() << " milliseconds" << std::endl;
      std::cout << "plan length: " << plan->size() << std::endl;
      if(MAX_PLAN_VISITS == plan->size())
      {
        std::cout << "Warning: Max plan length was met." << std::endl;
      }
    }
  }
//  std::cout << ecr_two_norm_sum / double(count) << std::endl;
//  std::cout << ecr_inf_norm_sum / double(count) << std::endl;
//  std::cout << rt_two_norm_sum  / double(count) << std::endl;
//  std::cout << rt_inf_norm_sum  / double(count) << std::endl;

//  exit(EXIT_SUCCESS);

//  std::cout << "plan:\n";
//  for(size_t agent_ind = 0; agent_ind < plan.size(); ++agent_ind)
//  {
//    std::cout << plan[agent_ind] << std::endl;
//  }

  /// Plot results
  matplotlibcpp::figure();
  if(use_firefly_planner)
  {
    const size_t num_gen         = fa_planner_cost[0].size();
    const size_t population_size = fa_planner_cost[0][0].rows();

    std::vector<double> mean_cost(num_gen);
    std::vector<double> min_cost( num_gen);
    for(size_t gen_ind = 0; gen_ind < num_gen; ++gen_ind)
    {
      mean_cost[gen_ind] = fa_planner_cost[0][gen_ind].mean();
      min_cost[ gen_ind] = fa_planner_cost[0][gen_ind].minCoeff();
    }
    matplotlibcpp::named_plot<double>("Hybrid Firefly Planner", fa_planner_time_sec[0], min_cost, "k*-");

    for(size_t rand_it = 1; rand_it < num_rand_runs; ++rand_it)
    {
      const size_t num_gen         = fa_planner_cost[rand_it].size();
      const size_t population_size = fa_planner_cost[rand_it][0].rows();

      std::vector<double> mean_cost(num_gen);
      std::vector<double> min_cost( num_gen);
      for(size_t gen_ind = 0; gen_ind < num_gen; ++gen_ind)
      {
        mean_cost[gen_ind] = fa_planner_cost[rand_it][gen_ind].mean();
        min_cost[ gen_ind] = fa_planner_cost[rand_it][gen_ind].minCoeff();
      }
      matplotlibcpp::plot<double>(fa_planner_time_sec[rand_it], min_cost, "k*-");
    }
  }
  if(use_simulated_annealing_planner)
  {
    const size_t num_gen         = sa_planner_cost[0].size();
    const size_t population_size = sa_planner_cost[0][0].rows();

    std::vector<double> mean_cost(num_gen);
    std::vector<double> min_cost( num_gen);
    for(size_t gen_ind = 0; gen_ind < num_gen; ++gen_ind)
    {
      mean_cost[gen_ind] = sa_planner_cost[0][gen_ind].mean();
      min_cost[ gen_ind] = sa_planner_cost[0][gen_ind].minCoeff();
    }
    matplotlibcpp::named_plot<double>("Hybrid Population-Based Simulated Annealing", sa_planner_time_sec[0], min_cost, "m*-");
    for(size_t rand_it = 1; rand_it < num_rand_runs; ++rand_it)
    {
      const size_t num_gen         = sa_planner_cost[rand_it].size();
      const size_t population_size = sa_planner_cost[rand_it][0].rows();

      std::vector<double> mean_cost(num_gen);
      std::vector<double> min_cost( num_gen);
      for(size_t gen_ind = 0; gen_ind < num_gen; ++gen_ind)
      {
        mean_cost[gen_ind] = sa_planner_cost[rand_it][gen_ind].mean();
        min_cost[ gen_ind] = sa_planner_cost[rand_it][gen_ind].minCoeff();
      }
      matplotlibcpp::plot<double>(sa_planner_time_sec[rand_it], min_cost, "m*-");
    }
  }
  if(use_exhaustive_planner)
  {
    matplotlibcpp::named_plot<double>("Exhaustive Planner",
                                      std::vector<double>(1, exhaustive_planner_time_sec),
                                      std::vector<double>(1, exhaustive_planner_cost),
                                      "g*");
  }
  if(use_random_planner)
  {
    const double time_mean          = random_planner_times_sec.mean();
    const double cost_mean          = random_planner_costs.mean();
    const double time_three_std_dev = (((random_planner_times_sec.rowwise() - random_planner_times_sec.colwise().mean()).colwise().squaredNorm().array()/double(num_rand_runs-1)).sqrt() * double(3))[0];
    const double cost_three_std_dev = (((random_planner_costs.rowwise() - random_planner_costs.colwise().mean()).colwise().squaredNorm().array()/double(num_rand_runs-1)).sqrt() * double(3))[0];

    matplotlibcpp::named_plot<double>("Random Planner",
                                      std::vector<double>({time_mean - time_three_std_dev, time_mean + time_three_std_dev}),
                                      std::vector<double>({cost_mean, cost_mean}),
                                      "y");
    matplotlibcpp::plot(toVec(random_planner_times_sec),
                        toVec(random_planner_costs),
                        "y*");
    matplotlibcpp::plot(std::vector<double>({time_mean, time_mean}),
                        std::vector<double>({cost_mean - cost_three_std_dev, cost_mean + cost_three_std_dev}),
                        "y");
    matplotlibcpp::plot(std::vector<double>({time_mean - time_three_std_dev, time_mean - time_three_std_dev, time_mean + time_three_std_dev, time_mean + time_three_std_dev, time_mean - time_three_std_dev}),
                        std::vector<double>({cost_mean - cost_three_std_dev, cost_mean + cost_three_std_dev, cost_mean + cost_three_std_dev, cost_mean - cost_three_std_dev, cost_mean - cost_three_std_dev}),
                        "y--");
  }
  if(use_local_search_planner)
  {
    const double time_mean          = local_search_planner_times_sec.mean();
    const double cost_mean          = local_search_planner_costs.mean();
    const double time_three_std_dev = (((local_search_planner_times_sec.rowwise() - local_search_planner_times_sec.colwise().mean()).colwise().squaredNorm().array()/double(num_rand_runs-1)).sqrt() * double(3))[0];
    const double cost_three_std_dev = (((local_search_planner_costs.rowwise() - local_search_planner_costs.colwise().mean()).colwise().squaredNorm().array()/double(num_rand_runs-1)).sqrt() * double(3))[0];

    matplotlibcpp::named_plot<double>("Local Search Planner",
                                      std::vector<double>({time_mean - time_three_std_dev, time_mean + time_three_std_dev}),
                                      std::vector<double>({cost_mean, cost_mean}),
                                      "g");
    matplotlibcpp::plot(toVec(local_search_planner_times_sec),
                        toVec(local_search_planner_costs),
                        "g*");
    matplotlibcpp::plot(std::vector<double>({time_mean, time_mean}),
                        std::vector<double>({cost_mean - cost_three_std_dev, cost_mean + cost_three_std_dev}),
                        "g");
    matplotlibcpp::plot(std::vector<double>({time_mean - time_three_std_dev, time_mean - time_three_std_dev, time_mean + time_three_std_dev, time_mean + time_three_std_dev, time_mean - time_three_std_dev}),
                        std::vector<double>({cost_mean - cost_three_std_dev, cost_mean + cost_three_std_dev, cost_mean + cost_three_std_dev, cost_mean - cost_three_std_dev, cost_mean - cost_three_std_dev}),
                        "g--");
  }
  if(use_greedy_planner)
  {
    matplotlibcpp::named_plot<double>("Greedy Planner",
                                      std::vector<double>(1, greedy_planner_time_sec),
                                      std::vector<double>(1, greedy_planner_cost),
                                      "b*");
  }
  if(use_greedy_and_local_planner)
  {
    matplotlibcpp::named_plot<double>("Greedy and Local Planner",
                                      std::vector<double>(1, greedy_and_local_planner_time_sec),
                                      std::vector<double>(1, greedy_and_local_planner_cost),
                                      "b+");
  }
  if(use_field_accent_planner)
  {
    matplotlibcpp::named_plot<double>("Field Accent Planner",
                                      std::vector<double>(1, field_accent_planner_time_sec),
                                      std::vector<double>(1, field_accent_planner_cost),
                                      "c*");
  }
  matplotlibcpp::xlabel("Solve Time (sec)");
  matplotlibcpp::ylabel("Objective Function Value");
  matplotlibcpp::title("Planner Results");
  matplotlibcpp::legend();
//  matplotlibcpp::show();

  /// Test plan
  std::cout << "Testing Plan" << std::endl;
  const size_t TIME_IND      = 0;
  const size_t START_POS_IND = 1;
  const size_t START_SOC_IND = 1 + (agents.size() * 3);
  const size_t START_POI_IND = START_SOC_IND + agents.size();
  const size_t START_RT_IND  = START_POI_IND + NUM_HOTSPOTS;
//  {
//    const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> sim_vec = plan::planToSimulationVector(plan,
//                                                                                                     0.1,
//                                                                                                     agents,
//                                                                                                     hotspots,
//                                                                                                     start_time,
//                                                                                                     end_time,
//                                                                                                     graph.cgetRootNode().cgetPosition());
////    if((sim_vec(Eigen::seq(START_SOC_IND, START_SOC_IND+agents.size()-1),Eigen::all).array() < 0).any())
////    {
////      std::cout << "Error: There are negative SOC in plan!!!" << std::endl;
////      for(Eigen::Index sim_ind = 0; sim_ind != sim_vec.cols(); ++sim_ind)
////      {
////        if((sim_vec(Eigen::seq(START_SOC_IND, START_SOC_IND+agents.size()-1),sim_ind).array() < 0).any())
////        {
////          std::cout << sim_vec(TIME_IND, sim_ind) << std::endl;
////          std::cout << (sim_vec(Eigen::seq(START_SOC_IND, START_SOC_IND+agents.size()-1),sim_ind)) << std::endl << std::endl;
////        }
////      }
////    }
//    if((sim_vec(Eigen::seq(START_SOC_IND, START_SOC_IND+agents.size()-1),Eigen::all).array() > agents[0].cgetMaxStateOfCharge()).any())
//    {
//      std::cout << "Error: There are SOC that are greater then the max in plan!!!" << std::endl;
//    }
//  }

  // Send to plotter
  std::cout << "Sending to plotter" << std::endl;
  const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,EIG_OPTIONS> sim_vec =
    plan::planToSimulationVector<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(*plan,
                                                                                      plan_dt,
                                                                                      agents,
                                                                                      hotspots,
                                                                                      start_time,
                                                                                      end_time,
                                                                                      graph.cgetRootNode().cgetPosition(),
                                                                                      graph,
                                                                                      event_points);
  const size_t num_time_steps = sim_vec.cols();

  std::vector<double> max_response_time( num_time_steps, -std::numeric_limits<double>::infinity());
  std::vector<double> avg_response_time( num_time_steps, 0);
  std::vector<double> max_exp_event_rate(num_time_steps, -std::numeric_limits<double>::infinity());
  std::vector<double> avg_exp_event_rate(num_time_steps, 0);
  for(Eigen::Index sim_ind = 0; sim_ind != num_time_steps; ++sim_ind)
  {
    for(size_t event_pnt_ind = 0; event_pnt_ind < event_points.size(); ++event_pnt_ind)
    {
      if(max_response_time[sim_ind] < sim_vec(START_RT_IND + event_pnt_ind, sim_ind))
      {
        max_response_time[sim_ind] = sim_vec(START_RT_IND + event_pnt_ind, sim_ind);
      }
      avg_response_time[sim_ind] += sim_vec(START_RT_IND + event_pnt_ind, sim_ind);
    }
    avg_response_time[sim_ind] /= double(event_points.size());
    for(size_t poi_ind = 0; poi_ind < hotspots.size(); ++poi_ind)
    {
      if(max_exp_event_rate[sim_ind] < sim_vec(START_POI_IND + poi_ind, sim_ind))
      {
        max_exp_event_rate[sim_ind] = sim_vec(START_POI_IND + poi_ind, sim_ind);
      }
      avg_exp_event_rate[sim_ind] += sim_vec(START_POI_IND + poi_ind, sim_ind);
    }
    avg_exp_event_rate[sim_ind] /= double(hotspot_inds.size());
  }

  graph_surveillance_planning_msgs::msg::PlotPlan plan_msg;
  // Resize things
  plan_msg.node_position_x.resize(graph.numberSimplifiedNodes());
  plan_msg.node_position_y.resize(graph.numberSimplifiedNodes());
  plan_msg.node_position_z.resize(graph.numberSimplifiedNodes());
  for(size_t node_it = 0; node_it < graph.numberSimplifiedNodes(); ++node_it)
  {
    plan_msg.node_position_x[node_it] = graph.cgetSimplifiedNodes()[node_it]->cgetPosition()[0] / double(1000);
    plan_msg.node_position_y[node_it] = graph.cgetSimplifiedNodes()[node_it]->cgetPosition()[1] / double(1000);
    plan_msg.node_position_z[node_it] = graph.cgetSimplifiedNodes()[node_it]->cgetPosition()[2] / double(1000);
  }
  plan_msg.from_node_ind.              resize(graph.numberSimplifiedEdges());
  plan_msg.to_node_ind.                resize(graph.numberSimplifiedEdges());
  plan_msg.edge_paths.                 resize(graph.numberSimplifiedEdges());
  plan_msg.poi_position_x.             resize(hotspot_inds.size());
  plan_msg.poi_position_y.             resize(hotspot_inds.size());
  plan_msg.poi_radius.                 resize(hotspot_inds.size());
  plan_msg.poi_max_expected_event_rate.resize(hotspot_inds.size());
  plan_msg.expected_event_rates.       resize(hotspot_inds.size());
  for(size_t poi_it = 0; poi_it < hotspot_inds.size(); ++poi_it)
  {
    plan_msg.expected_event_rates[poi_it].expected_event_rate.resize(num_time_steps);
  }
  plan_msg.rtp_position_x.resize(event_points.size());
  plan_msg.rtp_position_y.resize(event_points.size());
  plan_msg.time.resize(num_time_steps);
  plan_msg.agents_state.resize(agents.size());
  for(size_t agent_it = 0; agent_it < agents.size(); ++agent_it)
  {
    plan_msg.agents_state[agent_it].position_x.     resize(num_time_steps);
    plan_msg.agents_state[agent_it].position_y.     resize(num_time_steps);
    plan_msg.agents_state[agent_it].position_z.     resize(num_time_steps);
    plan_msg.agents_state[agent_it].state_of_charge.resize(num_time_steps);
  }
  plan_msg.metrics.resize(4);
//  plan_msg.metrics.resize(2);
  plan_msg.metrics[2].name = "Average Response Time (hr)";
  plan_msg.metrics[2].max_possible_value = std::numeric_limits<double>::infinity();
  plan_msg.metrics[2].min_possible_value = 0;
  plan_msg.metrics[2].values.resize(num_time_steps);
  plan_msg.metrics[3].name = "Max Response Time (hr)";
  plan_msg.metrics[3].max_possible_value = std::numeric_limits<double>::infinity();
  plan_msg.metrics[3].min_possible_value = 0;
  plan_msg.metrics[3].values.resize(num_time_steps);
  plan_msg.metrics[0].name = "Avg POI Rate";
  plan_msg.metrics[0].max_possible_value = std::numeric_limits<double>::infinity();
  plan_msg.metrics[0].min_possible_value = 0;
  plan_msg.metrics[0].values.resize(num_time_steps);
  plan_msg.metrics[1].name = "Max POI Rate";
  plan_msg.metrics[1].max_possible_value = std::numeric_limits<double>::infinity();
  plan_msg.metrics[1].min_possible_value = 0;
  plan_msg.metrics[1].values.resize(num_time_steps);
  // Copy static things
  for(size_t edge_it = 0; edge_it < graph.numberSimplifiedEdges(); ++edge_it)
  {
    Eigen::Matrix<float,3,Eigen::Dynamic> vec;
    graph.cgetSimplifiedEdges()[edge_it]->generateDiscreetSteps(plan_dt, vec);
    const Eigen::Index vec_len = vec.cols();
    plan_msg.edge_paths[edge_it].position_x.resize(vec_len);
    plan_msg.edge_paths[edge_it].position_y.resize(vec_len);
    plan_msg.edge_paths[edge_it].position_z.resize(vec_len);
    for(Eigen::Index vec_it = 0; vec_it < vec_len; ++vec_it)
    {
      plan_msg.edge_paths[edge_it].position_x[vec_it] = vec(0, vec_it) / double(1000);
      plan_msg.edge_paths[edge_it].position_y[vec_it] = vec(1, vec_it) / double(1000);
      plan_msg.edge_paths[edge_it].position_z[vec_it] = vec(2, vec_it) / double(1000);
    }
    plan_msg.from_node_ind[edge_it] = graph.cgetSimplifiedEdges()[edge_it]->cgetFromNode()->cgetGraphIndex();
    plan_msg.to_node_ind[  edge_it] = graph.cgetSimplifiedEdges()[edge_it]->cgetToNode()->  cgetGraphIndex();
  }
  for(size_t poi_it = 0; poi_it < hotspot_inds.size(); ++poi_it)
  {
    plan_msg.poi_position_x[             poi_it] = graph.cgetOriginalNodes()[hotspot_inds[poi_it]]->cgetPosition()[0] / double(1000);
    plan_msg.poi_position_y[             poi_it] = graph.cgetOriginalNodes()[hotspot_inds[poi_it]]->cgetPosition()[1] / double(1000);
    plan_msg.poi_radius[                 poi_it] = double(50)                                                         / double(1000);
    plan_msg.poi_max_expected_event_rate[poi_it] = hotspots[poi_it].cgetMaxExpectedEventRate();
  }
  for(size_t rtp_ind = 0; rtp_ind < event_points.size(); ++rtp_ind)
  {
    plan_msg.rtp_position_x[rtp_ind] = graph.cgetOriginalNodes()[event_points[rtp_ind]]->cgetPosition()[0] / double(1000);
    plan_msg.rtp_position_y[rtp_ind] = graph.cgetOriginalNodes()[event_points[rtp_ind]]->cgetPosition()[1] / double(1000);
  }
  // Copy states
  for(Eigen::Index sim_ind = 0; sim_ind != num_time_steps; ++sim_ind)
  {
    // Save state
    plan_msg.time[sim_ind] = sim_vec(TIME_IND, sim_ind) / double(60*60);
    for(size_t agent_it = 0; agent_it < agents.size(); ++agent_it)
    {
      const size_t pos_base_ind = START_POS_IND + (3 * agent_it);

      plan_msg.agents_state[agent_it].position_x[     sim_ind] = sim_vec(pos_base_ind,     sim_ind) / double(1000);
      plan_msg.agents_state[agent_it].position_y[     sim_ind] = sim_vec(pos_base_ind + 1, sim_ind) / double(1000);
      plan_msg.agents_state[agent_it].position_z[     sim_ind] = sim_vec(pos_base_ind + 2, sim_ind) / double(1000);
      plan_msg.agents_state[agent_it].state_of_charge[sim_ind] = sim_vec(START_SOC_IND + agent_it, sim_ind);
    }
    for(size_t poi_it = 0; poi_it < hotspot_inds.size(); ++poi_it)
    {
      plan_msg.expected_event_rates[poi_it].expected_event_rate[sim_ind] = sim_vec(START_POI_IND + poi_it, sim_ind);
    }
    plan_msg.metrics[2].values[sim_ind] = avg_response_time[ sim_ind] / double(60*60);
    plan_msg.metrics[3].values[sim_ind] = max_response_time[ sim_ind] / double(60*60);
    plan_msg.metrics[0].values[sim_ind] = avg_exp_event_rate[sim_ind];
    plan_msg.metrics[1].values[sim_ind] = max_exp_event_rate[sim_ind];
  }

  // Send plan out
  std::cout << "Sent to plotter" << std::endl;
  plan_pub->publish(plan_msg);

  rclcpp::spin(node);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}

/* planner_test.cpp */
