/**
 * @File: planner_tuner_server.cpp
 * @Date: January 2025
 * @Author: James Swedeen
 *
 * @brief
 * Simple tuning node for picking the hyper-parameters in the planning algorithms.
 **/

/* C++ Headers */
#include<malloc.h>
#include<mutex>
#include<fstream>
#include<time.h>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<graph_surveillance_planning_msgs/srv/try_firefly_config.hpp>
#include<surveillance_planner/hotspot.hpp>
#include<surveillance_planner/random_planner.hpp>
#include<surveillance_planner/greedy_planner.hpp>
#include<surveillance_planner/metrics.hpp>
#include<surveillance_planner/firefly_planner.hpp>
#include<surveillance_planner/simulated_annealing.hpp>
#include<surveillance_planner/cofield_accent_planner.hpp>

#define USE_LOS_ANGELES

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

  return buf;
}

class FifoMutex
{
public:
  FifoMutex() noexcept
   : m_now_serving(0),
     m_next_available(0)
  {}

  void lock()
  {
    std::unique_lock<std::mutex> lock(this->m_mutex);
    const size_t m_number = this->m_next_available;
    ++this->m_next_available;

    while(m_number != this->m_now_serving)
    {
      this->m_cv.wait(lock);
    }
  }

  void unlock()
  {
    std::lock_guard<std::mutex> lock(this->m_mutex);
    ++this->m_now_serving;
    this->m_cv.notify_one();
  }

private:
  std::mutex m_mutex;
  std::condition_variable m_cv;
  size_t m_now_serving;
  size_t m_next_available;
};

/// Global Parameters
#ifdef USE_LOS_ANGELES
inline static constexpr const double                  end_time            = 48*60*60;
inline static constexpr const size_t                  num_agent_active    = 4;
inline static constexpr const Eigen::Index            NUM_HOTSPOTS        = 25;
inline static constexpr const Eigen::Index            NUM_EVENT_POINTS    = 10;
#endif
inline static constexpr const double                  cost_time_step      = 15.0*60.0;
inline static constexpr const double                  start_time          = 0;
inline static constexpr const double                  shift_length        = 12.0 * 60.0 * 60.0;
inline static constexpr const Eigen::Index            NUM_AGENTS          = std::ceil((end_time - start_time) / (shift_length / double(num_agent_active))) + (int64_t(num_agent_active) - 1);
inline static constexpr const Eigen::Index            MAX_PLAN_VISITS     = Eigen::Dynamic;
inline static constexpr const double                  max_charge          = 85.0*0.8; // max: 85 kWh
inline static constexpr const double                  charging_rate       = 190.0 / double(60*60); // slow: 19.2 kW fast: ~190 kW
inline static constexpr const double                  max_trip_length     = 455.444 * 1000.0; // Meters
inline static constexpr const double                  ecr_two_norm_weight = 1.0e-3;
inline static constexpr const double                  ecr_inf_norm_weight = 10;
inline static constexpr const double                  rt_two_norm_weight  = 1.0e-6;
inline static constexpr const double                  rt_inf_norm_weight  = 1.0e-2;
inline static constexpr const Eigen::StorageOptions   EIG_OPTIONS         = Eigen::StorageOptions(Eigen::ColMajor bitor Eigen::AutoAlign);
inline static constexpr const plan::ConstraintOptions CONSTRAINT_OPTIONS  = plan::ConstraintOptions(plan::ConstraintOptions::USE_SOC_CONSTRAINTS bitor plan::ConstraintOptions::USE_CONSTRAINTS_IN_RESPONSE_TIME_CALCULATION); // bitor plan::ConstraintOptions::USE_DISTANCE_CONSTRAINTS);


// Helper functions
plan::FaParams<plan::mo::MutationsFlags::USE_RECOMMENDATIONS> make_fa_params(const graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::ConstSharedPtr& req)
{
  plan::FaParams<plan::mo::MutationsFlags::USE_RECOMMENDATIONS> fa_params;
  fa_params.population_size             = req->population_size;
  fa_params.max_dwell_time_hotspot      = req->max_dwell_time_hotspot;
  fa_params.max_dwell_time_depot        = req->max_dwell_time_depot;
  fa_params.dist_pow                    = req->distance_power;
  fa_params.dist_mult                   = req->distance_multiplier;
  fa_params.base_attractiveness_mult    = req->base_attractiveness_multiplier;
  fa_params.tweak_dwell_time_max_change = req->tweak_dwell_time_max_change;
  fa_params.local_search_probability    = req->local_search_probability;
  fa_params.max_local_search_time       = std::chrono::duration_cast<std::chrono::duration<int64_t,std::nano>>(std::chrono::duration<double>(req->local_search_max_time_sec));
  fa_params.number_clusters             = req->number_clusters;
  fa_params.number_cluster_iterations   = req->number_cluster_iterations;

  //fa_params.mutation_operators_probability.setConstant(42);

  //fa_params.mutation_operators_probability[addVisitMutationIndex(            plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_visit;
  fa_params.mutation_operators_probability[addHotspotVisitMutationIndex(     plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_hotspot_visit;
  fa_params.mutation_operators_probability[addDepotVisitMutationIndex(       plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_depot_visit;
  //fa_params.mutation_operators_probability[removeVisitMutationIndex(         plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_visit;
  fa_params.mutation_operators_probability[removeHotspotVisitMutationIndex(  plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_hotspot_visit;
  fa_params.mutation_operators_probability[removeDepotVisitMutationIndex(    plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_depot_visit;
  fa_params.mutation_operators_probability[removeVisitsWindowedMutationIndex(plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_visits_windowed;
  //fa_params.mutation_operators_probability[changeDwellTimeMutationIndex(     plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_change_dwell_time;
  fa_params.mutation_operators_probability[tweakDwellTimeMutationIndex(      plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_tweak_dwell_time;
  fa_params.mutation_operators_probability[swapPathMutationIndex(            plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_path;
  fa_params.mutation_operators_probability[swapVisitsMutationIndex(          plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_visits;
  fa_params.mutation_operators_probability[swapMultipleVisitsMutationIndex(  plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_multiple_visits;
  //fa_params.mutation_operators_probability[moveVisitMutationIndex(           plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_move_visit;
  fa_params.mutation_operators_probability[simpleInversionMutationIndex(     plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_sim;

  //fa_params.mutation_operators_probability[addVisitMinTimePathMutationIndex(            plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_visit_min_time_path;
  //fa_params.mutation_operators_probability[addHotspotVisitMinTimePathMutationIndex(     plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_hotspot_visit_min_time_path;
  //fa_params.mutation_operators_probability[addDepotVisitMinTimePathMutationIndex(       plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_depot_visit_min_time_path;
  //fa_params.mutation_operators_probability[removeVisitMinTimePathMutationIndex(         plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_visit_min_time_path;
  //fa_params.mutation_operators_probability[removeHotspotVisitMinTimePathMutationIndex(  plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_hotspot_visit_min_time_path;
  //fa_params.mutation_operators_probability[removeDepotVisitMinTimePathMutationIndex(    plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_depot_visit_min_time_path;
  //fa_params.mutation_operators_probability[removeVisitsWindowedMinTimePathMutationIndex(plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_visits_windowed_min_time_path;
  //fa_params.mutation_operators_probability[swapPathMinTimePathMutationIndex(            plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_path_min_time_path;
  //fa_params.mutation_operators_probability[swapVisitsMinTimePathMutationIndex(          plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_visits_min_time_path;
  //fa_params.mutation_operators_probability[swapMultipleVisitsMinTimePathMutationIndex(  plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_multiple_visits_min_time_path;
  //fa_params.mutation_operators_probability[moveVisitMinTimePathMutationIndex(           plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_move_visit_min_time_path;
  //fa_params.mutation_operators_probability[simpleInversionMinTimePathMutationIndex(     plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_sim_min_time_path;

  //fa_params.mutation_operators_probability[addVisitMinChargePathMutationIndex(            plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_visit_min_charge_path;
  //fa_params.mutation_operators_probability[addHotspotVisitMinChargePathMutationIndex(     plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_hotspot_visit_min_charge_path;
  //fa_params.mutation_operators_probability[addDepotVisitMinChargePathMutationIndex(       plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_depot_visit_min_charge_path;
  //fa_params.mutation_operators_probability[removeVisitMinChargePathMutationIndex(         plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_visit_min_charge_path;
  //fa_params.mutation_operators_probability[removeHotspotVisitMinChargePathMutationIndex(  plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_hotspot_visit_min_charge_path;
  //fa_params.mutation_operators_probability[removeDepotVisitMinChargePathMutationIndex(    plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_depot_visit_min_charge_path;
  //fa_params.mutation_operators_probability[removeVisitsWindowedMinChargePathMutationIndex(plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_visits_windowed_min_charge_path;
  //fa_params.mutation_operators_probability[swapPathMinChargePathMutationIndex(            plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_path_min_charge_path;
  //fa_params.mutation_operators_probability[swapVisitsMinChargePathMutationIndex(          plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_visits_min_charge_path;
  //fa_params.mutation_operators_probability[swapMultipleVisitsMinChargePathMutationIndex(  plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_multiple_visits_min_charge_path;
  //fa_params.mutation_operators_probability[moveVisitMinChargePathMutationIndex(           plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_move_visit_min_charge_path;
  //fa_params.mutation_operators_probability[simpleInversionMinChargePathMutationIndex(     plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_sim_min_charge_path;

  //std::cout << fa_params.mutation_operators_probability << std::endl;

  // Fix any inconsistencies
  if((fa_params.tweak_dwell_time_max_change > fa_params.max_dwell_time_depot) and
     (fa_params.tweak_dwell_time_max_change > fa_params.max_dwell_time_hotspot))
  {
    std::cout << "Warning: tweak_dwell_time_max_change is larger then the max dwell time. Setting params.tweak_dwell_time_max_change = "
              << std::max<uint32_t>(fa_params.max_dwell_time_depot, fa_params.max_dwell_time_hotspot)
              << " instead of "
              << fa_params.tweak_dwell_time_max_change
              << std::endl;
    fa_params.tweak_dwell_time_max_change = std::max<uint32_t>(fa_params.max_dwell_time_depot, fa_params.max_dwell_time_hotspot);
  }
  if((req->use_selection == graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION) and
     (fa_params.population_size < fa_params.number_clusters))
  {
    std::cout << "Warning: number of clusters is larger then population size. Setting params.number_clusters = "
              << fa_params.population_size
              << " instead of "
              << fa_params.number_clusters
              << std::endl;
    fa_params.number_clusters = fa_params.population_size;
  }

  return fa_params;
}

plan::SaParams<plan::mo::MutationsFlags::USE_RECOMMENDATIONS> make_sa_params(const graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::ConstSharedPtr& req)
{
  plan::SaParams<plan::mo::MutationsFlags::USE_RECOMMENDATIONS> fa_params;
  fa_params.population_size             = req->population_size;
  fa_params.max_dwell_time_hotspot      = req->max_dwell_time_hotspot;
  fa_params.max_dwell_time_depot        = req->max_dwell_time_depot;
  fa_params.init_temp                   = req->initial_temperature;
  fa_params.min_temp                    = req->min_temperature;
  fa_params.cooling_rate                = req->cooling_rate;
  fa_params.group_number                = req->group_number;
  fa_params.num_elite                   = req->number_elite;
  fa_params.accept_cost_threshold       = req->accept_cost_threshold;
  fa_params.max_sa_time                 = std::chrono::duration_cast<std::chrono::duration<int64_t,std::nano>>(std::chrono::duration<double>(req->sa_max_time_sec));
  fa_params.tweak_dwell_time_max_change = req->tweak_dwell_time_max_change;
  fa_params.max_local_search_time       = std::chrono::duration_cast<std::chrono::duration<int64_t,std::nano>>(std::chrono::duration<double>(req->local_search_max_time_sec));
  fa_params.number_clusters             = req->number_clusters;
  fa_params.number_cluster_iterations   = req->number_cluster_iterations;

  //fa_params.mutation_operators_probability[addVisitMutationIndex(            plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_visit;
  fa_params.mutation_operators_probability[addHotspotVisitMutationIndex(     plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_hotspot_visit;
  fa_params.mutation_operators_probability[addDepotVisitMutationIndex(       plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_depot_visit;
  //fa_params.mutation_operators_probability[removeVisitMutationIndex(         plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_visit;
  fa_params.mutation_operators_probability[removeHotspotVisitMutationIndex(  plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_hotspot_visit;
  fa_params.mutation_operators_probability[removeDepotVisitMutationIndex(    plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_depot_visit;
  fa_params.mutation_operators_probability[removeVisitsWindowedMutationIndex(plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_visits_windowed;
  //fa_params.mutation_operators_probability[changeDwellTimeMutationIndex(     plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_change_dwell_time;
  fa_params.mutation_operators_probability[tweakDwellTimeMutationIndex(      plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_tweak_dwell_time;
  fa_params.mutation_operators_probability[swapPathMutationIndex(            plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_path;
  fa_params.mutation_operators_probability[swapVisitsMutationIndex(          plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_visits;
  fa_params.mutation_operators_probability[swapMultipleVisitsMutationIndex(  plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_multiple_visits;
  //fa_params.mutation_operators_probability[moveVisitMutationIndex(           plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_move_visit;
  fa_params.mutation_operators_probability[simpleInversionMutationIndex(     plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_sim;

  //fa_params.mutation_operators_probability[addVisitMinTimePathMutationIndex(            plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_visit_min_time_path;
  //fa_params.mutation_operators_probability[addHotspotVisitMinTimePathMutationIndex(     plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_hotspot_visit_min_time_path;
  //fa_params.mutation_operators_probability[addDepotVisitMinTimePathMutationIndex(       plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_depot_visit_min_time_path;
  //fa_params.mutation_operators_probability[removeVisitMinTimePathMutationIndex(         plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_visit_min_time_path;
  //fa_params.mutation_operators_probability[removeHotspotVisitMinTimePathMutationIndex(  plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_hotspot_visit_min_time_path;
  //fa_params.mutation_operators_probability[removeDepotVisitMinTimePathMutationIndex(    plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_depot_visit_min_time_path;
  //fa_params.mutation_operators_probability[removeVisitsWindowedMinTimePathMutationIndex(plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_visits_windowed_min_time_path;
  //fa_params.mutation_operators_probability[swapPathMinTimePathMutationIndex(            plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_path_min_time_path;
  //fa_params.mutation_operators_probability[swapVisitsMinTimePathMutationIndex(          plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_visits_min_time_path;
  //fa_params.mutation_operators_probability[swapMultipleVisitsMinTimePathMutationIndex(  plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_multiple_visits_min_time_path;
  //fa_params.mutation_operators_probability[moveVisitMinTimePathMutationIndex(           plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_move_visit_min_time_path;
  //fa_params.mutation_operators_probability[simpleInversionMinTimePathMutationIndex(     plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_sim_min_time_path;

  //fa_params.mutation_operators_probability[addVisitMinChargePathMutationIndex(            plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_visit_min_charge_path;
  //fa_params.mutation_operators_probability[addHotspotVisitMinChargePathMutationIndex(     plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_hotspot_visit_min_charge_path;
  //fa_params.mutation_operators_probability[addDepotVisitMinChargePathMutationIndex(       plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_add_depot_visit_min_charge_path;
  //fa_params.mutation_operators_probability[removeVisitMinChargePathMutationIndex(         plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_visit_min_charge_path;
  //fa_params.mutation_operators_probability[removeHotspotVisitMinChargePathMutationIndex(  plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_hotspot_visit_min_charge_path;
  //fa_params.mutation_operators_probability[removeDepotVisitMinChargePathMutationIndex(    plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_depot_visit_min_charge_path;
  //fa_params.mutation_operators_probability[removeVisitsWindowedMinChargePathMutationIndex(plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_remove_visits_windowed_min_charge_path;
  //fa_params.mutation_operators_probability[swapPathMinChargePathMutationIndex(            plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_path_min_charge_path;
  //fa_params.mutation_operators_probability[swapVisitsMinChargePathMutationIndex(          plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_visits_min_charge_path;
  //fa_params.mutation_operators_probability[swapMultipleVisitsMinChargePathMutationIndex(  plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_swap_multiple_visits_min_charge_path;
  //fa_params.mutation_operators_probability[moveVisitMinChargePathMutationIndex(           plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_move_visit_min_charge_path;
  //fa_params.mutation_operators_probability[simpleInversionMinChargePathMutationIndex(     plan::mo::MutationsFlags::USE_RECOMMENDATIONS)] = req->mop_sim_min_charge_path;

  // Fix any inconsistencies
  if((fa_params.tweak_dwell_time_max_change > fa_params.max_dwell_time_depot) and
     (fa_params.tweak_dwell_time_max_change > fa_params.max_dwell_time_hotspot))
  {
    std::cout << "Warning: tweak_dwell_time_max_change is larger then the max dwell time. Setting params.tweak_dwell_time_max_change = "
              << std::max<uint32_t>(fa_params.max_dwell_time_depot, fa_params.max_dwell_time_hotspot)
              << " instead of "
              << fa_params.tweak_dwell_time_max_change
              << std::endl;
    fa_params.tweak_dwell_time_max_change = std::max<uint32_t>(fa_params.max_dwell_time_depot, fa_params.max_dwell_time_hotspot);
  }
  if((req->use_selection == graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION) and
     (fa_params.population_size < fa_params.number_clusters))
  {
    std::cout << "Warning: number of clusters is larger then population size. Setting params.number_clusters = "
              << fa_params.population_size
              << " instead of "
              << fa_params.number_clusters
              << std::endl;
    fa_params.number_clusters = fa_params.population_size;
  }
  if(fa_params.num_elite > fa_params.population_size)
  {
    std::cout << "Warning: number of elite samples is larger then population size. Setting params.num_elite = "
              << fa_params.population_size
              << " instead of "
              << fa_params.num_elite
              << std::endl;
    fa_params.num_elite = fa_params.population_size;
  }
  if(fa_params.min_temp > fa_params.init_temp)
  {
    std::cout << "Warning: minimum temperature is larger then starting temperature. Setting params.min_temp = "
              << fa_params.init_temp * 0.1
              << " instead of "
              << fa_params.min_temp
              << std::endl;
    fa_params.min_temp = fa_params.init_temp * 0.1;
  }

  return fa_params;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("planner_tuner_server");

  /// Load problem
  std::cout << "Loading Problem" << std::endl;
  // Generate problem
  const std::string base_dir("~/ros_ws/src/graph_surveillance_planning/street_graph/config/");
#ifdef USE_LOS_ANGELES
  graph::Graph      graph(base_dir+"los_angeles_3500_nodes.csv", base_dir+"los_angeles_3500_edges.csv", 2500);
#endif

  // Make agents
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
  std::vector<uint32_t> hotspot_inds;
  hotspot_inds.reserve(NUM_HOTSPOTS);
  const boost::integer_range<uint32_t> orig_node_inds(0, graph.numberOriginalNodes());
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
  size_t seed = 54;
  do
  {
    std::sample(orig_node_inds.begin(), orig_node_inds.end(), std::back_inserter(hotspot_inds), NUM_HOTSPOTS - hotspot_inds.size(), std::mt19937{seed++});
    std::sort(std::execution::par_unseq, hotspot_inds.begin(), hotspot_inds.end());
    hotspot_inds.erase(std::unique(std::execution::par_unseq, hotspot_inds.begin(), hotspot_inds.end()), hotspot_inds.end());
  }
  while(hotspot_inds.size() < NUM_HOTSPOTS);
  // Make planning graph
  graph::PlanningGraph planning_graph(graph, hotspot_inds, max_charge, shift_length, max_trip_length, plan::socConstraintsFlag(CONSTRAINT_OPTIONS), plan::distanceConstraintsFlag(CONSTRAINT_OPTIONS));
  // Make hotspots
  plan::HOTSPOTS_VEC_TYPE<NUM_HOTSPOTS,EIG_OPTIONS> hotspots;
  for(uint32_t hs_ind = 0; hs_ind < hotspot_inds.size(); ++hs_ind)
  {
    hotspots[hs_ind] = plan::Hotspot(hs_ind + 1, 0.041);
  }
  // Make event points
  plan::EVENT_POINTS_VEC_TYPE<NUM_EVENT_POINTS,EIG_OPTIONS> event_points;
  std::sample(orig_node_inds.begin(), orig_node_inds.end(), event_points.begin(), NUM_EVENT_POINTS, std::mt19937{43});
  // Define objective function
  const auto objective_func = [&] (const Eigen::Ref<const plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& cur_plan) -> std::pair<bool,double>
                              {
                                return plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,true,true,true,true>(cur_plan,
                                                                                    cost_time_step,
                                                                                    start_time,
                                                                                    end_time,
                                                                                    agents,
                                                                                    hotspots,
                                                                                    event_points,
                                                                                    graph,
                                                                                    ecr_two_norm_weight, ecr_inf_norm_weight, rt_two_norm_weight, rt_inf_norm_weight);
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

  /// Start testing
  std::cout << "Ready to Benchmark" << std::endl;
  FifoMutex one_at_a_time_mux;
  rclcpp::Service<graph_surveillance_planning_msgs::srv::TryFireflyConfig>::SharedPtr service =
    node->create_service<graph_surveillance_planning_msgs::srv::TryFireflyConfig>("firefly_config_test",
  [&] (const graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::ConstSharedPtr req,
             graph_surveillance_planning_msgs::srv::TryFireflyConfig::Response::SharedPtr     res) -> void
  {
    std::lock_guard<FifoMutex> lock(one_at_a_time_mux);
    std::cout << "Starting trial:\n";
    std::cout << graph_surveillance_planning_msgs::srv::to_yaml(*req) << std::endl;
    std::ofstream last_start_file("/home/james/ros_ws/last_started_trial.txt", std::ofstream::trunc | std::ofstream::out);
    last_start_file << "Start Time: " << currentDateTime() << "\n\n" << graph_surveillance_planning_msgs::srv::to_yaml(*req) << std::endl;
    last_start_file.flush();
    last_start_file.close();

    std::this_thread::sleep_for(std::chrono::seconds(long(3.0)));

    uint32_t initialization_max_dwell_time = req->initialization_max_dwell_time;

    if(graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::RAND == req->use_firefly_operator)
    {
      const auto plan_start_time = std::chrono::high_resolution_clock::now();
      const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
        plan::generateRandomPlan<NUM_AGENTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(planning_graph,
                                                                                            agents,
                                                                                            initialization_max_dwell_time,
                                                                                            std::chrono::system_clock::now().time_since_epoch().count());
      const auto plan_end_time = std::chrono::high_resolution_clock::now();
      res->time_gen_sec.    emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(plan_end_time - plan_start_time).count()) * double(1.0e-9));
      res->cost_gen.        emplace_back(objective_func(*plan).second);
      res->ecr_two_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,true,false,false,false>(
                                           *plan,
                                           cost_time_step,
                                           start_time,
                                           end_time,
                                           agents,
                                           hotspots,
                                           event_points,
                                           graph,
                                           1, 1, 1, 1).second);
      res->ecr_inf_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,true,false,false>(
                                           *plan,
                                           cost_time_step,
                                           start_time,
                                           end_time,
                                           agents,
                                           hotspots,
                                           event_points,
                                           graph,
                                           1, 1, 1, 1).second);
      res->response_time_two_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,false,true,false>(
                                                     *plan,
                                                     cost_time_step,
                                                     start_time,
                                                     end_time,
                                                     agents,
                                                     hotspots,
                                                     event_points,
                                                     graph,
                                                     1, 1, 1, 1).second);
      res->response_time_inf_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,false,false,true>(
                                                     *plan,
                                                     cost_time_step,
                                                     start_time,
                                                     end_time,
                                                     agents,
                                                     hotspots,
                                                     event_points,
                                                     graph,
                                                     1, 1, 1, 1).second);
      return;
    }
    if(graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::GREEDY == req->use_firefly_operator)
    {
      const auto plan_start_time = std::chrono::high_resolution_clock::now();
      const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
        plan::generateGreedyPlan<NUM_AGENTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(planning_graph,
                                                                                            agents,
                                                                                            initialization_max_dwell_time,
                                                                                            objective_func);
      const auto plan_end_time = std::chrono::high_resolution_clock::now();
      res->time_gen_sec.emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(plan_end_time - plan_start_time).count()) * double(1.0e-9));
      res->cost_gen.    emplace_back(objective_func(*plan).second);
      res->ecr_two_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,true,false,false,false>(
                                           *plan,
                                           cost_time_step,
                                           start_time,
                                           end_time,
                                           agents,
                                           hotspots,
                                           event_points,
                                           graph,
                                           1, 1, 1, 1).second);
      res->ecr_inf_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,true,false,false>(
                                           *plan,
                                           cost_time_step,
                                           start_time,
                                           end_time,
                                           agents,
                                           hotspots,
                                           event_points,
                                           graph,
                                           1, 1, 1, 1).second);
      res->response_time_two_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,false,true,false>(
                                                     *plan,
                                                     cost_time_step,
                                                     start_time,
                                                     end_time,
                                                     agents,
                                                     hotspots,
                                                     event_points,
                                                     graph,
                                                     1, 1, 1, 1).second);
      res->response_time_inf_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,false,false,true>(
                                                     *plan,
                                                     cost_time_step,
                                                     start_time,
                                                     end_time,
                                                     agents,
                                                     hotspots,
                                                     event_points,
                                                     graph,
                                                     1, 1, 1, 1).second);
      return;
    }
    if(graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FIELD_ACCENT == req->use_firefly_operator)
    {
      const auto plan_start_time = std::chrono::high_resolution_clock::now();
      const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
        plan::generateCofieldAccentPlan<NUM_AGENTS,NUM_HOTSPOTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(
                     planning_graph,
                     agents,
                     hotspots,
                     initialization_max_dwell_time,
                     start_time,
                     end_time,
                     req->field_accent_search_weight,
                     req->field_accent_avoid_weight,
                     req->field_accent_width);
      const auto plan_end_time = std::chrono::high_resolution_clock::now();
      res->time_gen_sec.emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(plan_end_time - plan_start_time).count()) * double(1.0e-9));
      res->cost_gen.    emplace_back(objective_func(*plan).second);
      res->ecr_two_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,true,false,false,false>(
                                           *plan,
                                           cost_time_step,
                                           start_time,
                                           end_time,
                                           agents,
                                           hotspots,
                                           event_points,
                                           graph,
                                           1, 1, 1, 1).second);
      res->ecr_inf_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,true,false,false>(
                                           *plan,
                                           cost_time_step,
                                           start_time,
                                           end_time,
                                           agents,
                                           hotspots,
                                           event_points,
                                           graph,
                                           1, 1, 1, 1).second);
      res->response_time_two_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,false,true,false>(
                                                     *plan,
                                                     cost_time_step,
                                                     start_time,
                                                     end_time,
                                                     agents,
                                                     hotspots,
                                                     event_points,
                                                     graph,
                                                     1, 1, 1, 1).second);
      res->response_time_inf_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,false,false,true>(
                                                     *plan,
                                                     cost_time_step,
                                                     start_time,
                                                     end_time,
                                                     agents,
                                                     hotspots,
                                                     event_points,
                                                     graph,
                                                     1, 1, 1, 1).second);
      return;
    }

    // Define initialization function
    const std::function<std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(const unsigned)> init_func =
      [&] (const unsigned rand_seed) -> std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
      {
        return plan::generateRandomPlan<NUM_AGENTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(planning_graph,
                                                                                agents,
                                                                                initialization_max_dwell_time,
                                                                                rand_seed);
      };
    std::vector<std::function<std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>()>> special_init_funcs;
    if(req->use_greedy_in_init)
    {
      special_init_funcs.emplace_back(
        [&] () -> std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
        {
          return plan::generateGreedyPlan<NUM_AGENTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(planning_graph,
                                                                                  agents,
                                                                                  initialization_max_dwell_time,
                                                                                  objective_func);
        });
    }
    if(req->use_field_accent_in_init)
    {
      special_init_funcs.emplace_back(
        [&] () -> std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
        {
          return plan::generateCofieldAccentPlan<NUM_AGENTS,NUM_HOTSPOTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS>(
                     planning_graph,
                     agents,
                     hotspots,
                     initialization_max_dwell_time,
                     start_time,
                     end_time,
                     req->field_accent_search_weight,
                     req->field_accent_avoid_weight,
                     req->field_accent_width);
        });
    }

    // Define stopping condition
    const constexpr size_t reserve_length = 10000;
    res->cost_gen.                  reserve(reserve_length);
    res->time_gen_sec.              reserve(reserve_length);
    res->ecr_two_norm_gen.          reserve(reserve_length);
    res->ecr_inf_norm_gen.          reserve(reserve_length);
    res->response_time_two_norm_gen.reserve(reserve_length);
    res->response_time_inf_norm_gen.reserve(reserve_length);
    auto plan_start_time    = std::chrono::high_resolution_clock::now();
    auto last_func_end_time = std::chrono::high_resolution_clock::now();
    const auto stopping_condition_func =
      [&] (const uint32_t generation_count, const double best_cost_found, const Eigen::Ref<const plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& best_plan_found) -> bool
      {
        const auto func_start_time = std::chrono::high_resolution_clock::now();

        res->cost_gen.emplace_back(best_cost_found);
        if(res->time_gen_sec.empty())
        {
          res->time_gen_sec.emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(func_start_time - plan_start_time).count()) * double(1.0e-9));
        }
        else
        {
          res->time_gen_sec.emplace_back((double(std::chrono::duration_cast<std::chrono::nanoseconds>(func_start_time - last_func_end_time).count()) * double(1.0e-9)) + res->time_gen_sec.back());
        }
        res->ecr_two_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,true,false,false,false>(
                                             best_plan_found,
                                             cost_time_step,
                                             start_time,
                                             end_time,
                                             agents,
                                             hotspots,
                                             event_points,
                                             graph,
                                             1, 1, 1, 1).second);
        res->ecr_inf_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,true,false,false>(
                                             best_plan_found,
                                             cost_time_step,
                                             start_time,
                                             end_time,
                                             agents,
                                             hotspots,
                                             event_points,
                                             graph,
                                             1, 1, 1, 1).second);
        res->response_time_two_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,false,true,false>(
                                                       best_plan_found,
                                                       cost_time_step,
                                                       start_time,
                                                       end_time,
                                                       agents,
                                                       hotspots,
                                                       event_points,
                                                       graph,
                                                       1, 1, 1, 1).second);
        res->response_time_inf_norm_gen.emplace_back(plan::totalCostFunction<NUM_AGENTS,NUM_HOTSPOTS,NUM_EVENT_POINTS,MAX_PLAN_VISITS,CONSTRAINT_OPTIONS,EIG_OPTIONS,false,false,false,true>(
                                                       best_plan_found,
                                                       cost_time_step,
                                                       start_time,
                                                       end_time,
                                                       agents,
                                                       hotspots,
                                                       event_points,
                                                       graph,
                                                       1, 1, 1, 1).second);
        last_func_end_time = std::chrono::high_resolution_clock::now();
        return req->max_solve_time_sec <= res->time_gen_sec.back();
      };
    // Define log function
    const auto logging_func = [] (const Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS>&) -> void {};

    // Run correct version
    if(req->use_local_search)
    {
      switch(req->use_firefly_operator)
      {
        case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::NO_USE_FIREFLY_OPERATOR:
          switch(req->use_selection)
          {
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_SELECTION:
              if(req->force_no_duplicates)
              {
                static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                               plan::FaFlags::USE_FITNESS_SELECTION bitor
                                                                               plan::FaFlags::FORCE_NO_DUPLICATES);
                plan_start_time = std::chrono::high_resolution_clock::now();
                const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                  plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                       planning_graph,
                                                       init_func,
                                                       objective_func,
                                                       stopping_condition_func,
                                                       logging_func,
                                                       special_init_funcs,
                                                       std::chrono::system_clock::now().time_since_epoch().count());
              }
              else
              {
                static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                               plan::FaFlags::USE_FITNESS_SELECTION);
                plan_start_time = std::chrono::high_resolution_clock::now();
                const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                  plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                       planning_graph,
                                                       init_func,
                                                       objective_func,
                                                       stopping_condition_func,
                                                       logging_func,
                                                       special_init_funcs,
                                                       std::chrono::system_clock::now().time_since_epoch().count());
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SUS_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            default:
              throw std::runtime_error("Not a valid case");
              break;
          }
          break;
        case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::NORMAL_FIREFLY_OPERATOR:
          if(req->use_original_luminosity_multiplier)
          {
            switch(req->use_selection)
            {
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_SELECTION:
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SUS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              default:
                throw std::runtime_error("Not a valid case");
                break;
            }
          }
          else // Use other luminosity multiplier
          {
            switch(req->use_selection)
            {
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_SELECTION:
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SUS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              default:
                throw std::runtime_error("Not a valid case");
                break;
            }
          }
          break;
        case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::BIDIRECTIONAL_FIREFLY_OPERATOR:
          if(req->use_original_luminosity_multiplier)
          {
            switch(req->use_selection)
            {
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_SELECTION:
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SUS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              default:
                throw std::runtime_error("Not a valid case");
                break;
            }
          }
          else // Use other luminosity multiplier
          {
            switch(req->use_selection)
            {
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_SELECTION:
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SUS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              default:
                throw std::runtime_error("Not a valid case");
                break;
            }
          }
          break;
        case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SIMULATED_ANNEALING:
          switch(req->use_selection)
          {
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_SELECTION:
              if(req->force_no_duplicates)
              {
                static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_LOCAL_SEARCH bitor
                                                                               plan::SaFlags::USE_FITNESS_SELECTION bitor
                                                                               plan::SaFlags::FORCE_NO_DUPLICATES);
                plan_start_time = std::chrono::high_resolution_clock::now();
                const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                  plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                       planning_graph,
                                                       init_func,
                                                       objective_func,
                                                       stopping_condition_func,
                                                       logging_func,
                                                       special_init_funcs,
                                                       std::chrono::system_clock::now().time_since_epoch().count());
              }
              else
              {
                static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_LOCAL_SEARCH bitor
                                                                               plan::SaFlags::USE_FITNESS_SELECTION);
                plan_start_time = std::chrono::high_resolution_clock::now();
                const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                  plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                       planning_graph,
                                                       init_func,
                                                       objective_func,
                                                       stopping_condition_func,
                                                       logging_func,
                                                       special_init_funcs,
                                                       std::chrono::system_clock::now().time_since_epoch().count());
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SUS_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::SaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::SaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::SaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::SaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::SaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::SaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::SaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::SaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::SaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::SaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::SaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::SaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::SaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::SaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::SaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::SaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::SaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::SaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::SaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::SaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::SaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::SaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::SaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(plan::SaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::SaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            default:
              throw std::runtime_error("Not a valid case");
              break;
          }
          break;
        case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::PATH_RELINKING:
          switch(req->use_selection)
          {
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_SELECTION:
              if(req->force_no_duplicates)
              {
                static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                               plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                               plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                               plan::FaFlags::USE_FITNESS_SELECTION);
                plan_start_time = std::chrono::high_resolution_clock::now();
                const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                  plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                       planning_graph,
                                                       init_func,
                                                       objective_func,
                                                       stopping_condition_func,
                                                       logging_func,
                                                       special_init_funcs,
                                                       std::chrono::system_clock::now().time_since_epoch().count());
              }
              else
              {
                static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                               plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                               plan::FaFlags::USE_FITNESS_SELECTION);
                plan_start_time = std::chrono::high_resolution_clock::now();
                const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                  plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                       planning_graph,
                                                       init_func,
                                                       objective_func,
                                                       stopping_condition_func,
                                                       logging_func,
                                                       special_init_funcs,
                                                       std::chrono::system_clock::now().time_since_epoch().count());
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SUS_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor plan::FaFlags::USE_LOCAL_SEARCH bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            default:
              throw std::runtime_error("Not a valid case");
              break;
          }
          break;
        default:
          throw std::runtime_error("Not a valid case");
          break;
      }
    }
    else // No Local Search
    {
      switch(req->use_firefly_operator)
      {
        case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::NO_USE_FIREFLY_OPERATOR:
          switch(req->use_selection)
          {
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_SELECTION:
              if(req->force_no_duplicates)
              {
                static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                               plan::FaFlags::USE_FITNESS_SELECTION bitor
                                                                               plan::FaFlags::FORCE_NO_DUPLICATES);
                plan_start_time = std::chrono::high_resolution_clock::now();
                const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                  plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                       planning_graph,
                                                       init_func,
                                                       objective_func,
                                                       stopping_condition_func,
                                                       logging_func,
                                                       special_init_funcs,
                                                       std::chrono::system_clock::now().time_since_epoch().count());
              }
              else
              {
                static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                               plan::FaFlags::USE_FITNESS_SELECTION);
                plan_start_time = std::chrono::high_resolution_clock::now();
                const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                  plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                       planning_graph,
                                                       init_func,
                                                       objective_func,
                                                       stopping_condition_func,
                                                       logging_func,
                                                       special_init_funcs,
                                                       std::chrono::system_clock::now().time_since_epoch().count());
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SUS_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            default:
              throw std::runtime_error("Not a valid case");
              break;
          }
          break;
        case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::NORMAL_FIREFLY_OPERATOR:
          if(req->use_original_luminosity_multiplier)
          {
            switch(req->use_selection)
            {
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_SELECTION:
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SUS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              default:
                throw std::runtime_error("Not a valid case");
                break;
            }
          }
          else // Use other luminosity multiplier
          {
            switch(req->use_selection)
            {
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_SELECTION:
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SUS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              default:
                throw std::runtime_error("Not a valid case");
                break;
            }
          }
          break;
        case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::BIDIRECTIONAL_FIREFLY_OPERATOR:
          if(req->use_original_luminosity_multiplier)
          {
            switch(req->use_selection)
            {
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_SELECTION:
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SUS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              default:
                throw std::runtime_error("Not a valid case");
                break;
            }
          }
          else // Use other luminosity multiplier
          {
            switch(req->use_selection)
            {
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_SELECTION:
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FITNESS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SUS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION:
                if(req->use_elitism_selection)
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                else // Don't use elitism selection
                {
                  if(req->force_no_duplicates)
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                  else
                  {
                    static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(
                                                                                   plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                                   plan::FaFlags::USE_K_MEANS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                      plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                           planning_graph,
                                                           init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           special_init_funcs,
                                                           std::chrono::system_clock::now().time_since_epoch().count());
                  }
                }
                break;
              default:
                throw std::runtime_error("Not a valid case");
                break;
            }
          }
          break;
        case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SIMULATED_ANNEALING:
          switch(req->use_selection)
          {
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_SELECTION:
              if(req->force_no_duplicates)
              {
                static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(
                                                                               plan::SaFlags::USE_FITNESS_SELECTION bitor
                                                                               plan::SaFlags::FORCE_NO_DUPLICATES);
                plan_start_time = std::chrono::high_resolution_clock::now();
                const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                  plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                       planning_graph,
                                                       init_func,
                                                       objective_func,
                                                       stopping_condition_func,
                                                       logging_func,
                                                       special_init_funcs,
                                                       std::chrono::system_clock::now().time_since_epoch().count());
              }
              else
              {
                static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(
                                                                               plan::SaFlags::USE_FITNESS_SELECTION);
                plan_start_time = std::chrono::high_resolution_clock::now();
                const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                  plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                       planning_graph,
                                                       init_func,
                                                       objective_func,
                                                       stopping_condition_func,
                                                       logging_func,
                                                       special_init_funcs,
                                                       std::chrono::system_clock::now().time_since_epoch().count());
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SUS_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(
                                                                                 plan::SaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::SaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::SaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(
                                                                                 plan::SaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::SaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(
                                                                                 plan::SaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::SaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(
                                                                                 plan::SaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(
                                                                                 plan::SaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::SaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::SaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(
                                                                                 plan::SaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::SaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(
                                                                                 plan::SaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::SaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(
                                                                                 plan::SaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(
                                                                                 plan::SaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::SaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::SaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(
                                                                                 plan::SaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::SaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(
                                                                                 plan::SaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::SaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::SaFlags SA_CONFIG = plan::SaFlags(
                                                                                 plan::SaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateSimulatedAnnealingPlan<SA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_sa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            default:
              throw std::runtime_error("Not a valid case");
              break;
          }
          break;
        case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::PATH_RELINKING:
          switch(req->use_selection)
          {
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_SELECTION:
              if(req->force_no_duplicates)
              {
                static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor
                                                                               plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                               plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                               plan::FaFlags::USE_FITNESS_SELECTION);
                plan_start_time = std::chrono::high_resolution_clock::now();
                const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                  plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                       planning_graph,
                                                       init_func,
                                                       objective_func,
                                                       stopping_condition_func,
                                                       logging_func,
                                                       special_init_funcs,
                                                       std::chrono::system_clock::now().time_since_epoch().count());
              }
              else
              {
                static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor
                                                                               plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                               plan::FaFlags::USE_FITNESS_SELECTION);
                plan_start_time = std::chrono::high_resolution_clock::now();
                const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                  plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                       planning_graph,
                                                       init_func,
                                                       objective_func,
                                                       stopping_condition_func,
                                                       logging_func,
                                                       special_init_funcs,
                                                       std::chrono::system_clock::now().time_since_epoch().count());
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::SUS_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyConfig::Request::K_MEANS_SELECTION:
              if(req->use_elitism_selection)
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_ELITISM_SELECTION bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              else // Don't use elitism selection
              {
                if(req->force_no_duplicates)
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::FORCE_NO_DUPLICATES bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
                else
                {
                  static constexpr const plan::FaFlags FA_CONFIG = plan::FaFlags(plan::FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_FIREFLY_OPERATOR bitor
                                                                                 plan::FaFlags::USE_K_MEANS_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> plan =
                    plan::generateFireflyPlan<FA_CONFIG,plan::mo::MutationsFlags::USE_RECOMMENDATIONS,MAX_PLAN_VISITS,EIG_OPTIONS>(make_fa_params(req),
                                                         planning_graph,
                                                         init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         special_init_funcs,
                                                         std::chrono::system_clock::now().time_since_epoch().count());
                }
              }
              break;
            default:
              throw std::runtime_error("Not a valid case");
              break;
          }
          break;
        default:
          throw std::runtime_error("Not a valid case");
          break;
      }
    }
    malloc_trim(0);
  });


  rclcpp::spin(node);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}

/* planner_tuner_server.cpp */
