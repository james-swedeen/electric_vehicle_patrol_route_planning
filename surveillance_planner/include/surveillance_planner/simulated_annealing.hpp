/**
 * @File: simulated_annealing.hpp
 * @Date: October 2024
 * @Author: James Swedeen
 *
 * @brief
 * Plans paths for agents via the Simulated Annealing algorithm.
 **/

#ifndef SURVEILLANCE_PLANNING_SIMULATED_ANNEALING_PLANNER_HPP
#define SURVEILLANCE_PLANNING_SIMULATED_ANNEALING_PLANNER_HPP

/* C++ Headers */
#include<vector>
#include<memory>
#include<random>
#include<chrono>
#include<execution>
#include<algorithm>
#include<mutex>
#include<iostream> // TODO

/* Boost Headers */
#include<boost/range/irange.hpp>
#include<boost/json/src.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Street Graph Headers */
#include<street_graph/planning_graph.hpp>

/* Local Headers */
#include<surveillance_planner/agent.hpp>
#include<surveillance_planner/helpers.hpp>
#include<surveillance_planner/mutation_operators.hpp>
#include<surveillance_planner/levenshtein_distance.hpp>
#include<surveillance_planner/local_search.hpp>

namespace plan
{
/**
 * @SaFlags
 *
 * @brief
 * Used to control parts of the Simulated Annealing algorithm.
 **/
enum class SaFlags : uint64_t
{
  /**
   * @NULL_FLAG
   *
   * This enumeration explicitly represents nothing.
   **/
  NULL_FLAG = 0x0000'0000'0000'0000,
  /**
   * @FORCE_NO_DUPLICATES
   *
   * If set there will never be any duplicate solutions in the population.
   **/
  FORCE_NO_DUPLICATES = 0x0000'0000'0000'0001,
  /**
   * @USE_LOCAL_SEARCH
   *
   * @brief
   * Set to true to use a local search to hybridized the algorithm.
   **/
  USE_LOCAL_SEARCH = 0x0000'0000'0000'0002,
  /**
   * @USE_ELITISM_SELECTION
   *
   * If set, the best plan from the previous generation will always carry over.
   **/
  USE_ELITISM_SELECTION = 0x1000'0000'0000'0000,
  /**
   * @USE_*_SELECTION
   *
   * Set to use this selection method.
   **/
  SELECTION_BIT_FLAG = 0x0FFF'0000'0000'0000,

  USE_FITNESS_SELECTION                       = 0x0001'0000'0000'0000, // Just carry over the best N plans
  USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION = 0x0002'0000'0000'0000, // Equal probability of all plans being selected
  USE_FITNESS_PROPORTIONAL_SELECTION          = 0x0004'0000'0000'0000, // The probability that a plan is chosen increases as its fitness does
  USE_K_MEANS_SELECTION                       = 0x0008'0000'0000'0000 // From Chehouri, Adam, et al. "A selection process for genetic algorithm using clustering analysis." Algorithms 10.4 (2017): 123.
};
constexpr SaFlags operator |(SaFlags a, SaFlags b)
{
  return static_cast<SaFlags>(static_cast<uint64_t>(a) | static_cast<uint64_t>(b));
}
constexpr SaFlags operator &(SaFlags a, SaFlags b)
{
  return static_cast<SaFlags>(static_cast<uint64_t>(a) & static_cast<uint64_t>(b));
}
/**
 * @test
 *
 * @brief
 * Each is used to test if a given attribute is held in the given configuration.
 *
 * @parameters
 * config: The configuration to test
 *
 * @return
 * True if the attribute asked about is true in the configuration given.
 **/
constexpr bool nullFlag(                                const SaFlags config) noexcept;
constexpr bool noDuplicatesFlag(                        const SaFlags config) noexcept;
constexpr bool localSearchFlag(                         const SaFlags config) noexcept;
constexpr bool elitismSelectionFlag(                    const SaFlags config) noexcept;
constexpr bool fitnessSelectionFlag(                    const SaFlags config) noexcept;
constexpr bool stochasticUniversalSamplingSelectionFlag(const SaFlags config) noexcept;
constexpr bool fitnessProportionalSelectionFlag(        const SaFlags config) noexcept;
constexpr bool kMeansSelectionFlag(                     const SaFlags config) noexcept;
/**
 * @SaParams
 *
 * @brief
 * Holds needed parameters for the Simulated Annealing algorithm.
 *
 * @templates
 * MUTAT_CONFIG: Controls which mutation operators the Firefly algorithm will run
 * EIG_OPTIONS: Eigen storage options
 **/
template<mo::MutationsFlags MUTAT_CONFIG, Eigen::StorageOptions EIG_OPTIONS = Eigen::StorageOptions(Eigen::ColMajor bitor Eigen::AutoAlign)>
struct SaParams
{
public:
  /// General Parameters
  // The size of the population to keep after every generation
  uint32_t population_size;
  // The max dwell time allowed at a hotspot
  uint32_t max_dwell_time_hotspot;
  // The max dwell time allowed at the depot
  uint32_t max_dwell_time_depot;

  /// Simulated Annealing Parameters
  // Initial temperature
  float init_temp;
  // When the temperature goes below this it marks the end of a pass
  float min_temp;
  // Multiplied by the temperature to simulate cooling
  float cooling_rate;
  // How many evaluations to perform between each temperature adjustment
  uint32_t group_number;
  // How many of the best solutions from the previous pass to use in the new population
  uint32_t num_elite;
  // Only solutions that are within this percentage of the best solution found are kept
  double accept_cost_threshold;
  // Max time that one instance of the SA sub function is allowed to run
  std::chrono::duration<int64_t,std::nano> max_sa_time;

  /// Mutation Operator Parameters
  // 0-1 probability of each mutation operator being applied
  Eigen::Matrix<float,mo::numberMutations(MUTAT_CONFIG),1,EIG_OPTIONS,mo::numberMutations(MUTAT_CONFIG),1> mutation_operators_probability;
  // The total possible dwell time change when doing tweakDwellTime mutation
  uint32_t tweak_dwell_time_max_change;

  /// Hybridization Parameters
  // Max time that one instance of the local search is allowed to run
  std::chrono::duration<int64_t,std::nano> max_local_search_time;

  /// For K-Means selection
  uint32_t number_clusters;
  uint32_t number_cluster_iterations;

  inline SaParams(const SaParams&)            noexcept = default;
  inline SaParams(SaParams&&)                 noexcept = default;
  inline SaParams& operator=(const SaParams&) noexcept = default;
  inline SaParams& operator=(SaParams&&)      noexcept = default;
  inline ~SaParams()                          noexcept = default;

  /**
   * @Constructor
   *
   * @brief
   * This constructor initializes the class for use.
   **/
  SaParams() noexcept;
  /**
   * @Json Constructor
   *
   * @brief
   * Treits the given input string as a JSON and auto-fulls the parameters from it.
   *
   * @parameters
   * json: A json string with all parameters in it.
   **/
  SaParams(const std::string& json_str);
};
/**
 * @generateSimulatedAnnealingPlan
 *
 * @brief
 * Uses the Simulated Annealing algorithm to plan.
 *
 * @parameters
 * params: The Simulated Annealing parameters
 * graph: The graph to plan over
 * init_func: This function is responsible for producing the initial population, takes a random seed as arguments
 * cost_func: The objective function, also a boolean that determines if the plan satisfies all constraints
 * stopping_condition: Tells the algorithm when to stop, takes the number of generation executed, the cost of the best plan found, and the best plan found as arguments
 * log_func: This function will be called after every generation on the population, takes the population as arguments
 * special_init_funcs: Each of these functions are used to produce one member of the initial population
 * random_seed: Seed for random action selection
 *
 * @templates
 * CONFIG: Controls how the Simulated Annealing algorithm will run
 * MUTAT_CONFIG: Controls which mutation operators the Firefly algorithm will run
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 *
 * @return
 * Vector with each index being an action.
 **/
template<SaFlags CONFIG, mo::MutationsFlags MUTAT_CONFIG, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS = Eigen::ColMajor bitor Eigen::AutoAlign>
std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  generateSimulatedAnnealingPlan(const SaParams<MUTAT_CONFIG>&                                                                                           params,
                                 const graph::PlanningGraph&                                                                                             graph,
                                 const std::function<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(const unsigned)>&                           init_func,
                                 const std::function<std::pair<bool,double>(const Eigen::Ref<const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>&)>&           cost_func,
                                 const std::function<bool(const uint32_t,const double,const Eigen::Ref<const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>&)>& stopping_condition,
                                 const std::function<void(const Eigen::Matrix<double,Eigen::Dynamic,1>&)>&                                               log_func,
                                 const std::vector<std::function<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>()>>&                            special_init_funcs = std::vector<std::function<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>()>>(),
                                 const unsigned                                                                                                          random_seed = std::chrono::system_clock::now().time_since_epoch().count());
} // plan


constexpr bool plan::nullFlag(const SaFlags config) noexcept
{
  return SaFlags::NULL_FLAG == (config bitand SaFlags::NULL_FLAG);
}

constexpr bool plan::noDuplicatesFlag(const SaFlags config) noexcept
{
  return SaFlags::FORCE_NO_DUPLICATES == (config bitand SaFlags::FORCE_NO_DUPLICATES);
}

constexpr bool plan::localSearchFlag(const SaFlags config) noexcept
{
  return SaFlags::USE_LOCAL_SEARCH == (config bitand SaFlags::USE_LOCAL_SEARCH);
}

constexpr bool plan::elitismSelectionFlag(const SaFlags config) noexcept
{
  return SaFlags::USE_ELITISM_SELECTION == (config bitand SaFlags::USE_ELITISM_SELECTION);
}

constexpr bool plan::fitnessSelectionFlag(const SaFlags config) noexcept
{
  return SaFlags::USE_FITNESS_SELECTION == (config bitand SaFlags::SELECTION_BIT_FLAG);
}

constexpr bool plan::stochasticUniversalSamplingSelectionFlag(const SaFlags config) noexcept
{
  return SaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION == (config bitand SaFlags::SELECTION_BIT_FLAG);
}

constexpr bool plan::fitnessProportionalSelectionFlag(const SaFlags config) noexcept
{
  return SaFlags::USE_FITNESS_PROPORTIONAL_SELECTION == (config bitand SaFlags::SELECTION_BIT_FLAG);
}

constexpr bool plan::kMeansSelectionFlag(const SaFlags config) noexcept
{
  return SaFlags::USE_K_MEANS_SELECTION == (config bitand SaFlags::SELECTION_BIT_FLAG);
}

template<plan::mo::MutationsFlags MUTAT_CONFIG, Eigen::StorageOptions EIG_OPTIONS>
plan::SaParams<MUTAT_CONFIG,EIG_OPTIONS>::SaParams() noexcept
 : population_size(0),
   max_dwell_time_hotspot(0),
   max_dwell_time_depot(0),
   init_temp(std::numeric_limits<float>::quiet_NaN()),
   min_temp(std::numeric_limits<float>::quiet_NaN()),
   cooling_rate(std::numeric_limits<float>::quiet_NaN()),
   group_number(1),
   num_elite(0),
   accept_cost_threshold(1),
   mutation_operators_probability(Eigen::Matrix<float,mo::numberMutations(MUTAT_CONFIG),1,EIG_OPTIONS,mo::numberMutations(MUTAT_CONFIG),1>::Constant(std::numeric_limits<float>::quiet_NaN())),
   tweak_dwell_time_max_change(0),
   max_local_search_time(std::chrono::duration<int64_t,std::nano>::max()),
   number_clusters(0),
   number_cluster_iterations(0)
{}

template<plan::mo::MutationsFlags MUTAT_CONFIG, Eigen::StorageOptions EIG_OPTIONS>
plan::SaParams<MUTAT_CONFIG,EIG_OPTIONS>::SaParams(const std::string& json_str)
{
  // Get rid of any '
  std::string clean_json_str = json_str;
  {
    // Find the first occurrence of the substring
    size_t pos = clean_json_str.find("\'");
    // Iterate through the string and replace all
    // occurrences
    while(pos != std::string::npos)
    {
      // Replace the substring with the specified string
      clean_json_str.replace(pos, 1, "\"");
      // Find the next occurrence of the substring
      pos = clean_json_str.find("\'", pos + 1);
    }
  }

  // Parse the string
  boost::system::error_code  parse_error_code;
  boost::json::parse_options parse_options;
  parse_options.numbers = boost::json::number_precision::precise;

  const boost::json::value json = boost::json::parse(clean_json_str, parse_error_code, boost::json::storage_ptr(), parse_options);
  if(parse_error_code) { throw std::runtime_error("Parsing failed: " + parse_error_code.what()); }

  // Read the values out
  this->population_size        = json.at("population_size").       to_number<uint32_t>();
  this->max_dwell_time_hotspot = json.at("max_dwell_time_hotspot").to_number<uint32_t>();
  this->max_dwell_time_depot   = json.at("max_dwell_time_depot").  to_number<uint32_t>();
  this->init_temp              = json.at("initial_temperature").   to_number<float>();
  this->min_temp               = json.at("min_temperature").       to_number<float>();
  this->cooling_rate           = json.at("cooling_rate").          to_number<float>();
  this->group_number           = json.at("group_number").          to_number<uint32_t>();
  this->num_elite              = json.at("number_elite").          to_number<uint32_t>();
  this->max_sa_time            = std::chrono::duration_cast<std::chrono::duration<int64_t,std::nano>>(std::chrono::duration<double>(json.at("sa_max_time_sec").to_number<double>()));

  if constexpr(addVisitMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[addVisitMutationIndex(MUTAT_CONFIG)] = json.at("mop_add_visit").to_number<float>(); }
  if constexpr(addHotspotVisitMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[addHotspotVisitMutationIndex(MUTAT_CONFIG)] = json.at("mop_add_hotspot_visit").to_number<float>(); }
  if constexpr(addDepotVisitMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[addDepotVisitMutationIndex(MUTAT_CONFIG)] = json.at("mop_add_depot_visit").to_number<float>(); }
  if constexpr(removeVisitMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[removeVisitMutationIndex(MUTAT_CONFIG)] = json.at("mop_remove_visit").to_number<float>(); }
  if constexpr(removeHotspotVisitMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[removeHotspotVisitMutationIndex(MUTAT_CONFIG)] = json.at("mop_remove_hotspot_visit").to_number<float>(); }
  if constexpr(removeDepotVisitMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[removeDepotVisitMutationIndex(MUTAT_CONFIG)] = json.at("mop_remove_depot_visit").to_number<float>(); }
  if constexpr(removeVisitsWindowedMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[removeVisitsWindowedMutationIndex(MUTAT_CONFIG)] = json.at("mop_remove_visits_windowed").to_number<float>(); }
  if constexpr(changeDwellTimeMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[changeDwellTimeMutationIndex(MUTAT_CONFIG)] = json.at("mop_change_dwell_time").to_number<float>(); }
  if constexpr(tweakDwellTimeMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[tweakDwellTimeMutationIndex(MUTAT_CONFIG)] = json.at("mop_tweak_dwell_time").to_number<float>(); }
  if constexpr(swapPathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[swapPathMutationIndex(MUTAT_CONFIG)] = json.at("mop_swap_path").to_number<float>(); }
  if constexpr(swapVisitsMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[swapVisitsMutationIndex(MUTAT_CONFIG)] = json.at("mop_swap_visits").to_number<float>(); }
  if constexpr(swapMultipleVisitsMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[swapMultipleVisitsMutationIndex(MUTAT_CONFIG)] = json.at("mop_swap_multiple_visits").to_number<float>(); }
  if constexpr(moveVisitMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[moveVisitMutationIndex(MUTAT_CONFIG)] = json.at("mop_move_visit").to_number<float>(); }
  if constexpr(simpleInversionMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[simpleInversionMutationIndex(MUTAT_CONFIG)] = json.at("mop_sim").to_number<float>(); }
  if constexpr(addVisitMinTimePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[addVisitMinTimePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_add_visit_min_time_path").to_number<float>(); }
  if constexpr(addHotspotVisitMinTimePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[addHotspotVisitMinTimePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_add_hotspot_visit_min_time_path").to_number<float>(); }
  if constexpr(addDepotVisitMinTimePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[addDepotVisitMinTimePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_add_depot_visit_min_time_path").to_number<float>(); }
  if constexpr(removeVisitMinTimePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[removeVisitMinTimePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_remove_visit_min_time_path").to_number<float>(); }
  if constexpr(removeHotspotVisitMinTimePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[removeHotspotVisitMinTimePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_remove_hotspot_visit_min_time_path").to_number<float>(); }
  if constexpr(removeDepotVisitMinTimePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[removeDepotVisitMinTimePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_remove_depot_visit_min_time_path").to_number<float>(); }
  if constexpr(removeVisitsWindowedMinTimePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[removeVisitsWindowedMinTimePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_remove_visits_windowed_min_time_path").to_number<float>(); }
  if constexpr(swapPathMinTimePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[swapPathMinTimePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_swap_path_min_time_path").to_number<float>(); }
  if constexpr(swapVisitsMinTimePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[swapVisitsMinTimePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_swap_visits_min_time_path").to_number<float>(); }
  if constexpr(swapMultipleVisitsMinTimePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[swapMultipleVisitsMinTimePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_swap_multiple_visits_min_time_path").to_number<float>(); }
  if constexpr(moveVisitMinTimePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[moveVisitMinTimePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_move_visit_min_time_path").to_number<float>(); }
  if constexpr(simpleInversionMinTimePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[simpleInversionMinTimePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_sim_min_time_path").to_number<float>(); }
  if constexpr(addVisitMinChargePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[addVisitMinChargePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_add_visit_min_charge_path").to_number<float>(); }
  if constexpr(addHotspotVisitMinChargePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[addHotspotVisitMinChargePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_add_hotspot_visit_min_charge_path").to_number<float>(); }
  if constexpr(addDepotVisitMinChargePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[addDepotVisitMinChargePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_add_depot_visit_min_charge_path").to_number<float>(); }
  if constexpr(removeVisitMinChargePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[removeVisitMinChargePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_remove_visit_min_charge_path").to_number<float>(); }
  if constexpr(removeHotspotVisitMinChargePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[removeHotspotVisitMinChargePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_remove_hotspot_visit_min_charge_path").to_number<float>(); }
  if constexpr(removeDepotVisitMinChargePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[removeDepotVisitMinChargePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_remove_depot_visit_min_charge_path").to_number<float>(); }
  if constexpr(removeVisitsWindowedMinChargePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[removeVisitsWindowedMinChargePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_remove_visits_windowed_min_charge_path").to_number<float>(); }
  if constexpr(swapPathMinChargePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[swapPathMinChargePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_swap_path_min_charge_path").to_number<float>(); }
  if constexpr(swapVisitsMinChargePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[swapVisitsMinChargePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_swap_visits_min_charge_path").to_number<float>(); }
  if constexpr(swapMultipleVisitsMinChargePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[swapMultipleVisitsMinChargePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_swap_multiple_visits_min_charge_path").to_number<float>(); }
  if constexpr(moveVisitMinChargePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[moveVisitMinChargePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_move_visit_min_charge_path").to_number<float>(); }
  if constexpr(simpleInversionMinChargePathMutationFlag(MUTAT_CONFIG))
  { this->mutation_operators_probability[simpleInversionMinChargePathMutationIndex(MUTAT_CONFIG)] = json.at("mop_sim_min_charge_path").to_number<float>(); }
  
  if constexpr(tweakDwellTimeMutationFlag(MUTAT_CONFIG))
  {
    this->tweak_dwell_time_max_change = json.at("tweak_dwell_time_max_change").to_number<uint32_t>();
  }
  this->max_local_search_time     = std::chrono::duration_cast<std::chrono::duration<int64_t,std::nano>>(std::chrono::duration<double>(json.at("local_search_max_time_sec").to_number<double>()));
  this->number_clusters           = json.at("number_clusters").          to_number<uint32_t>();
  this->number_cluster_iterations = json.at("number_cluster_iterations").to_number<uint32_t>();
}

template<plan::SaFlags CONFIG, plan::mo::MutationsFlags MUTAT_CONFIG, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::generateSimulatedAnnealingPlan(const SaParams<MUTAT_CONFIG>&                                                                                           params,
                                       const graph::PlanningGraph&                                                                                             graph,
                                       const std::function<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(const unsigned)>&                           init_func,
                                       const std::function<std::pair<bool,double>(const Eigen::Ref<const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>&)>&           cost_func,
                                       const std::function<bool(const uint32_t,const double,const Eigen::Ref<const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>&)>& stopping_condition,
                                       const std::function<void(const Eigen::Matrix<double,Eigen::Dynamic,1>&)>&                                               log_func,
                                       const std::vector<std::function<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>()>>&                            special_init_funcs,
                                       const unsigned                                                                                                          random_seed)
{
  // At least one selection and only one
  static_assert(fitnessSelectionFlag(CONFIG) or stochasticUniversalSamplingSelectionFlag(CONFIG) or fitnessProportionalSelectionFlag(CONFIG) or kMeansSelectionFlag(CONFIG));
  if constexpr(fitnessSelectionFlag(CONFIG))
  {
    static_assert(not stochasticUniversalSamplingSelectionFlag(CONFIG));
    static_assert(not fitnessProportionalSelectionFlag(CONFIG));
    static_assert(not kMeansSelectionFlag(CONFIG));
  }
  if constexpr(stochasticUniversalSamplingSelectionFlag(CONFIG))
  {
    static_assert(not fitnessSelectionFlag(CONFIG));
    static_assert(not fitnessProportionalSelectionFlag(CONFIG));
    static_assert(not kMeansSelectionFlag(CONFIG));
  }
  if constexpr(fitnessProportionalSelectionFlag(CONFIG))
  {
    static_assert(not fitnessSelectionFlag(CONFIG));
    static_assert(not stochasticUniversalSamplingSelectionFlag(CONFIG));
    static_assert(not kMeansSelectionFlag(CONFIG));
  }
  if constexpr(kMeansSelectionFlag(CONFIG))
  {
    static_assert(not fitnessSelectionFlag(CONFIG));
    static_assert(not stochasticUniversalSamplingSelectionFlag(CONFIG));
    static_assert(not fitnessProportionalSelectionFlag(CONFIG));
  }


  Eigen::Matrix<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,Eigen::Dynamic,1> population(      params.population_size);
  Eigen::Matrix<double,Eigen::Dynamic,1>                                                  population_costs(params.population_size);
  const boost::integer_range<uint32_t>                                                    population_inds(0, params.population_size);
  const boost::integer_range<uint32_t>                                                    mutation_operator_inds(0, mo::numberMutations(MUTAT_CONFIG));
  const uint32_t                                                                          max_dwell_time = std::max<uint32_t>(params.max_dwell_time_depot, params.max_dwell_time_hotspot);
  std::default_random_engine                                                              rand_gen(random_seed);
  std::uniform_real_distribution<double>                                                  probability_dist(0, 1);
  const boost::integer_range<uint32_t>                                                    elite_inds(  0, params.num_elite);
  const boost::integer_range<uint32_t>                                                    cluster_inds(0, params.number_clusters);

  /// Generate initial population
  const uint32_t num_special_init_funcs = special_init_funcs.size();
  std::thread first_init_thread([&] () -> void
  {
    const boost::integer_range<uint32_t> special_init_funcs_inds(0, num_special_init_funcs);
    std::for_each(std::execution::par_unseq, special_init_funcs_inds.begin(), special_init_funcs_inds.end(),
    [&] (const uint32_t special_init_func_ind) -> void
    {
      population[special_init_func_ind] = special_init_funcs[special_init_func_ind]();

      [[maybe_unused]] bool plan_valid_flag;
      std::tie(plan_valid_flag, population_costs[special_init_func_ind]) = cost_func(*population[special_init_func_ind]);
      assert(plan_valid_flag);
    });
  });
  std::for_each(std::execution::par_unseq, std::next(population_inds.begin(), num_special_init_funcs), population_inds.end(),
  [&] (const uint32_t population_ind) -> void
  {
    population[population_ind] = init_func(random_seed + population_ind + 1);

    [[maybe_unused]] bool plan_valid_flag;
    std::tie(plan_valid_flag, population_costs[population_ind]) = cost_func(*population[population_ind]);
    assert(plan_valid_flag);
  });
  first_init_thread.join();
  if constexpr(noDuplicatesFlag(CONFIG))
  {
    // Remove duplicates
    std::for_each(population_inds.begin(), population_inds.end(),
    [&] (const uint32_t population_ind) -> void
    {
      // While there is a duplicate
      uint32_t attempt_count = 1;
      while(std::any_of(std::execution::unseq, std::next(population_inds.begin(), population_ind + 1), population_inds.end(),
                        [&] (const uint32_t comp_ind) -> bool
                        {
                          return (population[population_ind]->rows() == population[comp_ind]->rows()) and (population[population_ind] == population[comp_ind]);
                        }))
      {
        population[population_ind] = init_func(random_seed + population_ind + 1 + (params.population_size * attempt_count++));

        [[maybe_unused]] bool plan_valid_flag;
        std::tie(plan_valid_flag, population_costs[population_ind]) = cost_func(*population[population_ind]);
        assert(plan_valid_flag);
      }
    });
  }

  /// Bookkeeping
  log_func(population_costs);
  uint32_t best_plan_found_ind = *std::min_element(std::execution::unseq, population_inds.begin(), population_inds.end(),
                                                 [&population_costs] (const uint32_t i, const uint32_t j) -> bool
                                                 { return population_costs[i] < population_costs[j]; });

  for(uint32_t generation_count = 0; not stopping_condition(generation_count, population_costs[best_plan_found_ind], *population[best_plan_found_ind]); ++generation_count)
  {
    // Run Simulated Annealing on every element of the population
    std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
    [&] (const uint32_t population_ind) -> void
    {
      // Run SA
      std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> cur_plan         = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(*population[population_ind]);
      double                                                  cur_cost         = population_costs[population_ind];
      double                                                  best_cost_local  = cur_cost;
      double                                                  accept_threshold = cur_cost * params.accept_cost_threshold;
      const std::chrono::high_resolution_clock::time_point    start_time       = std::chrono::high_resolution_clock::now();
      for(double temperature = params.init_temp; (temperature >= params.min_temp) and (params.max_sa_time >= (std::chrono::high_resolution_clock::now() - start_time)); temperature *= params.cooling_rate)
      {
        for(uint32_t group_count = 0; group_count < params.group_number; ++group_count)
        {
          // Perform random movement
          const uint32_t mutation_ind = weightedRandomSampling<1>(params.mutation_operators_probability, 1, rand_gen)[0];
          std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> mutated_plan =
            mo::applyMutation<MUTAT_CONFIG,MAX_PLAN_VISITS,EIG_OPTIONS>(*cur_plan,
                                                                        mutation_ind,
                                                                        params.max_dwell_time_hotspot,
                                                                        params.max_dwell_time_depot,
                                                                        params.tweak_dwell_time_max_change,
                                                                        graph,
                                                                        rand_gen);
          if(nullptr == mutated_plan.get()) { continue; }
          // Determine if take
          const std::pair<bool,double> cost = cost_func(*mutated_plan);
          if(not cost.first) { continue; }
          if(accept_threshold < cost.second) { continue; }
          const double cost_diff = cost.second - cur_cost;
          if((cost_diff < 0) or (std::exp(-cost_diff / temperature) < probability_dist(rand_gen)))
          {
            cur_plan = std::move(mutated_plan);
            cur_cost = cost.second;
          }
          // Bookkeeping
          if(cost.second < population_costs[population_ind])
          {
            assert(cost_diff < 0);
            *population[     population_ind] = *cur_plan;
            population_costs[population_ind] = cost.second;
            best_cost_local                  = cost.second;
            accept_threshold                 = best_cost_local * params.accept_cost_threshold;
          }
        }
      }
      // Apply local search
      if constexpr(localSearchFlag(CONFIG))
      {
        std::tie(population[population_ind], population_costs[population_ind]) =
          localSearch<MAX_PLAN_VISITS,EIG_OPTIONS>(*population[     population_ind],
                                                   population_costs[population_ind],
                                                   params.max_dwell_time_hotspot,
                                                   params.max_dwell_time_depot,
                                                   graph,
                                                   cost_func,
						   params.max_local_search_time);
      }
    });

    /// Bookkeeping
    log_func(population_costs);

    /// Perform selection
    {
      std::vector<uint32_t> old_population_inds(population_inds.begin(), population_inds.end());

      Eigen::Matrix<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,Eigen::Dynamic,1,EIG_OPTIONS> old_population       = std::move(population);
      Eigen::Matrix<double,                                                 Eigen::Dynamic,1,EIG_OPTIONS> old_population_costs = std::move(population_costs);
      population.      resize(params.population_size);
      population_costs.resize(params.population_size);

      if constexpr(fitnessSelectionFlag(CONFIG))
      {
        std::partial_sort(std::execution::unseq, old_population_inds.begin(), old_population_inds.begin() + params.num_elite, old_population_inds.end(),
                          [&] (const uint32_t i, const uint32_t j) -> bool
                          { return old_population_costs[i] < old_population_costs[j]; });
        std::for_each(std::execution::unseq, elite_inds.begin(), elite_inds.end(),
        [&] (const uint32_t elite_ind) -> void
        {
          population[      elite_ind] = std::move(old_population[      old_population_inds[elite_ind]]);
          population_costs[elite_ind] =           old_population_costs[old_population_inds[elite_ind]];
        });
      }
      if constexpr(stochasticUniversalSamplingSelectionFlag(CONFIG))
      {
        std::vector<uint32_t> new_population_inds;
        new_population_inds.reserve(params.num_elite);
        if constexpr(elitismSelectionFlag(CONFIG))
        {
          auto min_it = std::min_element(std::execution::unseq, old_population_inds.begin(), old_population_inds.end(),
                                         [&] (const uint32_t i, const uint32_t j) -> bool
                                         { return old_population_costs[i] < old_population_costs[j]; });
          assert(min_it != old_population_inds.end());
          std::swap(*min_it, old_population_inds.back());
          new_population_inds.emplace_back(old_population_inds.back());
          old_population_inds.pop_back();
        }

        if(0 < params.num_elite)
        {
          std::sample(old_population_inds.begin(), old_population_inds.end(), std::back_inserter(new_population_inds), params.num_elite - new_population_inds.size(), rand_gen);
          std::for_each(std::execution::unseq, elite_inds.begin(), elite_inds.end(),
          [&] (const uint32_t elite_ind) -> void
          {
            population[      elite_ind] = std::move(old_population[      old_population_inds[elite_ind]]);
            population_costs[elite_ind] =           old_population_costs[old_population_inds[elite_ind]];
          });
        }
      }
      if constexpr(fitnessProportionalSelectionFlag(CONFIG))
      {
        std::vector<uint32_t> new_population_inds;
        new_population_inds.reserve(params.num_elite);
        if constexpr(elitismSelectionFlag(CONFIG))
        {
          auto min_it = std::min_element(std::execution::unseq, old_population_inds.begin(), old_population_inds.end(),
                                         [&] (const uint32_t i, const uint32_t j) -> bool
                                         { return old_population_costs[i] < old_population_costs[j]; });
          assert(min_it != old_population_inds.end());
          std::swap(*min_it, old_population_inds.back());
          new_population_inds.emplace_back(old_population_inds.back());
          old_population_inds.pop_back();
        }

        if(0 < params.num_elite)
        {
          // Find fitness and make is a cumulative probability
          Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> fitness_values = old_population_costs(old_population_inds);
          fitness_values = (double(1) - (fitness_values.array() / fitness_values.template maxCoeff<Eigen::NaNPropagationOptions::PropagateFast>())).eval();

          const uint32_t                                             num_to_select         = std::min<uint32_t>(params.num_elite - new_population_inds.size(), old_population_inds.size());
          const Eigen::Matrix<uint32_t,Eigen::Dynamic,1,EIG_OPTIONS> fitness_selected_inds = weightedRandomSampling<Eigen::Dynamic>(fitness_values, num_to_select, rand_gen);
          for(uint32_t selected_ind = 0; selected_ind < num_to_select; ++selected_ind)
          {
            new_population_inds.emplace_back(old_population_inds[fitness_selected_inds[selected_ind]]);
          }
          // Move selected solutions
          std::for_each(std::execution::unseq, elite_inds.begin(), elite_inds.end(),
          [&] (const uint32_t elite_ind) -> void
          {
            population[      elite_ind] = std::move(old_population[      old_population_inds[elite_ind]]);
            population_costs[elite_ind] =           old_population_costs[old_population_inds[elite_ind]];
          });
        }
      }
      if constexpr(kMeansSelectionFlag(CONFIG))
      {
        std::vector<uint32_t> new_population_inds;

        if((0 < params.num_elite) or elitismSelectionFlag(CONFIG))
        {
          new_population_inds.reserve(params.num_elite);

          /// Find distances between all solutions
          // From ind, then to ind
          Eigen::Matrix<uint32_t,Eigen::Dynamic,Eigen::Dynamic,EIG_OPTIONS> distances(params.population_size, params.population_size);
          std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
          [&] (const uint32_t from_ind) -> void
          {
            distances(from_ind, from_ind) = 0;
            const boost::integer_range<uint32_t> the_rest_of_the_population_inds(from_ind + 1, params.population_size);
            std::for_each(std::execution::par_unseq, the_rest_of_the_population_inds.begin(), the_rest_of_the_population_inds.end(),
            [&] (const uint32_t to_ind) -> void
            {
              // Calculation for one way
              std::tie(distances(from_ind, to_ind), std::ignore) =
                ld::findDistance<MAX_PLAN_VISITS,EIG_OPTIONS>(*old_population[from_ind],
                                                              *old_population[to_ind],
                                                              max_dwell_time);
              // Now the other way
              distances(to_ind, from_ind) = distances(from_ind, to_ind);
            });
          });

          /// Find cluster centers
          Eigen::Matrix<uint32_t,Eigen::Dynamic,1,EIG_OPTIONS> cluster_center_inds(params.number_clusters);
          std::vector<uint32_t> old_population_inds_cluster_center = old_population_inds;
          {
            std::uniform_int_distribution<uint32_t> rand_dist(0, params.population_size - 1);
            cluster_center_inds[0] = rand_dist(rand_gen);
            old_population_inds_cluster_center.erase(std::next(old_population_inds_cluster_center.begin(), cluster_center_inds[0]));
          }
          for(uint32_t cluster_ind = 1; cluster_ind < params.number_clusters; ++cluster_ind)
          {
            const uint32_t                       old_population_inds_size = old_population_inds_cluster_center.size();
            const boost::integer_range<uint32_t> old_population_inds_inds(0, old_population_inds_size);

            Eigen::Matrix<uint32_t,Eigen::Dynamic,1,EIG_OPTIONS> dists_to_closest_cluster_p2(old_population_inds_size);
            std::for_each(std::execution::par_unseq, old_population_inds_inds.begin(), old_population_inds_inds.end(),
            [&] (const uint32_t old_pop_ind_ind) -> void
            {
              const uint32_t pop_ind = old_population_inds_cluster_center[old_pop_ind_ind];
              const auto         min_cluster_ind_ptr = std::min_element(std::execution::unseq, cluster_center_inds.data(), cluster_center_inds.data() + cluster_ind,
                                                                        [&] (const uint32_t i, const uint32_t j) -> bool { return distances(pop_ind, i) < distances(pop_ind, j); });
              assert(min_cluster_ind_ptr != (cluster_center_inds.data() + cluster_ind));
              const uint32_t distance = distances(pop_ind, *min_cluster_ind_ptr);
              dists_to_closest_cluster_p2[old_pop_ind_ind] = distance * distance;
            });

            const uint32_t to_add_ind = weightedRandomSampling<1>(dists_to_closest_cluster_p2, 1, rand_gen)[0];
            // Make it a cluster center and remove it from old population
            cluster_center_inds[cluster_ind] = old_population_inds_cluster_center[to_add_ind];
            old_population_inds_cluster_center.erase(std::next(old_population_inds_cluster_center.cbegin(), to_add_ind));
          }
          /// Perform Clustering
          Eigen::Matrix<std::vector<uint32_t>,Eigen::Dynamic,1,EIG_OPTIONS> cluster_member_inds(    params.number_clusters);
          Eigen::Matrix<std::mutex,Eigen::Dynamic,1,EIG_OPTIONS>            cluster_member_inds_mux(params.number_clusters);
          std::for_each(cluster_inds.begin(), cluster_inds.end(),
          [&] (const uint32_t cluster_ind) -> void
          {
            cluster_member_inds[cluster_ind].reserve(params.population_size);
            cluster_member_inds[cluster_ind].emplace_back(cluster_center_inds[cluster_ind]);
          });
          std::for_each(std::execution::par_unseq, old_population_inds_cluster_center.begin(), old_population_inds_cluster_center.end(),
          [&] (const uint32_t population_ind) -> void
          {
            // Find closest cluster
            Eigen::Index closest_cluster_ind;
            distances(population_ind, cluster_center_inds).template minCoeff<Eigen::NaNPropagationOptions::PropagateFast>(&closest_cluster_ind);
            // Add to that cluster
            {
              std::lock_guard<std::mutex> lock(cluster_member_inds_mux[closest_cluster_ind]);
              cluster_member_inds[closest_cluster_ind].emplace_back(population_ind);
            }
          });
          for(uint32_t clustering_it = 0; clustering_it < params.number_cluster_iterations; ++clustering_it)
          {
            // Find new cluster centers
            std::atomic<uint32_t> num_cluster_centers_moved = 0;
            std::for_each(std::execution::par_unseq, cluster_inds.begin(), cluster_inds.end(),
            [&] (const uint32_t cluster_ind) -> void
            {
              const uint32_t                                       cluster_size = cluster_member_inds[cluster_ind].size();
              const boost::integer_range<uint32_t>                 cluster_member_inds_inds(0, cluster_size);
              Eigen::Matrix<uint32_t,Eigen::Dynamic,1,EIG_OPTIONS> sum_of_dists(cluster_size);
              std::for_each(std::execution::unseq, cluster_member_inds_inds.begin(), cluster_member_inds_inds.end(),
              [&] (const uint32_t member_ind) -> void
              {
                sum_of_dists[member_ind] = distances(member_ind, cluster_member_inds[cluster_ind]).sum();
              });
              // Find new center
              Eigen::Index min_sum_ind;
              sum_of_dists.template minCoeff<Eigen::NaNPropagationOptions::PropagateFast>(&min_sum_ind);
              const uint32_t new_cluster_center = cluster_member_inds[cluster_ind][min_sum_ind];
              if(new_cluster_center != cluster_center_inds[cluster_ind]) { ++num_cluster_centers_moved; }
              cluster_center_inds[cluster_ind] = new_cluster_center;
            });
            if(0 == num_cluster_centers_moved) { break; }
            // Clear old cluster info
            std::for_each(cluster_inds.begin(), cluster_inds.end(),
            [&] (const uint32_t cluster_ind) -> void
            {
              cluster_member_inds[cluster_ind].clear();
              cluster_member_inds[cluster_ind].reserve(params.population_size);
              cluster_member_inds[cluster_ind].emplace_back(cluster_center_inds[cluster_ind]);
            });
            // Assign clusters
            std::for_each(std::execution::par_unseq, old_population_inds.begin(), old_population_inds.end(),
            [&] (const uint32_t population_ind) -> void
            {
              if((cluster_center_inds.array() == population_ind).any()) { return; }
              // Find closest cluster
              Eigen::Index closest_cluster_ind;
              distances(population_ind, cluster_center_inds).template minCoeff<Eigen::NaNPropagationOptions::PropagateFast>(&closest_cluster_ind);
              // Add to that cluster
              {
                std::lock_guard<std::mutex> lock(cluster_member_inds_mux[closest_cluster_ind]);
                cluster_member_inds[closest_cluster_ind].emplace_back(population_ind);
              }
            });
            assert(params.population_size == cluster_member_inds.unaryExpr([] (const std::vector<uint32_t>& i) -> uint32_t { return i.size(); }).sum());
          }
          if constexpr(elitismSelectionFlag(CONFIG))
          {
            std::for_each(cluster_inds.begin(), cluster_inds.end(),
            [&] (const uint32_t cluster_ind) -> void
            {
              auto min_it = std::min_element(std::execution::unseq, cluster_member_inds[cluster_ind].begin(), cluster_member_inds[cluster_ind].end(),
                                             [&] (const uint32_t i, const uint32_t j) -> bool
                                             { return old_population_costs[i] < old_population_costs[j]; });
              assert(min_it != cluster_member_inds[cluster_ind].end());
              std::swap(*min_it, cluster_member_inds[cluster_ind].back());
              new_population_inds.emplace_back(cluster_member_inds[cluster_ind].back());
              cluster_member_inds[cluster_ind].pop_back();
            });
          }
          /// Calculate membership probability index
          Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> membership_probability_index(params.population_size);
          const double inv_population_size = double(1) / double(params.population_size);
          std::for_each(std::execution::par_unseq, cluster_inds.begin(), cluster_inds.end(),
          [&] (const uint32_t cluster_ind) -> void
          {
            uint32_t cluster_size = cluster_member_inds[cluster_ind].size();
            if(1 == cluster_size) { cluster_size = 2; } // To avoid NaNs in the size term
            const double sum_of_fitness    = old_population_costs(cluster_member_inds[cluster_ind]).sum();
            const double size_term         = double(cluster_size) / double(cluster_size - 1);
            const double size_and_pop_term = size_term * inv_population_size;
            std::for_each(std::execution::unseq, cluster_member_inds[cluster_ind].cbegin(), cluster_member_inds[cluster_ind].cend(),
            [&] (const uint32_t member_ind) -> void
            {
              membership_probability_index[member_ind] = size_and_pop_term * ((sum_of_fitness - old_population_costs[member_ind]) / sum_of_fitness);
            });
          });
          /// Perform selection
          // Selection from Efraimidis, Pavlos S., and Paul G. Spirakis. "Weighted random sampling with a reservoir." Information processing letters 97.5 (2006): 181-185.
          const Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> u    = Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS>::NullaryExpr(params.population_size, [&] () { return probability_dist(rand_gen); });
                Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> keys = u.binaryExpr(membership_probability_index, [] (const double ui, const double mpi) -> double { return std::pow(ui, double(1) / mpi); });
          if constexpr(elitismSelectionFlag(CONFIG))
          {
            // Prevent elite samples from getting selected again
            keys(new_population_inds, 0).setConstant(-std::numeric_limits<double>::infinity());
          }
          const uint32_t num_to_sort = std::min<int32_t>(std::max<int32_t>(int32_t(params.num_elite) - int32_t(new_population_inds.size()), 0), old_population_inds.size());
          std::partial_sort(std::execution::unseq, old_population_inds.begin(), old_population_inds.begin() + num_to_sort, old_population_inds.end(),
                            [&] (const uint32_t i, const uint32_t j) -> bool { return keys[i] > keys[j]; });
          std::copy_n(std::execution::unseq, old_population_inds.begin(), num_to_sort, std::back_inserter(new_population_inds));
          assert(params.num_elite == new_population_inds.size());
          // Move selected solutions
          std::for_each(std::execution::unseq, elite_inds.begin(), elite_inds.end(),
          [&] (const uint32_t elite_ind) -> void
          {
            population[      elite_ind] = std::move(old_population[      old_population_inds[elite_ind]]);
            population_costs[elite_ind] =           old_population_costs[old_population_inds[elite_ind]];
          });
        }
      }
    }
    /// Now fill in the rest of the population with random plans
    std::for_each(std::execution::par_unseq, std::next(population_inds.begin(), params.num_elite), population_inds.end(),
    [&] (const uint32_t population_ind) -> void
    {
      population[population_ind] = init_func(random_seed + population_ind + 1 + generation_count);

      [[maybe_unused]] bool plan_valid_flag;
      std::tie(plan_valid_flag, population_costs[population_ind]) = cost_func(*population[population_ind]);
      assert(plan_valid_flag);
    });
    if constexpr(noDuplicatesFlag(CONFIG))
    {
      // Remove duplicates
      std::for_each(std::next(population_inds.begin(), params.num_elite), population_inds.end(),
      [&] (const uint32_t population_ind) -> void
      {
        // While there is a duplicate
        uint32_t attempt_count = 1;
        while(std::any_of(std::execution::unseq, population_inds.begin(), std::next(population_inds.begin(), population_ind),
                          [&] (const uint32_t comp_ind) -> bool
                          {
                            return (population[population_ind]->rows() == population[comp_ind]->rows()) and (*population[population_ind] == *population[comp_ind]);
                          }))
        {
          population[population_ind] = init_func(random_seed + population_ind + 1 + (params.population_size * attempt_count++) + generation_count);

          [[maybe_unused]] bool plan_valid_flag;
          std::tie(plan_valid_flag, population_costs[population_ind]) = cost_func(*population[population_ind]);
          assert(plan_valid_flag);
        }
      });
    }

    assert(params.population_size == population.rows());
    assert(population.unaryExpr([] (const std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& i) -> bool { return i.get() != nullptr; }).all());

    /// Bookkeeping
    best_plan_found_ind = *std::min_element(std::execution::unseq, population_inds.begin(), population_inds.end(),
                                            [&population_costs] (const uint32_t i, const uint32_t j) -> bool
                                            { return population_costs[i] < population_costs[j]; });
  }

  return std::move(population[best_plan_found_ind]);
}

#endif
/* simulated_annealing.hpp */
