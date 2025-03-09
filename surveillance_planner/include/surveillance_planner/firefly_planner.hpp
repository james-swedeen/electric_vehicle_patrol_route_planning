/**
 * @File: firefly_planner.hpp
 * @Date: June 2024
 * @Author: James Swedeen
 *
 * @brief
 * Plans paths for agents via the Firefly algorithm.
 **/

#ifndef SURVEILLANCE_PLANNING_FIREFLY_PLANNER_HPP
#define SURVEILLANCE_PLANNING_FIREFLY_PLANNER_HPP

/* C++ Headers */
#include<string>
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
#include<surveillance_planner/action.hpp>
#include<surveillance_planner/helpers.hpp>
#include<surveillance_planner/levenshtein_distance.hpp>
#include<surveillance_planner/mutation_operators.hpp>
#include<surveillance_planner/local_search.hpp>

namespace plan
{
/**
 * @FaFlags
 *
 * @brief
 * Used to control parts of the Firefly algorithm.
 **/
enum class FaFlags : uint64_t
{
  /**
   * @NULL_FLAG
   *
   * This enumeration explicitly represents nothing.
   **/
  NULL_FLAG = 0x0000'0000'0000'0000,
  /**
   * @USE_FIREFLY_OPERATOR
   *
   * Set to use the Firefly, steer to better solutions, operator.
   **/
  USE_FIREFLY_OPERATOR = 0x0000'0000'0000'0001,
  /**
   * @USE_BIDIRECTIONAL_FIREFLY_OPERATOR
   *
   * Set to use the Firefly, steer to better solutions, operator in bidirectional mode.
   **/
  USE_BIDIRECTIONAL_FIREFLY_OPERATOR = 0x0000'0000'0000'0002,
  /**
   * @USE_PATH_RELINKING_FIREFLY_OPERATOR
   *
   * Set to use the Firefly, steer to better solutions, operator in bidirectional mode.
   **/
  USE_PATH_RELINKING_FIREFLY_OPERATOR = 0x0000'0000'0000'0004,
  /**
   * @USE_DEFAULT_LUMINOSITY_MULTIPLAYER
   *
   * Set to use the default luminosity multiplayer e^{-gamma r^m} instead of the flatter option 1/(1+gamma r^m).
   **/
  USE_DEFAULT_LUMINOSITY_MULTIPLAYER = 0x0000'0000'0000'0008,
  /**
   * @FORCE_NO_DUPLICATES
   *
   * If set there will never be any duplicate solutions in the population.
   **/
  FORCE_NO_DUPLICATES = 0x0000'0000'0000'0010,
  /**
   * @USE_LOCAL_SEARCH
   *
   * @brief
   * Set to true to use a local search to hybridized the algorithm.
   **/
  USE_LOCAL_SEARCH = 0x0000'0000'0000'0020,
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
constexpr FaFlags operator |(FaFlags a, FaFlags b)
{
  return static_cast<FaFlags>(static_cast<uint64_t>(a) | static_cast<uint64_t>(b));
}
constexpr FaFlags operator &(FaFlags a, FaFlags b)
{
  return static_cast<FaFlags>(static_cast<uint64_t>(a) & static_cast<uint64_t>(b));
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
constexpr bool nullFlag(                                const FaFlags config) noexcept;
constexpr bool fireflyOperatorFlag(                     const FaFlags config) noexcept;
constexpr bool bidirectionalFireflyOperatorFlag(        const FaFlags config) noexcept;
constexpr bool pathRelinkingFireflyOperatorFlag(        const FaFlags config) noexcept;
constexpr bool defaultLuminosityMultiplayerFlag(        const FaFlags config) noexcept;
constexpr bool noDuplicatesFlag(                        const FaFlags config) noexcept;
constexpr bool localSearchFlag(                         const FaFlags config) noexcept;
constexpr bool elitismSelectionFlag(                    const FaFlags config) noexcept;
constexpr bool fitnessSelectionFlag(                    const FaFlags config) noexcept;
constexpr bool stochasticUniversalSamplingSelectionFlag(const FaFlags config) noexcept;
constexpr bool fitnessProportionalSelectionFlag(        const FaFlags config) noexcept;
constexpr bool kMeansSelectionFlag(                     const FaFlags config) noexcept;
/**
 * @FaParams
 *
 * @brief
 * Holds needed parameters for the Firefly algorithm.
 *
 * @templates
 * MUTAT_CONFIG: Controls which mutation operators the Firefly algorithm will run
 * EIG_OPTIONS: Eigen storage options
 **/
template<mo::MutationsFlags MUTAT_CONFIG, Eigen::StorageOptions EIG_OPTIONS = Eigen::StorageOptions(Eigen::ColMajor bitor Eigen::AutoAlign)>
struct FaParams
{
public:
  /// General Parameters
  // The size of the population to keep after every generation
  uint32_t population_size;
  // The max dwell time allowed at a hotspot
  uint32_t max_dwell_time_hotspot;
  // The max dwell time allowed at the depot
  uint32_t max_dwell_time_depot;

  /// Firefly Parameters
  // The value that solution distances are raised to the power of in attractiveness calculation
  float dist_pow;
  // The value that solution distances are multiplied by in attractiveness calculation
  float dist_mult;
  // Base attractiveness multiplayer
  float base_attractiveness_mult;

  /// Mutation Operator Parameters
  // 0-1 probability of each mutation operator being applied
  Eigen::Matrix<float,mo::numberMutations(MUTAT_CONFIG),1,EIG_OPTIONS,mo::numberMutations(MUTAT_CONFIG),1> mutation_operators_probability;
  // The total possible dwell time change when doing tweakDwellTime mutation
  uint32_t tweak_dwell_time_max_change;

  /// Hybridization Parameters
  // The probability of using a local search on each element of the population, each generation
  float local_search_probability;
  // Max time that one instance of the local search is allowed to run
  std::chrono::duration<int64_t,std::nano> max_local_search_time;

  /// For K-Means selection
  uint32_t number_clusters;
  uint32_t number_cluster_iterations;

  inline FaParams(const FaParams&)            noexcept = default;
  inline FaParams(FaParams&&)                 noexcept = default;
  inline FaParams& operator=(const FaParams&) noexcept = default;
  inline FaParams& operator=(FaParams&&)      noexcept = default;
  inline ~FaParams()                          noexcept = default;

  /**
   * @Constructor
   *
   * @brief
   * This constructor initializes the class for use.
   **/
  FaParams() noexcept;
  /**
   * @Json Constructor
   *
   * @brief
   * Treits the given input string as a JSON and auto-fulls the parameters from it.
   *
   * @parameters
   * json: A json string with all parameters in it.
   **/
  FaParams(const std::string& json_str);
};
/**
 * @generateFireflyPlan
 *
 * @brief
 * Uses the Firefly algorithm to plan.
 *
 * @parameters
 * params: The Firefly parameters
 * graph: The graph to plan over
 * init_func: This function is responsible for producing the initial population, takes a random seed as arguments
 * cost_func: The objective function, also a boolean that determines if the plan satisfies all constraints
 * stopping_condition: Tells the algorithm when to stop, takes the number of generation executed, the cost of the best plan found, and the best plan found as arguments
 * log_func: This function will be called after every generation on the population, takes the population costs as arguments
 * special_init_funcs: Each of these functions are used to produce one member of the initial population
 * random_seed: Seed for random action selection
 *
 * @templates
 * CONFIG: Controls how the Firefly algorithm will run
 * MUTAT_CONFIG: Controls which mutation operators the Firefly algorithm will run
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 *
 * @return
 * Vector with each index being an action.
 **/
template<FaFlags CONFIG, mo::MutationsFlags MUTAT_CONFIG, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS = Eigen::ColMajor bitor Eigen::AutoAlign>
std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  generateFireflyPlan(const FaParams<MUTAT_CONFIG>&                                                                                           params,
                      const graph::PlanningGraph&                                                                                             graph,
                      const std::function<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(const unsigned)>&                           init_func,
                      const std::function<std::pair<bool,double>(const Eigen::Ref<const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>&)>&           cost_func,
                      const std::function<bool(const uint32_t,const double,const Eigen::Ref<const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>&)>& stopping_condition,
                      const std::function<void(const Eigen::Matrix<double,Eigen::Dynamic,1>&)>&                                               log_func,
                      const std::vector<std::function<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>()>>&                            special_init_funcs = std::vector<std::function<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>()>>(),
                      const unsigned                                                                                                          random_seed = std::chrono::system_clock::now().time_since_epoch().count());
} // plan


constexpr bool plan::nullFlag(const FaFlags config) noexcept
{
  return FaFlags::NULL_FLAG == (config bitand FaFlags::NULL_FLAG);
}

constexpr bool plan::fireflyOperatorFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_FIREFLY_OPERATOR == (config bitand FaFlags::USE_FIREFLY_OPERATOR);
}

constexpr bool plan::bidirectionalFireflyOperatorFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR == (config bitand FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR);
}

constexpr bool plan::pathRelinkingFireflyOperatorFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR == (config bitand FaFlags::USE_PATH_RELINKING_FIREFLY_OPERATOR);
}

constexpr bool plan::defaultLuminosityMultiplayerFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER == (config bitand FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
}

constexpr bool plan::noDuplicatesFlag(const FaFlags config) noexcept
{
  return FaFlags::FORCE_NO_DUPLICATES == (config bitand FaFlags::FORCE_NO_DUPLICATES);
}

constexpr bool plan::localSearchFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_LOCAL_SEARCH == (config bitand FaFlags::USE_LOCAL_SEARCH);
}

constexpr bool plan::elitismSelectionFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_ELITISM_SELECTION == (config bitand FaFlags::USE_ELITISM_SELECTION);
}

constexpr bool plan::fitnessSelectionFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_FITNESS_SELECTION == (config bitand FaFlags::SELECTION_BIT_FLAG);
}

constexpr bool plan::stochasticUniversalSamplingSelectionFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION == (config bitand FaFlags::SELECTION_BIT_FLAG);
}

constexpr bool plan::fitnessProportionalSelectionFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION == (config bitand FaFlags::SELECTION_BIT_FLAG);
}

constexpr bool plan::kMeansSelectionFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_K_MEANS_SELECTION == (config bitand FaFlags::SELECTION_BIT_FLAG);
}

template<plan::mo::MutationsFlags MUTAT_CONFIG, Eigen::StorageOptions EIG_OPTIONS>
plan::FaParams<MUTAT_CONFIG,EIG_OPTIONS>::FaParams() noexcept
 : population_size(0),
   max_dwell_time_hotspot(0),
   max_dwell_time_depot(0),
   dist_pow(std::numeric_limits<float>::quiet_NaN()),
   dist_mult(std::numeric_limits<float>::quiet_NaN()),
   mutation_operators_probability(Eigen::Matrix<float,mo::numberMutations(MUTAT_CONFIG),1,EIG_OPTIONS,mo::numberMutations(MUTAT_CONFIG),1>::Constant(std::numeric_limits<float>::quiet_NaN())),
   tweak_dwell_time_max_change(0),
   local_search_probability(std::numeric_limits<float>::quiet_NaN()),
   max_local_search_time(std::chrono::duration<int64_t,std::nano>::max()),
   number_clusters(0),
   number_cluster_iterations(0)
{}

template<plan::mo::MutationsFlags MUTAT_CONFIG, Eigen::StorageOptions EIG_OPTIONS>
plan::FaParams<MUTAT_CONFIG,EIG_OPTIONS>::FaParams(const std::string& json_str)
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
  this->population_size          = json.at("population_size").               to_number<uint32_t>();
  this->max_dwell_time_hotspot   = json.at("max_dwell_time_hotspot").        to_number<uint32_t>();
  this->max_dwell_time_depot     = json.at("max_dwell_time_depot").          to_number<uint32_t>();
  this->dist_pow                 = json.at("distance_power").                to_number<float>();
  this->dist_mult                = json.at("distance_multiplier").           to_number<float>();
  this->base_attractiveness_mult = json.at("base_attractiveness_multiplier").to_number<float>();

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
  this->local_search_probability  = json.at("local_search_probability").to_number<float>();
  this->max_local_search_time     = std::chrono::duration_cast<std::chrono::duration<int64_t,std::nano>>(std::chrono::duration<double>(json.at("local_search_max_time_sec").to_number<double>()));
  this->number_clusters           = json.at("number_clusters").          to_number<uint32_t>();
  this->number_cluster_iterations = json.at("number_cluster_iterations").to_number<uint32_t>();
}

template<plan::FaFlags CONFIG, plan::mo::MutationsFlags MUTAT_CONFIG, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::generateFireflyPlan(const FaParams<MUTAT_CONFIG>&                                                                                           params,
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
    assert(params.population_size >= params.number_clusters);
    static_assert(not fitnessSelectionFlag(CONFIG));
    static_assert(not stochasticUniversalSamplingSelectionFlag(CONFIG));
    static_assert(not fitnessProportionalSelectionFlag(CONFIG));
  }
  if constexpr(bidirectionalFireflyOperatorFlag(CONFIG))
  {
    static_assert(fireflyOperatorFlag(CONFIG));
    static_assert(not pathRelinkingFireflyOperatorFlag(CONFIG));
  }
  if constexpr(pathRelinkingFireflyOperatorFlag(CONFIG))
  {
    static_assert(fireflyOperatorFlag(CONFIG));
    static_assert(not bidirectionalFireflyOperatorFlag(CONFIG));
  }

  // Helper variables
  Eigen::Matrix<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,Eigen::Dynamic,1,EIG_OPTIONS> population(      params.population_size);
  Eigen::Matrix<double,                                                 Eigen::Dynamic,1,EIG_OPTIONS> population_costs(params.population_size);
  const boost::integer_range<uint32_t>                                                                population_inds(0, params.population_size);
  const boost::integer_range<uint32_t>                                                                mutation_operator_inds(0, mo::numberMutations(MUTAT_CONFIG));
  std::default_random_engine                                                                          rand_gen(random_seed);
  std::uniform_real_distribution<double>                                                              probability_dist(0, 1);
  const uint32_t                                                                                      max_dwell_time = std::max<uint32_t>(params.max_dwell_time_depot, params.max_dwell_time_hotspot);
        uint32_t                                                                                      next_gen_max_size = params.population_size * (mo::numberMutations(MUTAT_CONFIG) + uint32_t(localSearchFlag(CONFIG)));
  if constexpr(fireflyOperatorFlag(CONFIG))
  {
    next_gen_max_size += ((params.population_size * (params.population_size + uint32_t(1))) / uint32_t(2));
  }
  if constexpr(bidirectionalFireflyOperatorFlag(CONFIG) or pathRelinkingFireflyOperatorFlag(CONFIG))
  {
    next_gen_max_size += ((params.population_size * (params.population_size + uint32_t(1))) / uint32_t(2));
  }

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
                          return (population[population_ind]->rows() == population[comp_ind]->rows()) and (*population[population_ind] == *population[comp_ind]);
                        }))
      {
        population[population_ind] = init_func(random_seed + population_ind + 1 + (params.population_size * attempt_count++));

        [[maybe_unused]] bool plan_valid_flag;
        std::tie(plan_valid_flag, population_costs[population_ind]) = cost_func(*population[population_ind]);
        assert(plan_valid_flag);
      }
    });
  }

  /// Find distances between all solutions
  // From ind, then to ind
  Eigen::Matrix<uint32_t,Eigen::Dynamic,Eigen::Dynamic,EIG_OPTIONS>                                                         distances;
  Eigen::Matrix<std::unique_ptr<ld::ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,Eigen::Dynamic,Eigen::Dynamic,EIG_OPTIONS> edit_scripts;
  if constexpr(fireflyOperatorFlag(CONFIG))
  {
    distances.   resize(params.population_size, params.population_size);
    edit_scripts.resize(params.population_size, params.population_size);
  }
  if constexpr(kMeansSelectionFlag(CONFIG))
  {
    distances.resize(params.population_size, params.population_size);
    std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
    [&] (const uint32_t from_ind) -> void
    {
      distances(from_ind, from_ind) = 0;
      const boost::integer_range<uint32_t> the_rest_of_the_population_inds(from_ind + 1, params.population_size);
      std::for_each(std::execution::par_unseq, the_rest_of_the_population_inds.begin(), the_rest_of_the_population_inds.end(),
      [&] (const uint32_t to_ind) -> void
      {
        // Calculation for one way
        std::tie(distances(from_ind, to_ind), edit_scripts(from_ind, to_ind)) =
          ld::findDistance<MAX_PLAN_VISITS,EIG_OPTIONS>(*population[from_ind],
                                                        *population[to_ind],
                                                        max_dwell_time);
        #ifndef NDEBUG
          std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> temp = ld::performMovement<MAX_PLAN_VISITS,EIG_OPTIONS,std::default_random_engine>(*edit_scripts(from_ind, to_ind), *population[from_ind], std::numeric_limits<double>::infinity(), rand_gen).first;
          assert((temp->rows() == population[to_ind]->rows()) and (*temp == *population[to_ind]));
        #endif
        // Now the other way
        distances(to_ind, from_ind) = distances(from_ind, to_ind);
        if constexpr(fireflyOperatorFlag(CONFIG))
        {
          edit_scripts(to_ind, from_ind) = ld::reverseAlignment<MAX_PLAN_VISITS,EIG_OPTIONS>(*edit_scripts(from_ind, to_ind));
          #ifndef NDEBUG
            std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> temp = ld::performMovement<MAX_PLAN_VISITS,EIG_OPTIONS,std::default_random_engine>(*edit_scripts(to_ind, from_ind), *population[to_ind], std::numeric_limits<double>::infinity(), rand_gen).first;
            assert((temp->rows() == population[from_ind]->rows()) and (*temp == *population[from_ind]));
          #endif
        }
      });
    });
    if constexpr(fireflyOperatorFlag(CONFIG))
    {
      assert((params.population_size == edit_scripts.unaryExpr([] (const std::unique_ptr<ld::ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& es) -> bool { return not es; }).count()));
    }
  }
  /// Bookkeeping
  log_func(population_costs);
  uint32_t best_sol_found_ind, worst_sol_found_ind;
  {
    const auto best_worst_ind_its = std::minmax_element(std::execution::unseq, population_inds.begin(), population_inds.end(),
                                                        [&population_costs] (const uint32_t i, const uint32_t j) -> bool
                                                        { return population_costs[i] < population_costs[j]; });
    best_sol_found_ind  = *best_worst_ind_its.first;
    worst_sol_found_ind = *best_worst_ind_its.second;
  }

  for(uint32_t generation_count = 0; not stopping_condition(generation_count, population_costs[best_sol_found_ind], *population[best_sol_found_ind]); ++generation_count)
  {
    Eigen::Matrix<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,Eigen::Dynamic,1,EIG_OPTIONS> next_generation(      next_gen_max_size);
    Eigen::Matrix<double,                                                 Eigen::Dynamic,1,EIG_OPTIONS> next_generation_costs(next_gen_max_size);
    uint32_t                                                                                            next_generation_len = 0;
    std::mutex                                                                                          next_generation_mux;
    /// Apply hybridization
    std::thread hybridization_thread;
    Eigen::Array<bool,Eigen::Dynamic,1,EIG_OPTIONS,Eigen::Dynamic,1> apply_local_search;
    if constexpr(localSearchFlag(CONFIG))
    {
      apply_local_search = (Eigen::Array<float,Eigen::Dynamic,1,EIG_OPTIONS,Eigen::Dynamic,1>::NullaryExpr(params.population_size, [&] () { return probability_dist(rand_gen); }) < params.local_search_probability);
      hybridization_thread = std::thread([&] () -> void
      {
        std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
        [&] (const uint32_t population_ind) -> void
        {
          if(not apply_local_search[population_ind]) { return; }
          // Perform local search
          std::pair<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,double> new_plan_info =
            localSearch<MAX_PLAN_VISITS,EIG_OPTIONS>(*population[     population_ind],
                                                     population_costs[population_ind],
                                                     params.max_dwell_time_hotspot,
                                                     params.max_dwell_time_depot,
                                                     graph,
                                                     cost_func,
						     params.max_local_search_time);

          if constexpr(noDuplicatesFlag(CONFIG))
          {
            // If plan unique in population
            if(not std::none_of(population_inds.begin(), population_inds.end(),
                                [&] (const uint32_t comp_ind) -> bool
                                {
                                  return (new_plan_info.first->rows() == population[comp_ind]->rows()) and (*new_plan_info.first == *population[comp_ind]);
                                }))
            {
              return;
            }
          }
	  if(population_costs[population_ind] != new_plan_info.second)
          {
            // Add plan to next generation
            std::lock_guard<std::mutex> lock(next_generation_mux);
            next_generation[      next_generation_len] = std::move(new_plan_info.first);
            next_generation_costs[next_generation_len] = new_plan_info.second;
            ++next_generation_len;
          }
        });
      });
    }
    /// Apply Firefly operator
    std::thread firefly_operator_thread;
    if constexpr(fireflyOperatorFlag(CONFIG))
    {
      firefly_operator_thread = std::thread([&] () -> void
      {
        if constexpr(not kMeansSelectionFlag(CONFIG))
        {
          std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
          [&] (const uint32_t from_ind) -> void
          {
            distances(from_ind, from_ind) = 0;
            const boost::integer_range<uint32_t> the_rest_of_the_population_inds(from_ind + 1, params.population_size);
            std::for_each(std::execution::par_unseq, the_rest_of_the_population_inds.begin(), the_rest_of_the_population_inds.end(),
            [&] (const uint32_t to_ind) -> void
            {
              if(not edit_scripts(from_ind, to_ind))
              {
                // Calculation for one way
                std::tie(distances(from_ind, to_ind), edit_scripts(from_ind, to_ind)) =
                  ld::findDistance<MAX_PLAN_VISITS,EIG_OPTIONS>(*population[from_ind],
                                                                *population[to_ind],
                                                                max_dwell_time);
                // Now the other way
                distances(to_ind, from_ind) = distances(from_ind, to_ind);
                edit_scripts(to_ind, from_ind) = ld::reverseAlignment<MAX_PLAN_VISITS,EIG_OPTIONS>(*edit_scripts(from_ind, to_ind));
              }
              else
              {
                assert((distances(from_ind, to_ind) == ld::findDistance<MAX_PLAN_VISITS,EIG_OPTIONS>(*population[from_ind], *population[to_ind], max_dwell_time).first));
              }
            });
          });
        }
        assert((params.population_size == edit_scripts.unaryExpr([] (const std::unique_ptr<ld::ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& es) -> bool { return not es; }).count()));
        if constexpr(noDuplicatesFlag(CONFIG))
        {
          assert(params.population_size == (distances.array() == 0).count());
        }

        // Perform movement
        Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> inv_costs;
        double                                             cost_scale_mult;
        if constexpr(bidirectionalFireflyOperatorFlag(CONFIG))
        {
          inv_costs       = population_costs.array().inverse();
          cost_scale_mult = population_costs[best_sol_found_ind] / population_costs[worst_sol_found_ind];
        }
        std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
        [&] (const uint32_t from_ind) -> void
        {
          if constexpr(pathRelinkingFireflyOperatorFlag(CONFIG))
          {
            const boost::integer_range<uint32_t> the_rest_of_the_population_inds(from_ind + 1, params.population_size);
            std::for_each(std::execution::par_unseq, the_rest_of_the_population_inds.begin(), the_rest_of_the_population_inds.end(),
            [&] (const uint32_t to_ind) -> void
            {
              // Check if there are any intermediate solutions
              if((edit_scripts(from_ind, to_ind)->size() - 1) < 1) { return; }

              // Evaluate every solution between from and to
              std::vector<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>> intermediate_plans =
                ld::findIntermediateSolutions<MAX_PLAN_VISITS,EIG_OPTIONS>(*edit_scripts(from_ind, to_ind), *population[from_ind], rand_gen);
              const uint32_t num_intermediate_sol = intermediate_plans.size();
              const boost::integer_range<uint32_t> intermediate_plans_inds(0, num_intermediate_sol);

              Eigen::Matrix<std::pair<bool,double>,Eigen::Dynamic,1,EIG_OPTIONS,(MAX_PLAN_VISITS == Eigen::Dynamic) ? Eigen::Dynamic : (2*MAX_PLAN_VISITS)-1,1> intermediate_costs(num_intermediate_sol);
              std::for_each(std::execution::unseq, intermediate_plans_inds.begin(), intermediate_plans_inds.end(),
              [&cost_func,&intermediate_plans,&intermediate_costs] (const uint32_t inter_plan_ind) -> void
              {
                const std::pair<bool,double> cost = cost_func(*intermediate_plans[inter_plan_ind]);
                assert((     cost.first  and (not std::isinf(cost.second))) or
                       ((not cost.first) and      std::isinf(cost.second)));
                intermediate_costs[inter_plan_ind] = cost;
              });
              // Find best intermediate plan
              Eigen::Index best_inter_plan_ind;
              intermediate_costs.unaryExpr([] (const std::pair<bool,double>& cost) -> double { return cost.second; }).template minCoeff<Eigen::NaNPropagationOptions::PropagateFast>(&best_inter_plan_ind);
              // Add best to next population
              if(intermediate_costs[best_inter_plan_ind].first)
              {
                if constexpr(noDuplicatesFlag(CONFIG))
                {
                  // If plan unique in population
                  if(std::any_of(std::execution::unseq, population_inds.begin(), population_inds.end(),
                                 [&] (const uint32_t comp_ind) -> bool
                                 {
                                   return (intermediate_plans[best_inter_plan_ind]->rows() == population[comp_ind]->rows()) and (*intermediate_plans[best_inter_plan_ind] == *population[comp_ind]);
                                 }))
                  { return; }
                }
                // Store the plan
                {
                  std::lock_guard<std::mutex> lock(next_generation_mux);
                  next_generation[      next_generation_len] = std::move(intermediate_plans[best_inter_plan_ind]);
                  next_generation_costs[next_generation_len] = intermediate_costs[best_inter_plan_ind].second;
                  ++next_generation_len;
                }
              }
            });
          }
          else // Not Path Relinking
          {
            std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
            [&] (const uint32_t to_ind) -> void
            {
              if(from_ind == to_ind) { return; }
              if constexpr(bidirectionalFireflyOperatorFlag(CONFIG))
              {
                double luminosity_mult = params.base_attractiveness_mult;
                if constexpr(defaultLuminosityMultiplayerFlag(CONFIG))
                {
                  luminosity_mult *= std::exp(-params.dist_mult * std::pow(double(distances(from_ind, to_ind)), params.dist_pow));
                }
                else // Flatter luminosity multiplayer
                {
                  luminosity_mult *= (double(1) / (double(1) + (params.dist_mult * std::pow(double(distances(from_ind, to_ind)), params.dist_pow))));
                }
                // Find distance to move
                const double move_dist = double(distances(from_ind, to_ind))
                                       * luminosity_mult
                                       * cost_scale_mult
                                       * population_costs[from_ind]
                                       * inv_costs[to_ind];
                // Move the plan
                std::pair<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,bool> new_plan =
                  ld::performMovement<MAX_PLAN_VISITS,EIG_OPTIONS>(*edit_scripts(from_ind, to_ind),
                                                                   *population[from_ind],
                                                                   move_dist,
                                                                   rand_gen);
                if(not new_plan.second) { return; }
                if constexpr(noDuplicatesFlag(CONFIG))
                {
                  // If plan unique in population
                  if(std::any_of(std::execution::unseq, population_inds.begin(), population_inds.end(),
                                 [&] (const uint32_t comp_ind) -> bool
                                 {
                                   return (new_plan.first->rows() == population[comp_ind]->rows()) and (*new_plan.first == *population[comp_ind]);
                                 }))
                  { return; }
                }
                // Check the plan
                const std::pair<bool,double> cost = cost_func(*new_plan.first);
                if(cost.first)
                {
                  // Store the plan
                  std::lock_guard<std::mutex> lock(next_generation_mux);
                  next_generation[      next_generation_len] = std::move(new_plan.first);
                  next_generation_costs[next_generation_len] = cost.second;
                  ++next_generation_len;
                }
              }
              else // Normal Firefly algorithm
              {
                // Check if movement is desired
                if(population_costs[from_ind] > population_costs[to_ind])
                {
                  // Find distance to move
                  double luminosity_mult = params.base_attractiveness_mult;
                  if constexpr(defaultLuminosityMultiplayerFlag(CONFIG))
                  {
                    luminosity_mult *= std::exp(-params.dist_mult * std::pow(double(distances(from_ind, to_ind)), params.dist_pow));
                  }
                  else // Flatter luminosity multiplayer
                  {
                    luminosity_mult *= (double(1) / (double(1) + (params.dist_mult * std::pow(double(distances(from_ind, to_ind)), params.dist_pow))));
                  }
                  // Move the plan
                  std::pair<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,bool> new_plan =
                    ld::performMovement<MAX_PLAN_VISITS,EIG_OPTIONS,std::default_random_engine>(*edit_scripts(from_ind, to_ind),
                                                                                                *population[from_ind],
                                                                                                luminosity_mult,
                                                                                                rand_gen);
                  if(not new_plan.second) { return; }
                  if constexpr(noDuplicatesFlag(CONFIG))
                  {
                    // If plan unique in population
                    if(std::any_of(std::execution::unseq, population_inds.begin(), population_inds.end(),
                                   [&] (const uint32_t comp_ind) -> bool
                                   {
                                     return (new_plan.first->rows() == population[comp_ind]->rows()) and (*new_plan.first == *population[comp_ind]);
                                   }))
                    { return; }
                  }
                  // Check the plan
                  const std::pair<bool,double> cost = cost_func(*new_plan.first);
                  if(cost.first)
                  {
                    // Store the plan
                    std::lock_guard<std::mutex> lock(next_generation_mux);
                    next_generation[      next_generation_len] = std::move(new_plan.first);
                    next_generation_costs[next_generation_len] = cost.second;
                    ++next_generation_len;
                  }
                }
              }
            });
          }
        });
      });
    }
    /// Apply mutations
    std::thread mutations_thread;
    Eigen::Array<float,mo::numberMutations(MUTAT_CONFIG),Eigen::Dynamic,EIG_OPTIONS,mo::numberMutations(MUTAT_CONFIG),Eigen::Dynamic> apply_mutation_prob_sample;
    if constexpr(mo::usingAnyMutations(MUTAT_CONFIG))
    {
      apply_mutation_prob_sample =
        Eigen::Array<float,mo::numberMutations(MUTAT_CONFIG),Eigen::Dynamic,EIG_OPTIONS,mo::numberMutations(MUTAT_CONFIG),Eigen::Dynamic>::NullaryExpr(mo::numberMutations(MUTAT_CONFIG), params.population_size, [&] () { return probability_dist(rand_gen); });

      mutations_thread = std::thread([&] () -> void
      {
        std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
        [&] (const uint32_t population_ind) -> void
        {
          const Eigen::Array<bool,mo::numberMutations(MUTAT_CONFIG),1,EIG_OPTIONS,mo::numberMutations(MUTAT_CONFIG),1> apply_mutation = apply_mutation_prob_sample.col(population_ind) < params.mutation_operators_probability.array();

          std::for_each(mutation_operator_inds.begin(), mutation_operator_inds.end(),
          [&] (const uint32_t mutation_ind) -> void
          {
            if(not apply_mutation[mutation_ind]) { return; }
            // Perform mutation
            std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> mutated_plan =
              mo::applyMutation<MUTAT_CONFIG,MAX_PLAN_VISITS,EIG_OPTIONS>(*population[population_ind],
                                                                          mutation_ind,
                                                                          params.max_dwell_time_hotspot,
                                                                          params.max_dwell_time_depot,
                                                                          params.tweak_dwell_time_max_change,
                                                                          graph,
                                                                          rand_gen);
            if(nullptr == mutated_plan.get()) { return; }
            if constexpr(noDuplicatesFlag(CONFIG))
            {
              // If plan unique in population
              if(not std::none_of(population_inds.begin(), population_inds.end(),
                                  [&] (const uint32_t comp_ind) -> bool
                                  {
                                    return (mutated_plan->rows() == population[comp_ind]->rows()) and (*mutated_plan == *population[comp_ind]);
                                  }))
              {
                return;
              }
            }
            // Evaluate cost
            const std::pair<bool,double> cost = cost_func(*mutated_plan);
            // If plan valid
            if(cost.first)
            {
              // Add plan to next generation
              std::lock_guard<std::mutex> lock(next_generation_mux);
              next_generation[      next_generation_len] = std::move(mutated_plan);
              next_generation_costs[next_generation_len] = cost.second;
              ++next_generation_len;
            }
          });
        });
      });
    }

    if constexpr(mo::usingAnyMutations(MUTAT_CONFIG))
    {
      mutations_thread.join();
    }
    if constexpr(localSearchFlag(CONFIG))
    {
      hybridization_thread.join();
    }
    if constexpr(fireflyOperatorFlag(CONFIG))
    {
      firefly_operator_thread.join();
    }
    std::vector<uint32_t> non_duplicate_inds;
    if constexpr(noDuplicatesFlag(CONFIG))
    {
      non_duplicate_inds.reserve(next_generation_len);
      // Make sure everything is unique
      for(uint32_t next_gen_ind = 0; next_gen_ind < next_generation_len; ++next_gen_ind)
      {
        if(std::none_of(std::execution::unseq, non_duplicate_inds.cbegin(), non_duplicate_inds.cend(),
                        [&] (const uint32_t& comp_plan_ind) -> bool
                        {
                          return (next_generation[next_gen_ind]->rows() == next_generation[comp_plan_ind]->rows()) and (*next_generation[next_gen_ind] == *next_generation[comp_plan_ind]);
                        }))
        {
          non_duplicate_inds.emplace_back(next_gen_ind);
        }
      }
    }

    /// Perform selection
    {
      uint32_t new_fireflies_size;
      if constexpr(noDuplicatesFlag(CONFIG))
      {
        new_fireflies_size = non_duplicate_inds.size();
      }
      else
      {
        new_fireflies_size = next_generation_len;
      }
      if(0 == new_fireflies_size) { continue; }

      const uint32_t full_population_size = params.population_size + new_fireflies_size;

      std::vector<uint32_t> full_population_inds(full_population_size);
      std::iota(full_population_inds.begin(), full_population_inds.end(), 0);

      Eigen::Matrix<std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,Eigen::Dynamic,1,EIG_OPTIONS>                       full_population(      full_population_size);
      Eigen::Matrix<double,                                                 Eigen::Dynamic,1,EIG_OPTIONS>                       full_population_costs(full_population_size);
      Eigen::Matrix<uint32_t,Eigen::Dynamic,Eigen::Dynamic,EIG_OPTIONS>                                                         full_distances;
      Eigen::Matrix<std::unique_ptr<ld::ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>,Eigen::Dynamic,Eigen::Dynamic,EIG_OPTIONS> full_edit_scripts;

      #ifndef NDEBUG
        full_population_costs.setConstant(std::numeric_limits<double>::quiet_NaN());
      #endif

      std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
      [&] (const uint32_t population_ind) -> void
      {
        full_population[population_ind] = std::move(population[population_ind]);
      });
      std::for_each(std::execution::unseq, std::next(full_population_inds.cbegin(), params.population_size), full_population_inds.cend(),
      [&] (const uint32_t population_ind) -> void
      {
        if constexpr(noDuplicatesFlag(CONFIG))
        {
          full_population[population_ind] = std::move(next_generation[non_duplicate_inds[population_ind - params.population_size]]);
        }
        else
        {
          full_population[population_ind] = std::move(next_generation[population_ind - params.population_size]);
        }
      });
      full_population_costs.topRows(params.population_size) = population_costs.topRows(params.population_size);
      if constexpr(noDuplicatesFlag(CONFIG))
      {
        full_population_costs.bottomRows(new_fireflies_size) = next_generation_costs(non_duplicate_inds);
      }
      else
      {
        full_population_costs.bottomRows(new_fireflies_size) = next_generation_costs.topRows(new_fireflies_size);
      }
      assert(0 == full_population.unaryExpr([] (const std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& plan) -> bool { return nullptr == plan.get(); }).count());
      assert((full_population_costs.array() != std::numeric_limits<double>::quiet_NaN()).all());
      if constexpr(fireflyOperatorFlag(CONFIG))
      {
        full_distances.   resize(full_population_size, full_population_size);
        full_edit_scripts.resize(full_population_size, full_population_size);
        // Copy old population data
        full_distances.topLeftCorner(params.population_size, params.population_size) = distances;
        std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
        [&] (const uint32_t from_ind) -> void
        {
          std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
          [&] (const uint32_t to_ind) -> void
          {
            full_edit_scripts(from_ind, to_ind) = std::move(edit_scripts(from_ind, to_ind));
          });
        });
        // Set new population data
        if constexpr(kMeansSelectionFlag(CONFIG))
        {
          std::for_each(std::execution::par_unseq, full_population_inds.begin(), full_population_inds.end(),
          [&] (const uint32_t from_ind) -> void
          {
            const bool from_set = from_ind < params.population_size;
            full_distances(from_ind, from_ind) = 0;
            const boost::integer_range<uint32_t> the_rest_of_the_population_inds(from_ind + 1, full_population_size);
            std::for_each(std::execution::par_unseq, the_rest_of_the_population_inds.begin(), the_rest_of_the_population_inds.end(),
            [&] (const uint32_t to_ind) -> void
            {
              const bool to_set = to_ind < params.population_size;
              if(from_set and to_set) { return; }
              // Calculation for one way
              assert(full_population[from_ind]);
              assert(full_population[to_ind]);
              // Calculation for one way
              std::tie(full_distances(from_ind, to_ind), full_edit_scripts(from_ind, to_ind)) =
                ld::findDistance<MAX_PLAN_VISITS,EIG_OPTIONS>(*full_population[from_ind],
                                                              *full_population[to_ind],
                                                              max_dwell_time);
              // Now the other way
              full_distances(   to_ind, from_ind) = full_distances(from_ind, to_ind);
              full_edit_scripts(to_ind, from_ind) = ld::reverseAlignment<MAX_PLAN_VISITS,EIG_OPTIONS>(*full_edit_scripts(from_ind, to_ind));
            });
          });
        }
      }

      if constexpr(fitnessSelectionFlag(CONFIG))
      {
        std::partial_sort(std::execution::unseq, full_population_inds.begin(), full_population_inds.begin() + params.population_size, full_population_inds.end(),
                          [&] (const uint32_t i, const uint32_t j) -> bool
                          { return full_population_costs[i] < full_population_costs[j]; });
        std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
        [&] (const uint32_t population_ind) -> void
        {
          population[      population_ind] = std::move(full_population[full_population_inds[population_ind]]);
          population_costs[population_ind] = full_population_costs[full_population_inds[population_ind]];
        });
        if constexpr(fireflyOperatorFlag(CONFIG))
        {
          full_population_inds.resize(params.population_size);
          distances = full_distances(full_population_inds, full_population_inds);
          std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
          [&] (const uint32_t from_ind_ind) -> void
          {
            const uint32_t from_ind = full_population_inds[from_ind_ind];
            std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
            [&] (const uint32_t to_ind_ind) -> void
            {
              const uint32_t to_ind = full_population_inds[to_ind_ind];

              edit_scripts(from_ind_ind, to_ind_ind) = std::move(full_edit_scripts(from_ind, to_ind));
            });
          });
        }
      }
      if constexpr(stochasticUniversalSamplingSelectionFlag(CONFIG))
      {
        std::vector<uint32_t> new_population_inds;
        new_population_inds.reserve(params.population_size);

        if constexpr(elitismSelectionFlag(CONFIG))
        {
          auto min_it = std::min_element(std::execution::unseq, full_population_inds.begin(), full_population_inds.end(),
                                         [&] (const uint32_t i, const uint32_t j) -> bool
                                         { return full_population_costs[i] < full_population_costs[j]; });
          assert(min_it != full_population_inds.end());
          std::swap(*min_it, full_population_inds.back());
          new_population_inds.emplace_back(full_population_inds.back());
          full_population_inds.pop_back();
        }
        std::sample(full_population_inds.begin(), full_population_inds.end(), std::back_inserter(new_population_inds), params.population_size - new_population_inds.size(), rand_gen);
        std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
        [&] (const uint32_t population_ind) -> void
        {
          population[population_ind] = std::move(full_population[new_population_inds[population_ind]]);
        });
        population_costs = full_population_costs(new_population_inds);
        if constexpr(fireflyOperatorFlag(CONFIG))
        {
          distances = full_distances(new_population_inds, new_population_inds);
          std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
          [&] (const uint32_t from_ind_ind) -> void
          {
            const uint32_t from_ind = new_population_inds[from_ind_ind];
            std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
            [&] (const uint32_t to_ind_ind) -> void
            {
              const uint32_t to_ind = new_population_inds[to_ind_ind];

              edit_scripts(from_ind_ind, to_ind_ind) = std::move(full_edit_scripts(from_ind, to_ind));
            });
          });
        }
      }
      if constexpr(fitnessProportionalSelectionFlag(CONFIG))
      {
        std::vector<uint32_t> new_population_inds;
        new_population_inds.reserve(params.population_size);

        if constexpr(elitismSelectionFlag(CONFIG))
        {
          auto min_it = std::min_element(std::execution::unseq, full_population_inds.begin(), full_population_inds.end(),
                                         [&] (const uint32_t i, const uint32_t j) -> bool
                                         { return full_population_costs[i] < full_population_costs[j]; });
          assert(min_it != full_population_inds.end());
          std::swap(*min_it, full_population_inds.back());
          new_population_inds.emplace_back(full_population_inds.back());
          full_population_inds.pop_back();
        }
        // Find fitness and make is a cumulative probability
        const uint32_t full_population_inds_size = full_population_inds.size();
        Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> fitness_values = full_population_costs;
        fitness_values = (double(1) - (fitness_values.array() / fitness_values.template maxCoeff<Eigen::NaNPropagationOptions::PropagateFast>())).eval();

        /// Perform selection
        // Selection from Efraimidis, Pavlos S., and Paul G. Spirakis. "Weighted random sampling with a reservoir." Information processing letters 97.5 (2006): 181-185.
        const Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> u    = Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS>::NullaryExpr(full_population_size, [&] () { return probability_dist(rand_gen); });
              Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> keys = u.binaryExpr(fitness_values, [] (const double ui, const double mpi) -> double { return std::pow(ui, double(1) / mpi); });
        if constexpr(elitismSelectionFlag(CONFIG))
        {
          // Prevent elite samples from getting selected again
          keys(new_population_inds, 0).setConstant(-std::numeric_limits<double>::infinity());
        }
        const uint32_t num_to_sort = params.population_size - new_population_inds.size();
        std::partial_sort(std::execution::unseq, full_population_inds.begin(), full_population_inds.begin() + num_to_sort, full_population_inds.end(),
                          [&] (const uint32_t i, const uint32_t j) -> bool { return keys[i] > keys[j]; });
        std::copy_n(std::execution::unseq, full_population_inds.begin(), num_to_sort, std::back_inserter(new_population_inds));
        assert(params.population_size == new_population_inds.size());

        std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
        [&] (const uint32_t population_ind) -> void
        {
          population[population_ind] = std::move(full_population[new_population_inds[population_ind]]);
        });
        population_costs = full_population_costs(new_population_inds);
        if constexpr(fireflyOperatorFlag(CONFIG))
        {
          distances = full_distances(new_population_inds, new_population_inds);
          std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
          [&] (const uint32_t from_ind_ind) -> void
          {
            const uint32_t from_ind = new_population_inds[from_ind_ind];
            std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
            [&] (const uint32_t to_ind_ind) -> void
            {
              const uint32_t to_ind = new_population_inds[to_ind_ind];

              edit_scripts(from_ind_ind, to_ind_ind) = std::move(full_edit_scripts(from_ind, to_ind));
            });
          });
        }
        assert(0 == population.unaryExpr([] (const std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& plan) -> bool { return nullptr == plan.get(); }).count());
        assert((population_costs.array() != std::numeric_limits<double>::quiet_NaN()).all());
      }
      if constexpr(kMeansSelectionFlag(CONFIG))
      {
        std::vector<uint32_t> new_population_inds;
        new_population_inds.reserve(params.population_size);

        const uint32_t                       number_clusters = std::min<uint32_t>(params.number_clusters, full_population_size);
        const boost::integer_range<uint32_t> cluster_inds(0, number_clusters);

        /// Find cluster centers
        Eigen::Matrix<uint32_t,Eigen::Dynamic,1,EIG_OPTIONS> cluster_center_inds(number_clusters);
        std::vector<uint32_t> full_population_inds_cluster_center = full_population_inds;
        {
          std::uniform_int_distribution<uint32_t> rand_dist(0, full_population_size - 1);
          cluster_center_inds[0] = rand_dist(rand_gen);
          full_population_inds_cluster_center.erase(std::next(full_population_inds_cluster_center.begin(), cluster_center_inds[0]));
        }
        for(uint32_t cluster_ind = 1; cluster_ind < number_clusters; ++cluster_ind)
        {
          const uint32_t                       full_population_inds_size = full_population_inds_cluster_center.size();
          const boost::integer_range<uint32_t> full_population_inds_inds(0, full_population_inds_size);

          Eigen::Matrix<uint32_t,Eigen::Dynamic,1,EIG_OPTIONS> dists_to_closest_cluster_p2(full_population_inds_size);
          std::for_each(std::execution::par_unseq, full_population_inds_inds.begin(), full_population_inds_inds.end(),
          [&] (const uint32_t full_pop_ind_ind) -> void
          {
            const uint32_t pop_ind = full_population_inds_cluster_center[full_pop_ind_ind];
            const auto         min_cluster_ind_ptr = std::min_element(std::execution::unseq, cluster_center_inds.data(), cluster_center_inds.data() + cluster_ind,
                                                                      [&] (const uint32_t i, const uint32_t j) -> bool { return full_distances(pop_ind, i) < full_distances(pop_ind, j); });
            assert(min_cluster_ind_ptr != (cluster_center_inds.data() + cluster_ind));
            const uint32_t distance = full_distances(pop_ind, *min_cluster_ind_ptr);
            dists_to_closest_cluster_p2[full_pop_ind_ind] = distance * distance;
          });

          Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> cum_prob(full_population_inds_size);
          cum_prob[0] = dists_to_closest_cluster_p2[0];
          for(uint32_t population_ind = 1; population_ind < full_population_inds_size; ++population_ind)
          {
            cum_prob[population_ind] = double(dists_to_closest_cluster_p2[population_ind]) + cum_prob[population_ind - 1];
          }
          cum_prob.array() /= double(dists_to_closest_cluster_p2.sum());

          const double probability_val = probability_dist(rand_gen);
          const uint32_t to_add_ind      = (cum_prob.array() < probability_val).count();
          // Make it a cluster center and remove it from old population
          cluster_center_inds[cluster_ind] = full_population_inds_cluster_center[to_add_ind];
          full_population_inds_cluster_center.erase(std::next(full_population_inds_cluster_center.cbegin(), to_add_ind));
        }
        /// Perform Clustering
        Eigen::Matrix<std::vector<uint32_t>,Eigen::Dynamic,1,EIG_OPTIONS> cluster_member_inds(    number_clusters);
        Eigen::Matrix<std::mutex,Eigen::Dynamic,1,EIG_OPTIONS>                cluster_member_inds_mux(number_clusters);
        std::for_each(std::execution::unseq, cluster_inds.begin(), cluster_inds.end(),
        [&] (const uint32_t cluster_ind) -> void
        {
          cluster_member_inds[cluster_ind].reserve(full_population_size);
          cluster_member_inds[cluster_ind].emplace_back(cluster_center_inds[cluster_ind]);
        });
        std::for_each(std::execution::par_unseq, full_population_inds_cluster_center.begin(), full_population_inds_cluster_center.end(),
        [&] (const uint32_t population_ind) -> void
        {
          // Find closest cluster
        Eigen::Index closest_cluster_ind;
          full_distances(population_ind, cluster_center_inds).template minCoeff<Eigen::NaNPropagationOptions::PropagateFast>(&closest_cluster_ind);
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
              sum_of_dists[member_ind] = full_distances(member_ind, cluster_member_inds[cluster_ind]).sum();
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
          std::for_each(std::execution::unseq, cluster_inds.begin(), cluster_inds.end(),
          [&] (const uint32_t cluster_ind) -> void
          {
            cluster_member_inds[cluster_ind].clear();
            cluster_member_inds[cluster_ind].reserve(full_population_size);
            cluster_member_inds[cluster_ind].emplace_back(cluster_center_inds[cluster_ind]);
          });
          // Assign clusters
          std::for_each(std::execution::par_unseq, full_population_inds.begin(), full_population_inds.end(),
          [&] (const uint32_t population_ind) -> void
          {
            if((cluster_center_inds.array() == population_ind).any()) { return; }
            // Find closest cluster
            Eigen::Index closest_cluster_ind;
            full_distances(population_ind, cluster_center_inds).template minCoeff<Eigen::NaNPropagationOptions::PropagateFast>(&closest_cluster_ind);
            // Add to that cluster
            {
              std::lock_guard<std::mutex> lock(cluster_member_inds_mux[closest_cluster_ind]);
              cluster_member_inds[closest_cluster_ind].emplace_back(population_ind);
            }
          });
          assert(full_population_size == cluster_member_inds.unaryExpr([] (const std::vector<uint32_t>& i) -> uint32_t { return i.size(); }).sum());
        }
        if constexpr(elitismSelectionFlag(CONFIG))
        {
          std::for_each(cluster_inds.begin(), cluster_inds.end(),
          [&] (const uint32_t cluster_ind) -> void
          {
            auto min_it = std::min_element(std::execution::unseq, cluster_member_inds[cluster_ind].begin(), cluster_member_inds[cluster_ind].end(),
                                           [&] (const uint32_t i, const uint32_t j) -> bool
                                           { return full_population_costs[i] < full_population_costs[j]; });
            assert(min_it != cluster_member_inds[cluster_ind].end());
            std::swap(*min_it, cluster_member_inds[cluster_ind].back());
            new_population_inds.emplace_back(cluster_member_inds[cluster_ind].back());
            cluster_member_inds[cluster_ind].pop_back();
          });
        }
        /// Calculate membership probability index
        Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> membership_probability_index(full_population_size);
        const double inv_population_size = double(1) / double(full_population_size);
        std::for_each(std::execution::par_unseq, cluster_inds.begin(), cluster_inds.end(),
        [&] (const uint32_t cluster_ind) -> void
        {
          uint32_t cluster_size = cluster_member_inds[cluster_ind].size();
          if(1 == cluster_size) { cluster_size = 2; } // To avoid NaNs in the size term
          const double sum_of_fitness    = full_population_costs(cluster_member_inds[cluster_ind]).sum();
          const double size_term         = double(cluster_size) / double(cluster_size - 1);
          const double size_and_pop_term = size_term * inv_population_size;
          std::for_each(std::execution::unseq, cluster_member_inds[cluster_ind].cbegin(), cluster_member_inds[cluster_ind].cend(),
          [&] (const uint32_t member_ind) -> void
          {
            membership_probability_index[member_ind] = size_and_pop_term * ((sum_of_fitness - full_population_costs[member_ind]) / sum_of_fitness);
          });
        });
        /// Perform selection
        // Selection from Efraimidis, Pavlos S., and Paul G. Spirakis. "Weighted random sampling with a reservoir." Information processing letters 97.5 (2006): 181-185.
        const Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> u    = Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS>::NullaryExpr(full_population_size, [&] () { return probability_dist(rand_gen); });
              Eigen::Matrix<double,Eigen::Dynamic,1,EIG_OPTIONS> keys = u.binaryExpr(membership_probability_index, [] (const double ui, const double mpi) -> double { return std::pow(ui, double(1) / mpi); });
        if constexpr(elitismSelectionFlag(CONFIG))
        {
          // Prevent elite samples from getting selected again
          keys(new_population_inds, 0).setConstant(-std::numeric_limits<double>::infinity());
        }
        const uint32_t num_to_sort = params.population_size - new_population_inds.size();
        std::partial_sort(std::execution::unseq, full_population_inds.begin(), full_population_inds.begin() + num_to_sort, full_population_inds.end(),
                          [&] (const uint32_t i, const uint32_t j) -> bool { return keys[i] > keys[j]; });
        std::copy_n(std::execution::unseq, full_population_inds.begin(), num_to_sort, std::back_inserter(new_population_inds));
        assert(params.population_size == new_population_inds.size());
        /// Update population for next loop
        std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
        [&] (const uint32_t population_ind) -> void
        {
          assert(nullptr != full_population[new_population_inds[population_ind]].get());
          population[population_ind] = std::move(full_population[new_population_inds[population_ind]]);
        });
        population_costs = full_population_costs(new_population_inds);
        distances        = full_distances(new_population_inds, new_population_inds);
        if constexpr(fireflyOperatorFlag(CONFIG))
        {
          std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
          [&] (const uint32_t from_ind_ind) -> void
          {
            const uint32_t from_ind = new_population_inds[from_ind_ind];
            std::for_each(std::execution::unseq, population_inds.begin(), population_inds.end(),
            [&] (const uint32_t to_ind_ind) -> void
            {
              const uint32_t to_ind = new_population_inds[to_ind_ind];

              edit_scripts(from_ind_ind, to_ind_ind) = std::move(full_edit_scripts(from_ind, to_ind));
            });
          });
          assert((params.population_size == edit_scripts.unaryExpr([] (const std::unique_ptr<ld::ALIGNMENT_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& es) -> bool { return not es; }).count()));
          assert((distances.array() != -1).all());
        }
      }
    }
    assert(params.population_size == population.      rows());
    assert(params.population_size == population_costs.rows());
    assert(params.population_size == distances.       rows());
    assert(params.population_size == distances.       cols());
    assert(params.population_size == edit_scripts.    rows());
    assert(params.population_size == edit_scripts.    cols());
    assert(0 == population.unaryExpr([] (const std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& plan) -> bool { return nullptr == plan.get(); }).count());
    assert((population_costs.array() != std::numeric_limits<double>::quiet_NaN()).all());

    /// Bookkeeping
    log_func(population_costs);
    {
      const auto best_worst_ind_its = std::minmax_element(std::execution::unseq, population_inds.begin(), population_inds.end(),
                                                          [&population_costs] (const uint32_t i, const uint32_t j) -> bool
                                                          { return population_costs[i] < population_costs[j]; });
      best_sol_found_ind  = *best_worst_ind_its.first;
      worst_sol_found_ind = *best_worst_ind_its.second;
    }
  }

  return std::move(population[best_sol_found_ind]);
}

#endif
/* firefly_planner.hpp */
