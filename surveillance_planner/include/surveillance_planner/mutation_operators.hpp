/**
 * @File: mutation_operators.hpp
 * @Date: June 2024
 * @Author: James Swedeen
 *
 * @brief
 * Defines a few randomly generated mutation operators.
 **/

#ifndef SURVEILLANCE_PLANNING_MUTATION_OPERATORS_HPP
#define SURVEILLANCE_PLANNING_MUTATION_OPERATORS_HPP

/* C++ Headers */
#include<vector>
#include<random>
#include<iostream> // TODO: remove

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Street Graph Headers */
#include<street_graph/planning_graph.hpp>

/* Local Headers */
#include<surveillance_planner/action.hpp>
#include<surveillance_planner/helpers.hpp>

namespace plan
{
namespace mo
{
/**
 * @MutationsFlags
 *
 * @brief
 * Used to control what mutations are used in the planners.
 **/
enum MutationsFlags : uint64_t
{
  /**
   * @NULL_FLAG
   *
   * This enumeration explicitly represents nothing.
   **/
  NULL_FLAG = 0x0000'0000'0000'0000,
  /**
   * @USE_*_MUTATION
   *
   * Set to use this mutation.
   **/
  USE_ALL_MUTATIONS                                   = 0x0000'003F'FFFF'FFFF,
  USE_ALL_SIMPLE_MUTATIONS                            = 0x0000'0000'0000'3FFF,
  USE_ADD_VISIT_MUTATION                              = 0x0000'0000'0000'0001,
  USE_ADD_HOTSPOT_VISIT_MUTATION                      = 0x0000'0000'0000'0002,
  USE_ADD_DEPOT_VISIT_MUTATION                        = 0x0000'0000'0000'0004,
  USE_REMOVE_VISIT_MUTATION                           = 0x0000'0000'0000'0008,
  USE_REMOVE_HOTSPOT_VISIT_MUTATION                   = 0x0000'0000'0000'0010,
  USE_REMOVE_DEPOT_VISIT_MUTATION                     = 0x0000'0000'0000'0020,
  USE_REMOVE_VISITS_WINDOWED_MUTATION                 = 0x0000'0000'0000'0040,
  USE_CHANGE_DWELL_TIME_MUTATION                      = 0x0000'0000'0000'0080,
  USE_TWEAK_DWELL_TIME_MUTATION                       = 0x0000'0000'0000'0100,
  USE_SWAP_PATH_MUTATION                              = 0x0000'0000'0000'0200,
  USE_SWAP_VISITS_MUTATION                            = 0x0000'0000'0000'0400,
  USE_SWAP_MULTIPLE_VISITS_MUTATION                   = 0x0000'0000'0000'0800,
  USE_MOVE_VISIT_MUTATION                             = 0x0000'0000'0000'1000,
  USE_SIMPLE_INVERSION_MUTATION                       = 0x0000'0000'0000'2000,
  USE_ADD_VISIT_MIN_TIME_PATH_MUTATION                = 0x0000'0000'0000'4000,
  USE_ADD_HOTSPOT_VISIT_MIN_TIME_PATH_MUTATION        = 0x0000'0000'0000'8000,
  USE_ADD_DEPOT_VISIT_MIN_TIME_PATH_MUTATION          = 0x0000'0000'0001'0000,
  USE_REMOVE_VISIT_MIN_TIME_PATH_MUTATION             = 0x0000'0000'0002'0000,
  USE_REMOVE_HOTSPOT_VISIT_MIN_TIME_PATH_MUTATION     = 0x0000'0000'0004'0000,
  USE_REMOVE_DEPOT_VISIT_MIN_TIME_PATH_MUTATION       = 0x0000'0000'0008'0000,
  USE_REMOVE_VISITS_WINDOWED_MIN_TIME_PATH_MUTATION   = 0x0000'0000'0010'0000,
  USE_SWAP_PATH_MIN_TIME_PATH_MUTATION                = 0x0000'0000'0020'0000,
  USE_SWAP_VISITS_MIN_TIME_PATH_MUTATION              = 0x0000'0000'0040'0000,
  USE_SWAP_MULTIPLE_VISITS_MIN_TIME_PATH_MUTATION     = 0x0000'0000'0080'0000,
  USE_MOVE_VISIT_MIN_TIME_PATH_MUTATION               = 0x0000'0000'0100'0000,
  USE_SIMPLE_INVERSION_MIN_TIME_PATH_MUTATION         = 0x0000'0000'0200'0000,
  USE_ADD_VISIT_MIN_CHARGE_PATH_MUTATION              = 0x0000'0000'0400'0000,
  USE_ADD_HOTSPOT_VISIT_MIN_CHARGE_PATH_MUTATION      = 0x0000'0000'0800'0000,
  USE_ADD_DEPOT_VISIT_MIN_CHARGE_PATH_MUTATION        = 0x0000'0000'1000'0000,
  USE_REMOVE_VISIT_MIN_CHARGE_PATH_MUTATION           = 0x0000'0000'2000'0000,
  USE_REMOVE_HOTSPOT_VISIT_MIN_CHARGE_PATH_MUTATION   = 0x0000'0000'4000'0000,
  USE_REMOVE_DEPOT_VISIT_MIN_CHARGE_PATH_MUTATION     = 0x0000'0000'8000'0000,
  USE_REMOVE_VISITS_WINDOWED_MIN_CHARGE_PATH_MUTATION = 0x0000'0001'0000'0000,
  USE_SWAP_PATH_MIN_CHARGE_PATH_MUTATION              = 0x0000'0002'0000'0000,
  USE_SWAP_VISITS_MIN_CHARGE_PATH_MUTATION            = 0x0000'0004'0000'0000,
  USE_SWAP_MULTIPLE_VISITS_MIN_CHARGE_PATH_MUTATION   = 0x0000'0008'0000'0000,
  USE_MOVE_VISIT_MIN_CHARGE_PATH_MUTATION             = 0x0000'0010'0000'0000,
  USE_SIMPLE_INVERSION_MIN_CHARGE_PATH_MUTATION       = 0x0000'0020'0000'0000,
  /**
   * @USE_RECOMMENDATIONS
   *
   * @brief
   * These ar the mutations that I have found to be the most useful.
   **/
  USE_RECOMMENDATIONS = USE_ADD_DEPOT_VISIT_MUTATION bitor
                        USE_ADD_HOTSPOT_VISIT_MUTATION bitor
                        USE_REMOVE_DEPOT_VISIT_MUTATION bitor
                        USE_REMOVE_HOTSPOT_VISIT_MUTATION bitor
                        USE_REMOVE_VISITS_WINDOWED_MUTATION bitor
                        USE_SIMPLE_INVERSION_MUTATION bitor
                        USE_SWAP_MULTIPLE_VISITS_MUTATION bitor
                        USE_SWAP_PATH_MUTATION bitor
                        USE_SWAP_VISITS_MUTATION bitor
                        USE_TWEAK_DWELL_TIME_MUTATION
};
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
constexpr bool nullFlag(                                     const MutationsFlags config) noexcept;
constexpr bool addVisitMutationFlag(                         const MutationsFlags config) noexcept;
constexpr bool addHotspotVisitMutationFlag(                  const MutationsFlags config) noexcept;
constexpr bool addDepotVisitMutationFlag(                    const MutationsFlags config) noexcept;
constexpr bool removeVisitMutationFlag(                      const MutationsFlags config) noexcept;
constexpr bool removeHotspotVisitMutationFlag(               const MutationsFlags config) noexcept;
constexpr bool removeDepotVisitMutationFlag(                 const MutationsFlags config) noexcept;
constexpr bool removeVisitsWindowedMutationFlag(             const MutationsFlags config) noexcept;
constexpr bool changeDwellTimeMutationFlag(                  const MutationsFlags config) noexcept;
constexpr bool tweakDwellTimeMutationFlag(                   const MutationsFlags config) noexcept;
constexpr bool swapPathMutationFlag(                         const MutationsFlags config) noexcept;
constexpr bool swapVisitsMutationFlag(                       const MutationsFlags config) noexcept;
constexpr bool swapMultipleVisitsMutationFlag(               const MutationsFlags config) noexcept;
constexpr bool moveVisitMutationFlag(                        const MutationsFlags config) noexcept;
constexpr bool simpleInversionMutationFlag(                  const MutationsFlags config) noexcept;
constexpr bool addVisitMinTimePathMutationFlag(              const MutationsFlags config) noexcept;
constexpr bool addHotspotVisitMinTimePathMutationFlag(       const MutationsFlags config) noexcept;
constexpr bool addDepotVisitMinTimePathMutationFlag(         const MutationsFlags config) noexcept;
constexpr bool removeVisitMinTimePathMutationFlag(           const MutationsFlags config) noexcept;
constexpr bool removeHotspotVisitMinTimePathMutationFlag(    const MutationsFlags config) noexcept;
constexpr bool removeDepotVisitMinTimePathMutationFlag(      const MutationsFlags config) noexcept;
constexpr bool removeVisitsWindowedMinTimePathMutationFlag(  const MutationsFlags config) noexcept;
constexpr bool swapPathMinTimePathMutationFlag(              const MutationsFlags config) noexcept;
constexpr bool swapVisitsMinTimePathMutationFlag(            const MutationsFlags config) noexcept;
constexpr bool swapMultipleVisitsMinTimePathMutationFlag(    const MutationsFlags config) noexcept;
constexpr bool moveVisitMinTimePathMutationFlag(             const MutationsFlags config) noexcept;
constexpr bool simpleInversionMinTimePathMutationFlag(       const MutationsFlags config) noexcept;
constexpr bool addVisitMinChargePathMutationFlag(            const MutationsFlags config) noexcept;
constexpr bool addHotspotVisitMinChargePathMutationFlag(     const MutationsFlags config) noexcept;
constexpr bool addDepotVisitMinChargePathMutationFlag(       const MutationsFlags config) noexcept;
constexpr bool removeVisitMinChargePathMutationFlag(         const MutationsFlags config) noexcept;
constexpr bool removeHotspotVisitMinChargePathMutationFlag(  const MutationsFlags config) noexcept;
constexpr bool removeDepotVisitMinChargePathMutationFlag(    const MutationsFlags config) noexcept;
constexpr bool removeVisitsWindowedMinChargePathMutationFlag(const MutationsFlags config) noexcept;
constexpr bool swapPathMinChargePathMutationFlag(            const MutationsFlags config) noexcept;
constexpr bool swapVisitsMinChargePathMutationFlag(          const MutationsFlags config) noexcept;
constexpr bool swapMultipleVisitsMinChargePathMutationFlag(  const MutationsFlags config) noexcept;
constexpr bool moveVisitMinChargePathMutationFlag(           const MutationsFlags config) noexcept;
constexpr bool simpleInversionMinChargePathMutationFlag(     const MutationsFlags config) noexcept;
/**
 * @Mutation operator probability indexes
 *
 * @brief
 * Each is used to test if a given attribute is held in the given configuration.
 *
 * @parameters
 * config: The configuration to test
 **/
constexpr bool     usingAnyMutations(                             const MutationsFlags config) noexcept;
constexpr uint32_t numberMutations(                               const MutationsFlags config) noexcept;
constexpr uint32_t addVisitMutationIndex(                         const MutationsFlags config) noexcept;
constexpr uint32_t addHotspotVisitMutationIndex(                  const MutationsFlags config) noexcept;
constexpr uint32_t addDepotVisitMutationIndex(                    const MutationsFlags config) noexcept;
constexpr uint32_t removeVisitMutationIndex(                      const MutationsFlags config) noexcept;
constexpr uint32_t removeHotspotVisitMutationIndex(               const MutationsFlags config) noexcept;
constexpr uint32_t removeDepotVisitMutationIndex(                 const MutationsFlags config) noexcept;
constexpr uint32_t removeVisitsWindowedMutationIndex(             const MutationsFlags config) noexcept;
constexpr uint32_t changeDwellTimeMutationIndex(                  const MutationsFlags config) noexcept;
constexpr uint32_t tweakDwellTimeMutationIndex(                   const MutationsFlags config) noexcept;
constexpr uint32_t swapPathMutationIndex(                         const MutationsFlags config) noexcept;
constexpr uint32_t swapVisitsMutationIndex(                       const MutationsFlags config) noexcept;
constexpr uint32_t swapMultipleVisitsMutationIndex(               const MutationsFlags config) noexcept;
constexpr uint32_t moveVisitMutationIndex(                        const MutationsFlags config) noexcept;
constexpr uint32_t simpleInversionMutationIndex(                  const MutationsFlags config) noexcept;
constexpr uint32_t addVisitMinTimePathMutationIndex(              const MutationsFlags config) noexcept;
constexpr uint32_t addHotspotVisitMinTimePathMutationIndex(       const MutationsFlags config) noexcept;
constexpr uint32_t addDepotVisitMinTimePathMutationIndex(         const MutationsFlags config) noexcept;
constexpr uint32_t removeVisitMinTimePathMutationIndex(           const MutationsFlags config) noexcept;
constexpr uint32_t removeHotspotVisitMinTimePathMutationIndex(    const MutationsFlags config) noexcept;
constexpr uint32_t removeDepotVisitMinTimePathMutationIndex(      const MutationsFlags config) noexcept;
constexpr uint32_t removeVisitsWindowedMinTimePathMutationIndex(  const MutationsFlags config) noexcept;
constexpr uint32_t swapPathMinTimePathMutationIndex(              const MutationsFlags config) noexcept;
constexpr uint32_t swapVisitsMinTimePathMutationIndex(            const MutationsFlags config) noexcept;
constexpr uint32_t swapMultipleVisitsMinTimePathMutationIndex(    const MutationsFlags config) noexcept;
constexpr uint32_t moveVisitMinTimePathMutationIndex(             const MutationsFlags config) noexcept;
constexpr uint32_t simpleInversionMinTimePathMutationIndex(       const MutationsFlags config) noexcept;
constexpr uint32_t addVisitMinChargePathMutationIndex(            const MutationsFlags config) noexcept;
constexpr uint32_t addHotspotVisitMinChargePathMutationIndex(     const MutationsFlags config) noexcept;
constexpr uint32_t addDepotVisitMinChargePathMutationIndex(       const MutationsFlags config) noexcept;
constexpr uint32_t removeVisitMinChargePathMutationIndex(         const MutationsFlags config) noexcept;
constexpr uint32_t removeHotspotVisitMinChargePathMutationIndex(  const MutationsFlags config) noexcept;
constexpr uint32_t removeDepotVisitMinChargePathMutationIndex(    const MutationsFlags config) noexcept;
constexpr uint32_t removeVisitsWindowedMinChargePathMutationIndex(const MutationsFlags config) noexcept;
constexpr uint32_t swapPathMinChargePathMutationIndex(            const MutationsFlags config) noexcept;
constexpr uint32_t swapVisitsMinChargePathMutationIndex(          const MutationsFlags config) noexcept;
constexpr uint32_t swapMultipleVisitsMinChargePathMutationIndex(  const MutationsFlags config) noexcept;
constexpr uint32_t moveVisitMinChargePathMutationIndex(           const MutationsFlags config) noexcept;
constexpr uint32_t simpleInversionMinChargePathMutationIndex(     const MutationsFlags config) noexcept;
/**
 * @applyMutation
 *
 * @brief
 * Allows for the application of a given mutation.
 *
 * @templates
 * MUTAT_CONFIG: Controls which mutation operators the Firefly algorithm will run
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @parameters
 * cur_plan: The plan to mutate
 * mutation_ind: The index of the mutation to perform
 * max_dwell_time_hotspot: The max dwell time allowed at a hotspot
 * max_dwell_time_depot: The max dwell time allowed at the depot
 * tweak_dwell_time_max_change: The total possible dwell time change when doing tweakDwellTime mutation
 * graph: The graph to plan over
 * rand_gen: The random number generator to use
 *
 * @return
 * The mutated plan or an empty pointer if no plan was made.
 **/
template<MutationsFlags MUTAT_CONFIG, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  applyMutation(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                const uint32_t                                mutation_ind,
                const uint32_t                                max_dwell_time_hotspot,
                const uint32_t                                max_dwell_time_depot,
                const uint32_t                                tweak_dwell_time_max_change,
                const graph::PlanningGraph&                   graph,
                RAND_GEN_TYPE&                                rand_gen) noexcept;
/**
 * @PathPickType
 *
 * @brief
 * Determines which path picking method is used.
 **/
enum class PathPickType : uint64_t
{
  /**
   * @NULL_FLAG
   *
   * This enumeration explicitly represents nothing.
   **/
  NULL_FLAG         = 0,
  RANDOM            = 1,
  MIN_TIME          = 2,
  MIN_CHARGE        = 3,
  MIN_CHARGE_NEEDED = 4,
  MIN_END_CHARGE    = 5,
  MIN_LENGTH        = 6
};
/**
 * @VisitType
 *
 * @brief
 * Determines which visit picking method is used.
 **/
enum class VisitType : uint64_t
{
  /**
   * @NULL_FLAG
   *
   * This enumeration explicitly represents nothing.
   **/
  NULL_FLAG = 0,
  RANDOM    = 1,
  HOTSPOT   = 2,
  DEPOT     = 3
};
/**
 * @pickRandPath
 *
 * @brief
 * Picks a random path that goes between the two given points.
 *
 * @parameters
 * from_node_ind: The index of the node that the travel starts at in the original graph
 * to_node_ind: The index of the node that the travel ends at in the original graph
 * graph: The graph to plan over
 * rand_gen: The random number generator to use
 *
 * @templates
 * PATH_PICK_TYPE: Determines the type of path picked
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * A randomly chosen path.
 **/
template<PathPickType PATH_PICK_TYPE, typename RAND_GEN_TYPE>
inline const graph::Path* pickRandPath(const uint32_t              from_node_ind,
                                       const uint32_t              to_node_ind,
                                       const graph::PlanningGraph& graph,
                                       RAND_GEN_TYPE&              rand_gen) noexcept;
/**
 * @pickRandVertex
 *
 * @brief
 * Picks a random vertex (depot or hotspot) in the graph.
 *
 * @parameters
 * graph: The graph to plan over
 * rand_gen: The random number generator to use
 *
 * @templates
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * The PlanningGraph index of a random vertex.
 **/
template<typename RAND_GEN_TYPE>
inline uint32_t pickRandVertex(const graph::PlanningGraph& graph,
                               RAND_GEN_TYPE&              rand_gen) noexcept;
/**
 * @pickRandHotspot
 *
 * @brief
 * Picks a random hotspot in the graph.
 *
 * @parameters
 * graph: The graph to plan over
 * rand_gen: The random number generator to use
 *
 * @templates
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * The PlanningGraph index of a random hotspot.
 **/
template<typename RAND_GEN_TYPE>
inline uint32_t pickRandHotspot(const graph::PlanningGraph& graph,
                                RAND_GEN_TYPE&              rand_gen) noexcept;
/**
 * @pickRandDwellTime
 *
 * @brief
 * Picks a random dwell time.
 *
 * @parameters
 * max_dwell_time: Max allowed dwell time
 * rand_gen: The random number generator to use
 *
 * @templates
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * The random dwell time.
 **/
template<typename RAND_GEN_TYPE>
inline uint32_t pickRandDwellTime(const uint32_t max_dwell_time,
                                  RAND_GEN_TYPE& rand_gen) noexcept;
/**
 * @pickRandInsertLocation
 *
 * @brief
 * Picks a random location to insert an action in the current time.
 *
 * @parameters
 * cur_plan_len: The length of the current plan
 * rand_gen: The random number generator to use
 *
 * @templates
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * Index of the insert location.
 **/
template<typename RAND_GEN_TYPE>
inline uint32_t pickRandInsertLocation(const uint32_t cur_plan_len,
                                       RAND_GEN_TYPE& rand_gen) noexcept;
/**
 * @addNewVisit
 *
 * @brief
 * Adds a new visit to a random location and in a random spot, paths to and from the added visit are chosen randomly,
 * and so is dwell time.
 *
 * @parameters
 * cur_plan: Plan before mutation (flattened)
 * graph: The graph to plan over
 * max_dwell_time_hotspot: The max dwell time allowed at a hotspot
 * max_dwell_time_depot: The max dwell time allowed at the depot
 * rand_gen: The random number generator to use
 *
 * @templates
 * PATH_PICK_TYPE: Determines the type of path picked
 * VISIT_PICK_TYPE: Determines which visit picking method is used
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * The mutated plan or an empty pointer if no plan was made.
 **/
template<PathPickType PATH_PICK_TYPE, VisitType VISIT_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  addNewVisit(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
              const graph::PlanningGraph&                   graph,
              const uint32_t                                max_dwell_time_hotspot,
              const uint32_t                                max_dwell_time_depot,
              RAND_GEN_TYPE&                                rand_gen) noexcept;
/**
 * @removeVisit
 *
 * @brief
 * Removes a visit from a random location in the current plan, avoiding the first action of each agent.
 *
 * @parameters
 * cur_plan: Plan before mutation (flattened)
 * graph: The graph to plan over
 * rand_gen: The random number generator to use
 *
 * @templates
 * PATH_PICK_TYPE: Determines the type of path picked
 * VISIT_PICK_TYPE: Determines which visit picking method is used
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * The mutated plan or an empty pointer if no plan was made.
 **/
template<PathPickType PATH_PICK_TYPE, VisitType VISIT_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  removeVisit(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
              const graph::PlanningGraph&                   graph,
              RAND_GEN_TYPE&                                rand_gen) noexcept;
/**
 * @removeVisitsWindowed
 *
 * @brief
 * Removes all visit non-agent plan start visits that are between to randomly selected locations.
 *
 * @parameters
 * cur_plan: Plan before mutation (flattened)
 * graph: The graph to plan over
 * rand_gen: The random number generator to use
 *
 * @templates
 * PATH_PICK_TYPE: Determines the type of path picked
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * The mutated plan or an empty pointer if no plan was made.
 **/
template<PathPickType PATH_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  removeVisitsWindowed(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                       const graph::PlanningGraph&                   graph,
                       RAND_GEN_TYPE&                                rand_gen) noexcept;
/**
 * @changeDwellTime
 *
 * @brief
 * Randomly reinitializes a randomly chosen dwell time.
 *
 * @parameters
 * cur_plan: Plan before mutation (flattened)
 * graph: The graph to plan over
 * max_dwell_time_hotspot: The max dwell time allowed at a hotspot
 * max_dwell_time_depot: The max dwell time allowed at the depot
 * rand_gen: The random number generator to use
 *
 * @templates
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * The mutated plan or an empty pointer if no plan was made.
 **/
template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  changeDwellTime(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                  const graph::PlanningGraph&                   graph,
                  const uint32_t                                max_dwell_time_hotspot,
                  const uint32_t                                max_dwell_time_depot,
                  RAND_GEN_TYPE&                                rand_gen) noexcept;
/**
 * @tweakDwellTime
 *
 * @brief
 * Add a random about to a dwell time to a randomly chosen dwell time.
 * Capped and dwell time bounds. Random amount is a normal distribution with zero mean.
 *
 * @parameters
 * cur_plan: Plan before mutation (flattened)
 * graph: The graph to plan over
 * max_dwell_time_hotspot: The max dwell time allowed at a hotspot
 * max_dwell_time_depot: The max dwell time allowed at the depot
 * percent_max_for_noise_std: The percentage of the total possible dwell time to make the std of the dwell time addition distribution
 * rand_gen: The random number generator to use
 *
 * @templates
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * The mutated plan or an empty pointer if no plan was made.
 **/
template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  tweakDwellTime(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                 const graph::PlanningGraph&                   graph,
                 const uint32_t                                max_dwell_time_hotspot,
                 const uint32_t                                max_dwell_time_depot,
                 const uint32_t                                tweak_dwell_time_max_change,
                 RAND_GEN_TYPE&                                rand_gen) noexcept;
/**
 * @swapPath
 *
 * @brief
 * Swaps the path leading up to a randomly selected action wit another path that starts and ends in the same locations.
 *
 * @parameters
 * cur_plan: Plan before mutation (flattened)
 * graph: The graph to plan over
 * rand_gen: The random number generator to use
 *
 * @templates
 * PATH_PICK_TYPE: Determines the type of path picked
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * First: The mutated plan (flattened).
 * Second: True iff at least one path was successfully swapped.
 **/
template<PathPickType PATH_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  swapPath(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
           const graph::PlanningGraph&                   graph,
           RAND_GEN_TYPE&                                rand_gen) noexcept;
/**
 * @swapVisits
 *
 * @brief
 * Swaps the positions of two visits, randomly setting paths to be good.
 *
 * @parameters
 * cur_plan: Plan before mutation (flattened)
 * graph: The graph to plan over
 * rand_gen: The random number generator to use
 *
 * @templates
 * PATH_PICK_TYPE: Determines the type of path picked
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * The mutated plan or an empty pointer if no plan was made.
 **/
template<PathPickType PATH_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  swapVisits(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
             const graph::PlanningGraph&                   graph,
             RAND_GEN_TYPE&                                rand_gen) noexcept;
/**
 * @swapMultipleVisits
 *
 * @brief
 * Swaps the positions of two chains of visits, randomly setting paths to be good.
 *
 * @parameters
 * cur_plan: Plan before mutation (flattened)
 * graph: The graph to plan over
 * rand_gen: The random number generator to use
 *
 * @templates
 * PATH_PICK_TYPE: Determines the type of path picked
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * The mutated plan or an empty pointer if no plan was made.
 **/
template<PathPickType PATH_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  swapMultipleVisits(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                     const graph::PlanningGraph&                   graph,
                     RAND_GEN_TYPE&                                rand_gen) noexcept;
/**
 * @moveVisit
 *
 * @brief
 * Moves one visit to a random location, randomly setting paths to be good.
 *
 * @parameters
 * cur_plan: Plan before mutation (flattened)
 * graph: The graph to plan over
 * rand_gen: The random number generator to use
 *
 * @templates
 * PATH_PICK_TYPE: Determines the type of path picked
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * The mutated plan or an empty pointer if no plan was made.
 **/
template<PathPickType PATH_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  moveVisit(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
            const graph::PlanningGraph&                   graph,
            RAND_GEN_TYPE&                                rand_gen) noexcept;
/**
 * @simpleInversionMutation
 *
 * @brief
 * Randomly picks a sequence of visits, reverses its order and moves it somewhere random.
 *
 * @parameters
 * cur_plan: Plan before mutation (flattened)
 * graph: The graph to plan over
 * rand_gen: The random number generator to use
 *
 * @templates
 * PATH_PICK_TYPE: Determines the type of path picked
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 * RAND_GEN_TYPE: The type of the random number generator
 *
 * @return
 * The mutated plan or an empty pointer if no plan was made.
 **/
template<PathPickType PATH_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  simpleInversionMutation(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                          const graph::PlanningGraph&                   graph,
                          RAND_GEN_TYPE&                                rand_gen) noexcept;
} // mo
} // plan


constexpr bool plan::mo::nullFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::NULL_FLAG == (config bitand MutationsFlags::NULL_FLAG);
}

constexpr bool plan::mo::addVisitMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_ADD_VISIT_MUTATION == (config bitand MutationsFlags::USE_ADD_VISIT_MUTATION);
}

constexpr bool plan::mo::addHotspotVisitMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_ADD_HOTSPOT_VISIT_MUTATION == (config bitand MutationsFlags::USE_ADD_HOTSPOT_VISIT_MUTATION);
}

constexpr bool plan::mo::addDepotVisitMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_ADD_DEPOT_VISIT_MUTATION == (config bitand MutationsFlags::USE_ADD_DEPOT_VISIT_MUTATION);
}

constexpr bool plan::mo::removeVisitMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_REMOVE_VISIT_MUTATION == (config bitand MutationsFlags::USE_REMOVE_VISIT_MUTATION);
}

constexpr bool plan::mo::removeHotspotVisitMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_REMOVE_HOTSPOT_VISIT_MUTATION == (config bitand MutationsFlags::USE_REMOVE_HOTSPOT_VISIT_MUTATION);
}

constexpr bool plan::mo::removeDepotVisitMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_REMOVE_DEPOT_VISIT_MUTATION == (config bitand MutationsFlags::USE_REMOVE_DEPOT_VISIT_MUTATION);
}

constexpr bool plan::mo::removeVisitsWindowedMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_REMOVE_VISITS_WINDOWED_MUTATION == (config bitand MutationsFlags::USE_REMOVE_VISITS_WINDOWED_MUTATION);
}

constexpr bool plan::mo::changeDwellTimeMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_CHANGE_DWELL_TIME_MUTATION == (config bitand MutationsFlags::USE_CHANGE_DWELL_TIME_MUTATION);
}

constexpr bool plan::mo::tweakDwellTimeMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_TWEAK_DWELL_TIME_MUTATION == (config bitand MutationsFlags::USE_TWEAK_DWELL_TIME_MUTATION);
}

constexpr bool plan::mo::swapPathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_SWAP_PATH_MUTATION == (config bitand MutationsFlags::USE_SWAP_PATH_MUTATION);
}

constexpr bool plan::mo::swapVisitsMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_SWAP_VISITS_MUTATION == (config bitand MutationsFlags::USE_SWAP_VISITS_MUTATION);
}

constexpr bool plan::mo::swapMultipleVisitsMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_SWAP_MULTIPLE_VISITS_MUTATION == (config bitand MutationsFlags::USE_SWAP_MULTIPLE_VISITS_MUTATION);
}

constexpr bool plan::mo::moveVisitMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_MOVE_VISIT_MUTATION == (config bitand MutationsFlags::USE_MOVE_VISIT_MUTATION);
}

constexpr bool plan::mo::simpleInversionMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_SIMPLE_INVERSION_MUTATION == (config bitand MutationsFlags::USE_SIMPLE_INVERSION_MUTATION);
}

constexpr bool plan::mo::addVisitMinTimePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_ADD_VISIT_MIN_TIME_PATH_MUTATION == (config bitand MutationsFlags::USE_ADD_VISIT_MIN_TIME_PATH_MUTATION);
}

constexpr bool plan::mo::addHotspotVisitMinTimePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_ADD_HOTSPOT_VISIT_MIN_TIME_PATH_MUTATION == (config bitand MutationsFlags::USE_ADD_HOTSPOT_VISIT_MIN_TIME_PATH_MUTATION);
}

constexpr bool plan::mo::addDepotVisitMinTimePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_ADD_DEPOT_VISIT_MIN_TIME_PATH_MUTATION == (config bitand MutationsFlags::USE_ADD_DEPOT_VISIT_MIN_TIME_PATH_MUTATION);
}

constexpr bool plan::mo::removeVisitMinTimePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_REMOVE_VISIT_MIN_TIME_PATH_MUTATION == (config bitand MutationsFlags::USE_REMOVE_VISIT_MIN_TIME_PATH_MUTATION);
}

constexpr bool plan::mo::removeHotspotVisitMinTimePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_REMOVE_HOTSPOT_VISIT_MIN_TIME_PATH_MUTATION == (config bitand MutationsFlags::USE_REMOVE_HOTSPOT_VISIT_MIN_TIME_PATH_MUTATION);
}

constexpr bool plan::mo::removeDepotVisitMinTimePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_REMOVE_DEPOT_VISIT_MIN_TIME_PATH_MUTATION == (config bitand MutationsFlags::USE_REMOVE_DEPOT_VISIT_MIN_TIME_PATH_MUTATION);
}

constexpr bool plan::mo::removeVisitsWindowedMinTimePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_REMOVE_VISITS_WINDOWED_MIN_TIME_PATH_MUTATION == (config bitand MutationsFlags::USE_REMOVE_VISITS_WINDOWED_MIN_TIME_PATH_MUTATION);
}

constexpr bool plan::mo::swapPathMinTimePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_SWAP_PATH_MIN_TIME_PATH_MUTATION == (config bitand MutationsFlags::USE_SWAP_PATH_MIN_TIME_PATH_MUTATION);
}

constexpr bool plan::mo::swapVisitsMinTimePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_SWAP_VISITS_MIN_TIME_PATH_MUTATION == (config bitand MutationsFlags::USE_SWAP_VISITS_MIN_TIME_PATH_MUTATION);
}

constexpr bool plan::mo::swapMultipleVisitsMinTimePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_SWAP_MULTIPLE_VISITS_MIN_TIME_PATH_MUTATION == (config bitand MutationsFlags::USE_SWAP_MULTIPLE_VISITS_MIN_TIME_PATH_MUTATION);
}

constexpr bool plan::mo::moveVisitMinTimePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_MOVE_VISIT_MIN_TIME_PATH_MUTATION == (config bitand MutationsFlags::USE_MOVE_VISIT_MIN_TIME_PATH_MUTATION);
}

constexpr bool plan::mo::simpleInversionMinTimePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_SIMPLE_INVERSION_MIN_TIME_PATH_MUTATION == (config bitand MutationsFlags::USE_SIMPLE_INVERSION_MIN_TIME_PATH_MUTATION);
}

constexpr bool plan::mo::addVisitMinChargePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_ADD_VISIT_MIN_CHARGE_PATH_MUTATION == (config bitand MutationsFlags::USE_ADD_VISIT_MIN_CHARGE_PATH_MUTATION);
}

constexpr bool plan::mo::addHotspotVisitMinChargePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_ADD_HOTSPOT_VISIT_MIN_CHARGE_PATH_MUTATION == (config bitand MutationsFlags::USE_ADD_HOTSPOT_VISIT_MIN_CHARGE_PATH_MUTATION);
}

constexpr bool plan::mo::addDepotVisitMinChargePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_ADD_DEPOT_VISIT_MIN_CHARGE_PATH_MUTATION == (config bitand MutationsFlags::USE_ADD_DEPOT_VISIT_MIN_CHARGE_PATH_MUTATION);
}

constexpr bool plan::mo::removeVisitMinChargePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_REMOVE_VISIT_MIN_CHARGE_PATH_MUTATION == (config bitand MutationsFlags::USE_REMOVE_VISIT_MIN_CHARGE_PATH_MUTATION);
}

constexpr bool plan::mo::removeHotspotVisitMinChargePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_REMOVE_HOTSPOT_VISIT_MIN_CHARGE_PATH_MUTATION == (config bitand MutationsFlags::USE_REMOVE_HOTSPOT_VISIT_MIN_CHARGE_PATH_MUTATION);
}

constexpr bool plan::mo::removeDepotVisitMinChargePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_REMOVE_DEPOT_VISIT_MIN_CHARGE_PATH_MUTATION == (config bitand MutationsFlags::USE_REMOVE_DEPOT_VISIT_MIN_CHARGE_PATH_MUTATION);
}

constexpr bool plan::mo::removeVisitsWindowedMinChargePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_REMOVE_VISITS_WINDOWED_MIN_CHARGE_PATH_MUTATION == (config bitand MutationsFlags::USE_REMOVE_VISITS_WINDOWED_MIN_CHARGE_PATH_MUTATION);
}

constexpr bool plan::mo::swapPathMinChargePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_SWAP_PATH_MIN_CHARGE_PATH_MUTATION == (config bitand MutationsFlags::USE_SWAP_PATH_MIN_CHARGE_PATH_MUTATION);
}

constexpr bool plan::mo::swapVisitsMinChargePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_SWAP_VISITS_MIN_CHARGE_PATH_MUTATION == (config bitand MutationsFlags::USE_SWAP_VISITS_MIN_CHARGE_PATH_MUTATION);
}

constexpr bool plan::mo::swapMultipleVisitsMinChargePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_SWAP_MULTIPLE_VISITS_MIN_CHARGE_PATH_MUTATION == (config bitand MutationsFlags::USE_SWAP_MULTIPLE_VISITS_MIN_CHARGE_PATH_MUTATION);
}

constexpr bool plan::mo::moveVisitMinChargePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_MOVE_VISIT_MIN_CHARGE_PATH_MUTATION == (config bitand MutationsFlags::USE_MOVE_VISIT_MIN_CHARGE_PATH_MUTATION);
}

constexpr bool plan::mo::simpleInversionMinChargePathMutationFlag(const MutationsFlags config) noexcept
{
  return MutationsFlags::USE_SIMPLE_INVERSION_MIN_CHARGE_PATH_MUTATION == (config bitand MutationsFlags::USE_SIMPLE_INVERSION_MIN_CHARGE_PATH_MUTATION);
}

constexpr bool plan::mo::usingAnyMutations(const MutationsFlags config) noexcept
{
  return numberMutations(config) != 0;
}

constexpr uint32_t plan::mo::numberMutations(const MutationsFlags config) noexcept
{
  return 0
    + ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((moveVisitMinTimePathMutationFlag(             config)) ? 1 : 0)
    + ((simpleInversionMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addVisitMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMinChargePathMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMinChargePathMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMinChargePathMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMinChargePathMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMinChargePathMutationFlag(    config)) ? 1 : 0)
    + ((removeVisitsWindowedMinChargePathMutationFlag(config)) ? 1 : 0)
    + ((swapPathMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((swapVisitsMinChargePathMutationFlag(          config)) ? 1 : 0)
    + ((swapMultipleVisitsMinChargePathMutationFlag(  config)) ? 1 : 0)
    + ((moveVisitMinChargePathMutationFlag(           config)) ? 1 : 0)
    + ((simpleInversionMinChargePathMutationFlag(     config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::addVisitMutationIndex(const MutationsFlags config) noexcept
{
  return ((addVisitMutationFlag(config)) ? 0 : -1);
}

constexpr uint32_t plan::mo::addHotspotVisitMutationIndex(const MutationsFlags config) noexcept
{
  return (not addHotspotVisitMutationFlag(config)) ? -2 :
      ((addVisitMutationFlag(            config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::addDepotVisitMutationIndex(const MutationsFlags config) noexcept
{
  return (not addDepotVisitMutationFlag(config)) ? -3 :
      ((addVisitMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(     config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::removeVisitMutationIndex(const MutationsFlags config) noexcept
{
  return (not removeVisitMutationFlag(config)) ? -4 :
      ((addVisitMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(       config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::removeHotspotVisitMutationIndex(const MutationsFlags config) noexcept
{
  return (not removeHotspotVisitMutationFlag(config)) ? -5 :
      ((addVisitMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMutationFlag(         config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::removeDepotVisitMutationIndex(const MutationsFlags config) noexcept
{
  return (not removeDepotVisitMutationFlag(config)) ? -6 :
      ((addVisitMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(  config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::removeVisitsWindowedMutationIndex(const MutationsFlags config) noexcept
{
  return (not removeVisitsWindowedMutationFlag(config)) ? -7 :
      ((addVisitMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(    config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::changeDwellTimeMutationIndex(const MutationsFlags config) noexcept
{
  return (not changeDwellTimeMutationFlag(config)) ? -8 :
      ((addVisitMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(    config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::tweakDwellTimeMutationIndex(const MutationsFlags config) noexcept
{
  return (not tweakDwellTimeMutationFlag(config)) ? -9 :
      ((addVisitMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(    config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(     config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::swapPathMutationIndex(const MutationsFlags config) noexcept
{
  return (not swapPathMutationFlag(config)) ? -10 :
      ((addVisitMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(    config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(     config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(      config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::swapVisitsMutationIndex(const MutationsFlags config) noexcept
{
  return (not swapVisitsMutationFlag(config)) ? -11 :
      ((addVisitMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(    config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(     config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(      config)) ? 1 : 0)
    + ((swapPathMutationFlag(            config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::swapMultipleVisitsMutationIndex(const MutationsFlags config) noexcept
{
  return (not swapMultipleVisitsMutationFlag(config)) ? -12 :
      ((addVisitMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(    config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(     config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(      config)) ? 1 : 0)
    + ((swapPathMutationFlag(            config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(          config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::moveVisitMutationIndex(const MutationsFlags config) noexcept
{
  return (not moveVisitMutationFlag(config)) ? -13 :
      ((addVisitMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(    config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(     config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(      config)) ? 1 : 0)
    + ((swapPathMutationFlag(            config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(          config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(  config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::simpleInversionMutationIndex(const MutationsFlags config) noexcept
{
  return (not simpleInversionMutationFlag(config)) ? -14 :
      ((addVisitMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(    config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(     config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(      config)) ? 1 : 0)
    + ((swapPathMutationFlag(            config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(          config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(  config)) ? 1 : 0)
    + ((moveVisitMutationFlag(           config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::addVisitMinTimePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not addVisitMinTimePathMutationFlag(config)) ? -14:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::addHotspotVisitMinTimePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not addHotspotVisitMinTimePathMutationFlag(config)) ? -15:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::addDepotVisitMinTimePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not addDepotVisitMinTimePathMutationFlag(config)) ? -16:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::removeVisitMinTimePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not removeVisitMinTimePathMutationFlag(config)) ? -17:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::removeHotspotVisitMinTimePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not removeHotspotVisitMinTimePathMutationFlag(config)) ? -18:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::removeDepotVisitMinTimePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not removeDepotVisitMinTimePathMutationFlag(config)) ? -19:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::removeVisitsWindowedMinTimePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not removeVisitsWindowedMinTimePathMutationFlag(config)) ? -20:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::swapPathMinTimePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not swapPathMinTimePathMutationFlag(config)) ? -21:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::swapVisitsMinTimePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not swapVisitsMinTimePathMutationFlag(config)) ? -22:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::swapMultipleVisitsMinTimePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not swapMultipleVisitsMinTimePathMutationFlag(config)) ? -23:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::moveVisitMinTimePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not moveVisitMinTimePathMutationFlag(config)) ? -24:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::simpleInversionMinTimePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not simpleInversionMinTimePathMutationFlag(config)) ? -25:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((moveVisitMinTimePathMutationFlag(             config)) ? 1 : 0);
}


constexpr uint32_t plan::mo::addVisitMinChargePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not addVisitMinChargePathMutationFlag(config)) ? -26:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((moveVisitMinTimePathMutationFlag(             config)) ? 1 : 0)
    + ((simpleInversionMinTimePathMutationFlag(       config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::addHotspotVisitMinChargePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not addHotspotVisitMinChargePathMutationFlag(config)) ? -27:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((moveVisitMinTimePathMutationFlag(             config)) ? 1 : 0)
    + ((simpleInversionMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addVisitMinChargePathMutationFlag(            config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::addDepotVisitMinChargePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not addDepotVisitMinChargePathMutationFlag(config)) ? -28:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((moveVisitMinTimePathMutationFlag(             config)) ? 1 : 0)
    + ((simpleInversionMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addVisitMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMinChargePathMutationFlag(     config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::removeVisitMinChargePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not removeVisitMinChargePathMutationFlag(config)) ? -29:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((moveVisitMinTimePathMutationFlag(             config)) ? 1 : 0)
    + ((simpleInversionMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addVisitMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMinChargePathMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMinChargePathMutationFlag(       config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::removeHotspotVisitMinChargePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not removeHotspotVisitMinChargePathMutationFlag(config)) ? -30:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((moveVisitMinTimePathMutationFlag(             config)) ? 1 : 0)
    + ((simpleInversionMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addVisitMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMinChargePathMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMinChargePathMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMinChargePathMutationFlag(         config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::removeDepotVisitMinChargePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not removeDepotVisitMinChargePathMutationFlag(config)) ? -31:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((moveVisitMinTimePathMutationFlag(             config)) ? 1 : 0)
    + ((simpleInversionMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addVisitMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMinChargePathMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMinChargePathMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMinChargePathMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMinChargePathMutationFlag(  config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::removeVisitsWindowedMinChargePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not removeVisitsWindowedMinChargePathMutationFlag(config)) ? -32:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((moveVisitMinTimePathMutationFlag(             config)) ? 1 : 0)
    + ((simpleInversionMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addVisitMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMinChargePathMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMinChargePathMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMinChargePathMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMinChargePathMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMinChargePathMutationFlag(    config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::swapPathMinChargePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not swapPathMinChargePathMutationFlag(config)) ? -33:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((moveVisitMinTimePathMutationFlag(             config)) ? 1 : 0)
    + ((simpleInversionMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addVisitMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMinChargePathMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMinChargePathMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMinChargePathMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMinChargePathMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMinChargePathMutationFlag(    config)) ? 1 : 0)
    + ((removeVisitsWindowedMinChargePathMutationFlag(config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::swapVisitsMinChargePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not swapVisitsMinChargePathMutationFlag(config)) ? -34:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((moveVisitMinTimePathMutationFlag(             config)) ? 1 : 0)
    + ((simpleInversionMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addVisitMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMinChargePathMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMinChargePathMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMinChargePathMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMinChargePathMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMinChargePathMutationFlag(    config)) ? 1 : 0)
    + ((removeVisitsWindowedMinChargePathMutationFlag(config)) ? 1 : 0)
    + ((swapPathMinChargePathMutationFlag(            config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::swapMultipleVisitsMinChargePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not swapMultipleVisitsMinChargePathMutationFlag(config)) ? -35:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((moveVisitMinTimePathMutationFlag(             config)) ? 1 : 0)
    + ((simpleInversionMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addVisitMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMinChargePathMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMinChargePathMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMinChargePathMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMinChargePathMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMinChargePathMutationFlag(    config)) ? 1 : 0)
    + ((removeVisitsWindowedMinChargePathMutationFlag(config)) ? 1 : 0)
    + ((swapPathMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((swapVisitsMinChargePathMutationFlag(          config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::moveVisitMinChargePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not moveVisitMinChargePathMutationFlag(config)) ? -36:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((moveVisitMinTimePathMutationFlag(             config)) ? 1 : 0)
    + ((simpleInversionMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addVisitMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMinChargePathMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMinChargePathMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMinChargePathMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMinChargePathMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMinChargePathMutationFlag(    config)) ? 1 : 0)
    + ((removeVisitsWindowedMinChargePathMutationFlag(config)) ? 1 : 0)
    + ((swapPathMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((swapVisitsMinChargePathMutationFlag(          config)) ? 1 : 0)
    + ((swapMultipleVisitsMinChargePathMutationFlag(  config)) ? 1 : 0);
}

constexpr uint32_t plan::mo::simpleInversionMinChargePathMutationIndex(const MutationsFlags config) noexcept
{
  return (not simpleInversionMinChargePathMutationFlag(config)) ? -37:
      ((addVisitMutationFlag(                         config)) ? 1 : 0)
    + ((addHotspotVisitMutationFlag(                  config)) ? 1 : 0)
    + ((addDepotVisitMutationFlag(                    config)) ? 1 : 0)
    + ((removeVisitMutationFlag(                      config)) ? 1 : 0)
    + ((removeHotspotVisitMutationFlag(               config)) ? 1 : 0)
    + ((removeDepotVisitMutationFlag(                 config)) ? 1 : 0)
    + ((removeVisitsWindowedMutationFlag(             config)) ? 1 : 0)
    + ((changeDwellTimeMutationFlag(                  config)) ? 1 : 0)
    + ((tweakDwellTimeMutationFlag(                   config)) ? 1 : 0)
    + ((swapPathMutationFlag(                         config)) ? 1 : 0)
    + ((swapVisitsMutationFlag(                       config)) ? 1 : 0)
    + ((swapMultipleVisitsMutationFlag(               config)) ? 1 : 0)
    + ((moveVisitMutationFlag(                        config)) ? 1 : 0)
    + ((simpleInversionMutationFlag(                  config)) ? 1 : 0)
    + ((addVisitMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((addHotspotVisitMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addDepotVisitMinTimePathMutationFlag(         config)) ? 1 : 0)
    + ((removeVisitMinTimePathMutationFlag(           config)) ? 1 : 0)
    + ((removeHotspotVisitMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((removeDepotVisitMinTimePathMutationFlag(      config)) ? 1 : 0)
    + ((removeVisitsWindowedMinTimePathMutationFlag(  config)) ? 1 : 0)
    + ((swapPathMinTimePathMutationFlag(              config)) ? 1 : 0)
    + ((swapVisitsMinTimePathMutationFlag(            config)) ? 1 : 0)
    + ((swapMultipleVisitsMinTimePathMutationFlag(    config)) ? 1 : 0)
    + ((moveVisitMinTimePathMutationFlag(             config)) ? 1 : 0)
    + ((simpleInversionMinTimePathMutationFlag(       config)) ? 1 : 0)
    + ((addVisitMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((addHotspotVisitMinChargePathMutationFlag(     config)) ? 1 : 0)
    + ((addDepotVisitMinChargePathMutationFlag(       config)) ? 1 : 0)
    + ((removeVisitMinChargePathMutationFlag(         config)) ? 1 : 0)
    + ((removeHotspotVisitMinChargePathMutationFlag(  config)) ? 1 : 0)
    + ((removeDepotVisitMinChargePathMutationFlag(    config)) ? 1 : 0)
    + ((removeVisitsWindowedMinChargePathMutationFlag(config)) ? 1 : 0)
    + ((swapPathMinChargePathMutationFlag(            config)) ? 1 : 0)
    + ((swapVisitsMinChargePathMutationFlag(          config)) ? 1 : 0)
    + ((swapMultipleVisitsMinChargePathMutationFlag(  config)) ? 1 : 0)
    + ((moveVisitMinChargePathMutationFlag(           config)) ? 1 : 0);
}

template<plan::mo::MutationsFlags MUTAT_CONFIG, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::mo::applyMutation(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>&                                             cur_plan,
                          const uint32_t                                                                            mutation_ind,
                          const uint32_t                                                                            max_dwell_time_hotspot,
                          const uint32_t                                                                            max_dwell_time_depot,
                          const uint32_t                                                                            tweak_dwell_time_max_change,
                          const graph::PlanningGraph&                                                               graph,
                          RAND_GEN_TYPE&                                                                            rand_gen) noexcept
{
  switch(mutation_ind)
  {
    case addVisitMutationIndex(MUTAT_CONFIG): // 0
      assert(addVisitMutationFlag(MUTAT_CONFIG));
      return addNewVisit<mo::PathPickType::RANDOM,mo::VisitType::RANDOM,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, max_dwell_time_hotspot, max_dwell_time_depot, rand_gen);
      break;
    case addHotspotVisitMutationIndex(MUTAT_CONFIG): // 1
      assert(addHotspotVisitMutationFlag(MUTAT_CONFIG));
      return addNewVisit<PathPickType::RANDOM,VisitType::HOTSPOT,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, max_dwell_time_hotspot, max_dwell_time_depot, rand_gen);
      break;
    case addDepotVisitMutationIndex(MUTAT_CONFIG): // 2
      assert(addDepotVisitMutationFlag(MUTAT_CONFIG));
      return addNewVisit<PathPickType::RANDOM,VisitType::DEPOT,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, max_dwell_time_hotspot, max_dwell_time_depot, rand_gen);
      break;
    case removeVisitMutationIndex(MUTAT_CONFIG): // 3
      assert(removeVisitMutationFlag(MUTAT_CONFIG));
      return removeVisit<PathPickType::RANDOM,VisitType::RANDOM,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case removeHotspotVisitMutationIndex(MUTAT_CONFIG): // 4
      assert(removeHotspotVisitMutationFlag(MUTAT_CONFIG));
      return removeVisit<PathPickType::RANDOM,VisitType::HOTSPOT,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case removeDepotVisitMutationIndex(MUTAT_CONFIG): // 5
      assert(removeDepotVisitMutationFlag(MUTAT_CONFIG));
      return removeVisit<PathPickType::RANDOM,VisitType::DEPOT,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case removeVisitsWindowedMutationIndex(MUTAT_CONFIG): // 6
      assert(removeVisitsWindowedMutationFlag(MUTAT_CONFIG));
      return removeVisitsWindowed<PathPickType::RANDOM,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case changeDwellTimeMutationIndex(MUTAT_CONFIG): // 7
      assert(changeDwellTimeMutationFlag(MUTAT_CONFIG));
      return changeDwellTime<MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, max_dwell_time_hotspot, max_dwell_time_depot, rand_gen);
    break;
    case tweakDwellTimeMutationIndex(MUTAT_CONFIG): // 8
      assert(tweakDwellTimeMutationFlag(MUTAT_CONFIG));
      return tweakDwellTime<MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, max_dwell_time_hotspot, max_dwell_time_depot, tweak_dwell_time_max_change, rand_gen);
    break;
    case swapPathMutationIndex(MUTAT_CONFIG): // 9
      assert(swapPathMutationFlag(MUTAT_CONFIG));
      return swapPath<PathPickType::RANDOM,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case swapVisitsMutationIndex(MUTAT_CONFIG): // 10
      assert(swapVisitsMutationFlag(MUTAT_CONFIG));
      return swapVisits<PathPickType::RANDOM,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case swapMultipleVisitsMutationIndex(MUTAT_CONFIG): // 11
      assert(swapMultipleVisitsMutationFlag(MUTAT_CONFIG));
      return swapMultipleVisits<PathPickType::RANDOM,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case moveVisitMutationIndex(MUTAT_CONFIG): // 12
      assert(moveVisitMutationFlag(MUTAT_CONFIG));
      return moveVisit<PathPickType::RANDOM,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case simpleInversionMutationIndex(MUTAT_CONFIG): // 13
      assert(simpleInversionMutationFlag(MUTAT_CONFIG));
      return simpleInversionMutation<PathPickType::RANDOM,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case addVisitMinTimePathMutationIndex(MUTAT_CONFIG): // 14
      assert(addVisitMinTimePathMutationFlag(MUTAT_CONFIG));
      return addNewVisit<PathPickType::MIN_TIME,VisitType::RANDOM,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, max_dwell_time_hotspot, max_dwell_time_depot, rand_gen);
      break;
    break;
    case addHotspotVisitMinTimePathMutationIndex(MUTAT_CONFIG): // 15
      assert(addHotspotVisitMinTimePathMutationFlag(MUTAT_CONFIG));
      return addNewVisit<PathPickType::MIN_TIME,VisitType::HOTSPOT,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, max_dwell_time_hotspot, max_dwell_time_depot, rand_gen);
      break;
    break;
    case addDepotVisitMinTimePathMutationIndex(MUTAT_CONFIG): // 16
      assert(addDepotVisitMinTimePathMutationFlag(MUTAT_CONFIG));
      return addNewVisit<PathPickType::MIN_TIME,VisitType::DEPOT,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, max_dwell_time_hotspot, max_dwell_time_depot, rand_gen);
      break;
    break;
    case removeVisitMinTimePathMutationIndex(MUTAT_CONFIG): // 17
      assert(removeVisitMinTimePathMutationFlag(MUTAT_CONFIG));
      return removeVisit<PathPickType::MIN_TIME,VisitType::RANDOM,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case removeHotspotVisitMinTimePathMutationIndex(MUTAT_CONFIG): // 18
      assert(removeHotspotVisitMinTimePathMutationFlag(MUTAT_CONFIG));
      return removeVisit<PathPickType::MIN_TIME,VisitType::HOTSPOT,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case removeDepotVisitMinTimePathMutationIndex(MUTAT_CONFIG): // 19
      assert(removeDepotVisitMinTimePathMutationFlag(MUTAT_CONFIG));
      return removeVisit<PathPickType::MIN_TIME,VisitType::DEPOT,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case removeVisitsWindowedMinTimePathMutationIndex(MUTAT_CONFIG): // 20
      assert(removeVisitsWindowedMinTimePathMutationFlag(MUTAT_CONFIG));
      return removeVisitsWindowed<PathPickType::MIN_TIME,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case swapPathMinTimePathMutationIndex(MUTAT_CONFIG): // 21
      assert(swapPathMinTimePathMutationFlag(MUTAT_CONFIG));
      return swapPath<PathPickType::MIN_TIME,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case swapVisitsMinTimePathMutationIndex(MUTAT_CONFIG): // 22
      assert(swapVisitsMinTimePathMutationFlag(MUTAT_CONFIG));
      return swapVisits<PathPickType::MIN_TIME,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case swapMultipleVisitsMinTimePathMutationIndex(MUTAT_CONFIG): // 23
      assert(swapMultipleVisitsMinTimePathMutationFlag(MUTAT_CONFIG));
      return swapMultipleVisits<PathPickType::MIN_TIME,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case moveVisitMinTimePathMutationIndex(MUTAT_CONFIG): // 24
      assert(moveVisitMinTimePathMutationFlag(MUTAT_CONFIG));
      return moveVisit<PathPickType::MIN_TIME,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case simpleInversionMinTimePathMutationIndex(MUTAT_CONFIG): // 25
      assert(simpleInversionMinTimePathMutationFlag(MUTAT_CONFIG));
      return simpleInversionMutation<PathPickType::MIN_TIME,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case addVisitMinChargePathMutationIndex(MUTAT_CONFIG): // 26
      assert(addVisitMinChargePathMutationFlag(MUTAT_CONFIG));
      return addNewVisit<PathPickType::MIN_CHARGE,VisitType::RANDOM,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, max_dwell_time_hotspot, max_dwell_time_depot, rand_gen);
      break;
    break;
    case addHotspotVisitMinChargePathMutationIndex(MUTAT_CONFIG): // 27
      assert(addHotspotVisitMinChargePathMutationFlag(MUTAT_CONFIG));
      return addNewVisit<PathPickType::MIN_CHARGE,VisitType::HOTSPOT,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, max_dwell_time_hotspot, max_dwell_time_depot, rand_gen);
      break;
    break;
    case addDepotVisitMinChargePathMutationIndex(MUTAT_CONFIG): // 28
      assert(addDepotVisitMinChargePathMutationFlag(MUTAT_CONFIG));
      return addNewVisit<PathPickType::MIN_CHARGE,VisitType::DEPOT,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, max_dwell_time_hotspot, max_dwell_time_depot, rand_gen);
      break;
    break;
    case removeVisitMinChargePathMutationIndex(MUTAT_CONFIG): // 29
      assert(removeVisitMinChargePathMutationFlag(MUTAT_CONFIG));
      return removeVisit<PathPickType::MIN_CHARGE,VisitType::RANDOM,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case removeHotspotVisitMinChargePathMutationIndex(MUTAT_CONFIG): // 30
      assert(removeHotspotVisitMinChargePathMutationFlag(MUTAT_CONFIG));
      return removeVisit<PathPickType::MIN_CHARGE,VisitType::HOTSPOT,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case removeDepotVisitMinChargePathMutationIndex(MUTAT_CONFIG):
      assert(removeDepotVisitMinChargePathMutationFlag(MUTAT_CONFIG));
      return removeVisit<PathPickType::MIN_CHARGE,VisitType::DEPOT,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case removeVisitsWindowedMinChargePathMutationIndex(MUTAT_CONFIG):
      assert(removeVisitsWindowedMinChargePathMutationFlag(MUTAT_CONFIG));
      return removeVisitsWindowed<PathPickType::MIN_CHARGE,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case swapPathMinChargePathMutationIndex(MUTAT_CONFIG):
      assert(swapPathMinChargePathMutationFlag(MUTAT_CONFIG));
      return swapPath<PathPickType::MIN_CHARGE,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case swapVisitsMinChargePathMutationIndex(MUTAT_CONFIG):
      assert(swapVisitsMinChargePathMutationFlag(MUTAT_CONFIG));
      return swapVisits<PathPickType::MIN_CHARGE,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case swapMultipleVisitsMinChargePathMutationIndex(MUTAT_CONFIG):
      assert(swapMultipleVisitsMinChargePathMutationFlag(MUTAT_CONFIG));
      return swapMultipleVisits<PathPickType::MIN_CHARGE,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case moveVisitMinChargePathMutationIndex(MUTAT_CONFIG):
      assert(moveVisitMinChargePathMutationFlag(MUTAT_CONFIG));
      return moveVisit<PathPickType::MIN_CHARGE,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    case simpleInversionMinChargePathMutationIndex(MUTAT_CONFIG):
      assert(simpleInversionMinChargePathMutationFlag(MUTAT_CONFIG));
      return simpleInversionMutation<PathPickType::MIN_CHARGE,MAX_PLAN_VISITS,EIG_OPTIONS>(cur_plan, graph, rand_gen);
      break;
    default:
      assert(false);
      break;
  };
  return std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
}

template<plan::mo::PathPickType PATH_PICK_TYPE, typename RAND_GEN_TYPE>
inline const graph::Path* plan::mo::pickRandPath(const uint32_t              from_node_ind,
                                                 const uint32_t              to_node_ind,
                                                 const graph::PlanningGraph& graph,
                                                 RAND_GEN_TYPE&              rand_gen) noexcept
{
  assert(not graph.cgetPointToPointPaths()(from_node_ind, to_node_ind).empty());
  if constexpr(PATH_PICK_TYPE == PathPickType::RANDOM)
  {
    const uint32_t num_options = graph.cgetPointToPointPaths()(from_node_ind, to_node_ind).size();
    if(num_options == 1)
    {
      return &graph.cgetPointToPointPaths()(from_node_ind, to_node_ind).front();
    }
    std::uniform_int_distribution<uint32_t> rand_dist(0, num_options - 1);
    return &graph.cgetPointToPointPaths()(from_node_ind, to_node_ind)[rand_dist(rand_gen)];
  }
  else if constexpr(PATH_PICK_TYPE == PathPickType::MIN_TIME)
  {
    return &graph.minTravelTimePath(from_node_ind, to_node_ind);
  }
  else if constexpr(PATH_PICK_TYPE == PathPickType::MIN_CHARGE)
  {
    return &graph.minChargePath(from_node_ind, to_node_ind);
  }
  else if constexpr(PATH_PICK_TYPE == PathPickType::MIN_CHARGE_NEEDED)
  {
    return &graph.minChargeNecessaryPath(from_node_ind, to_node_ind);
  }
  else if constexpr(PATH_PICK_TYPE == PathPickType::MIN_END_CHARGE)
  {
    return &graph.minEndChargePath(from_node_ind, to_node_ind);
  }
  else if constexpr(PATH_PICK_TYPE == PathPickType::MIN_LENGTH)
  {
    return &graph.minLengthPath(from_node_ind, to_node_ind);
  }
  else
  {
    assert(false);
    return nullptr;
  }
}

template<typename RAND_GEN_TYPE>
inline uint32_t plan::mo::pickRandVertex(const graph::PlanningGraph& graph,
                                         RAND_GEN_TYPE&              rand_gen) noexcept
{
  std::uniform_int_distribution<uint32_t> rand_dist(0, graph.numberVertices() - 1);
  return rand_dist(rand_gen);
}

template<typename RAND_GEN_TYPE>
inline uint32_t plan::mo::pickRandHotspot(const graph::PlanningGraph& graph,
                                          RAND_GEN_TYPE&              rand_gen) noexcept
{
  std::uniform_int_distribution<uint32_t> rand_dist(1, graph.numberVertices() - 1);
  return rand_dist(rand_gen);
}

template<typename RAND_GEN_TYPE>
inline uint32_t plan::mo::pickRandDwellTime(const uint32_t max_dwell_time,
                                            RAND_GEN_TYPE& rand_gen) noexcept
{
  std::uniform_int_distribution<uint32_t> dwell_time_dist(0, max_dwell_time);
  return dwell_time_dist(rand_gen);
}

template<typename RAND_GEN_TYPE>
inline uint32_t plan::mo::pickRandInsertLocation(const uint32_t cur_plan_len,
                                                 RAND_GEN_TYPE& rand_gen) noexcept
{
  std::uniform_int_distribution<uint32_t> add_location_rand_dist(1, cur_plan_len);
  return add_location_rand_dist(rand_gen);
}

template<plan::mo::PathPickType PATH_PICK_TYPE, plan::mo::VisitType VISIT_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::mo::addNewVisit(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                        const graph::PlanningGraph&                   graph,
                        const uint32_t                                max_dwell_time_hotspot,
                        const uint32_t                                max_dwell_time_depot,
                        RAND_GEN_TYPE&                                rand_gen) noexcept
{
  const uint32_t cur_plan_size = cur_plan.size();

  if constexpr(Eigen::Dynamic != MAX_PLAN_VISITS)
  {
    if(MAX_PLAN_VISITS == cur_plan_size)
    {
      return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
    }
  }

  // Find insert location
  const uint32_t insert_ind = pickRandInsertLocation<RAND_GEN_TYPE>(cur_plan_size, rand_gen);
  // Find vertex it insert, dwell time, and path
  uint32_t insert_vert;
  if constexpr(VisitType::RANDOM == VISIT_PICK_TYPE)
  {
    insert_vert = pickRandVertex<RAND_GEN_TYPE>(graph, rand_gen);
  }
  else if constexpr(VisitType::HOTSPOT == VISIT_PICK_TYPE)
  {
    insert_vert = pickRandHotspot<RAND_GEN_TYPE>(graph, rand_gen);
  }
  else if constexpr(VisitType::DEPOT == VISIT_PICK_TYPE)
  {
    insert_vert = 0;
  }
  else
  {
    assert(false);
  }

  if(((cur_plan_size != insert_ind) and (nullptr != cur_plan[insert_ind].prev_path) and (cur_plan[insert_ind].vertex_ind == insert_vert)) or
     (cur_plan[insert_ind - 1].vertex_ind == insert_vert))
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }
  uint32_t insert_dwell_time;
  if constexpr(VisitType::RANDOM == VISIT_PICK_TYPE)
  {
    insert_dwell_time = pickRandDwellTime<RAND_GEN_TYPE>((graph::isDepot(insert_vert)) ? max_dwell_time_depot : max_dwell_time_hotspot, rand_gen);
  }
  else if constexpr(VisitType::HOTSPOT == VISIT_PICK_TYPE)
  {
    insert_dwell_time = pickRandDwellTime<RAND_GEN_TYPE>(max_dwell_time_hotspot, rand_gen);
  }
  else if constexpr(VisitType::DEPOT == VISIT_PICK_TYPE)
  {
    insert_dwell_time = pickRandDwellTime<RAND_GEN_TYPE>(max_dwell_time_depot, rand_gen);
  }

  const graph::Path* insert_path = pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(cur_plan[insert_ind - 1].vertex_ind, insert_vert, graph, rand_gen);
  // Insert the action
  const uint32_t                                          output_size = cur_plan_size + 1;
  std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> output      = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(output_size);
  output->topRows(insert_ind) = cur_plan.topRows(insert_ind);
  (*output)[insert_ind] = Action(insert_vert, insert_path, insert_dwell_time);
  output->bottomRows(cur_plan_size - insert_ind) = cur_plan.bottomRows(cur_plan_size - insert_ind);

  assert((*output)[insert_ind - 1].vertex_ind == (*output)[insert_ind].prev_path->from_vertex);
  // Update next action to have a path that starts in the right place
  if((output_size != (insert_ind + 1)) and
     (nullptr     != (*output)[insert_ind + 1].prev_path) and
     (insert_vert != (*output)[insert_ind + 1].prev_path->from_vertex))
  {
    (*output)[insert_ind + 1] = Action((*output)[insert_ind + 1].vertex_ind,
                                       pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(insert_vert, (*output)[insert_ind + 1].vertex_ind, graph, rand_gen),
                                       (*output)[insert_ind + 1].dwell_time);
  }
  assert((output_size == (insert_ind + 1)) or (nullptr == (*output)[insert_ind + 1].prev_path) or (insert_vert == (*output)[insert_ind + 1].prev_path->from_vertex));

  return output;
}

template<plan::mo::PathPickType PATH_PICK_TYPE, plan::mo::VisitType VISIT_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::mo::removeVisit(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                        const graph::PlanningGraph&                   graph,
                        RAND_GEN_TYPE&                                rand_gen) noexcept
{
  const uint32_t cur_plan_len = cur_plan.size();

  // Find all visits that are valid for removal
  std::vector<uint32_t> removal_inds;
  removal_inds.reserve(cur_plan_len);
  for(auto cur_plan_ind = 1; cur_plan_ind != cur_plan_len; ++cur_plan_ind)
  {
    if constexpr(VisitType::HOTSPOT == VISIT_PICK_TYPE)
    {
      if(not graph::isHotspot(cur_plan[cur_plan_ind].vertex_ind))
      {
        continue;
      }
    }
    if constexpr(VisitType::DEPOT == VISIT_PICK_TYPE)
    {
      if(not graph::isDepot(cur_plan[cur_plan_ind].vertex_ind))
      {
        continue;
      }
    }
    if((nullptr != cur_plan[cur_plan_ind].prev_path) and
       ((cur_plan_len == (cur_plan_ind + 1)) or
        (nullptr      == cur_plan[cur_plan_ind + 1].prev_path) or
        (cur_plan[cur_plan_ind - 1].vertex_ind != cur_plan[cur_plan_ind + 1].vertex_ind)))
    {
      removal_inds.emplace_back(cur_plan_ind);
    }
  }
  // Check that there is a valid action to remove
  const uint32_t num_for_remove = removal_inds.size();
  if(0 == num_for_remove)
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }
  // Pick an action for removal
  std::uniform_int_distribution<uint32_t> rand_dist(0, num_for_remove - 1);
  const uint32_t to_remove_ind = removal_inds[rand_dist(rand_gen)];
  // Copy all but the removed one
  std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> output(std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(cur_plan_len - 1));
  output->topRows(to_remove_ind) = cur_plan.topRows(to_remove_ind);
  if((cur_plan_len                           != (to_remove_ind + 1)) and
     (nullptr                                != cur_plan[to_remove_ind + 1].prev_path) and
     (cur_plan[to_remove_ind - 1].vertex_ind != cur_plan[to_remove_ind + 1].prev_path->from_vertex))
  {
    (*output)[to_remove_ind] = Action(cur_plan[to_remove_ind + 1].vertex_ind,
                                      pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(cur_plan[to_remove_ind - 1].vertex_ind,
                                                                                 cur_plan[to_remove_ind + 1].vertex_ind,
                                                                                 graph,
                                                                                 rand_gen),
                                      cur_plan[to_remove_ind + 1].dwell_time);
    output->bottomRows(cur_plan_len - to_remove_ind - 2) = cur_plan.bottomRows(cur_plan_len - to_remove_ind - 2);
  }
  else
  {
    output->bottomRows(cur_plan_len - to_remove_ind - 1) = cur_plan.bottomRows(cur_plan_len - to_remove_ind - 1);
  }

  return output;
}

template<plan::mo::PathPickType PATH_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::mo::removeVisitsWindowed(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                                 const graph::PlanningGraph&                   graph,
                                 RAND_GEN_TYPE&                                rand_gen) noexcept
{
  const uint32_t cur_plan_len = cur_plan.size();

  // Find all visits that are valid for removal
  std::vector<typename PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>::const_iterator> removal_its;
  removal_its.reserve(cur_plan_len);
  const auto cur_plan_end = cur_plan.cend();
  for(auto cur_plan_it = cur_plan.cbegin(); cur_plan_it != cur_plan_end; ++cur_plan_it)
  {
    if(nullptr != (*cur_plan_it).prev_path)
    {
      removal_its.emplace_back(cur_plan_it);
    }
  }
  // Check that there is a valid action to remove
  const uint32_t num_for_remove = removal_its.size();
  if(0 == num_for_remove)
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }
  if(1 == num_for_remove) // Just remove the one
  {
    std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> output        = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(cur_plan_len - 1);
    const uint32_t                                          to_remove_ind = std::distance(cur_plan.cbegin(), removal_its.front());
    output->topRows(to_remove_ind) = cur_plan.topRows(to_remove_ind);
    if((cur_plan_end                                 != std::next(removal_its.front())) and
       (nullptr                                      != (*std::next(removal_its.front())).prev_path) and
       ((*std::prev(removal_its.front())).vertex_ind != (*std::next(removal_its.front())).prev_path->from_vertex))
    {
      (*output)[to_remove_ind] = Action((*std::next(removal_its.front())).vertex_ind,
                                        pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>((*std::prev(removal_its.front())).vertex_ind,
                                                                                   (*std::next(removal_its.front())).vertex_ind,
                                                                                   graph,
                                                                                   rand_gen),
                                        (*std::next(removal_its.front())).dwell_time);
      output->bottomRows(cur_plan_len - to_remove_ind - 2) = cur_plan.bottomRows(cur_plan_len - to_remove_ind - 2);
    }
    else
    {
      output->bottomRows(cur_plan_len - to_remove_ind - 1) = cur_plan.bottomRows(cur_plan_len - to_remove_ind - 1);
    }

    return output;
  }
  // Pick actions for removal
  std::array<typename PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>::const_iterator,2> to_remove;
  std::sample(removal_its.cbegin(), removal_its.cend(), to_remove.begin(), 2, rand_gen);
  // Copy all but the removed ones
  std::vector<Action> output;
  output.reserve(cur_plan_len - 2);
  typename PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>::const_iterator to_remove_first;
  typename PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>::const_iterator to_remove_second;
  if(to_remove[0] < to_remove[1])
  {
    to_remove_first  = to_remove[0];
    to_remove_second = to_remove[1];
  }
  else // end comes first
  {
    to_remove_first  = to_remove[1];
    to_remove_second = to_remove[0];
  }
  // Copy up to first remove
  output.insert(output.begin(), cur_plan.cbegin(), to_remove_first);
  // Selective copy up to second remove
  for(auto cur_plan_it = std::next(to_remove_first); cur_plan_it != to_remove_second; ++cur_plan_it)
  {
    if(nullptr == cur_plan_it->prev_path)
    {
      output.emplace_back(*cur_plan_it);
    }
  }
  if((cur_plan_end != to_remove_second) and (cur_plan_end != std::next(to_remove_second)))
  {
    if(output.back().vertex_ind == std::next(to_remove_second)->vertex_ind)
    {
      return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
    }
    // Copy after remove
    if((cur_plan_end             != std::next(to_remove_second)) and
       (nullptr                  != std::next(to_remove_second)->prev_path) and
       (output.back().vertex_ind != std::next(to_remove_second)->prev_path->from_vertex))
    {
      output.emplace_back(Action(std::next(to_remove_second)->vertex_ind,
                                 pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                            std::next(to_remove_second)->vertex_ind,
                                                                            graph,
                                                                            rand_gen),
                                 std::next(to_remove_second)->dwell_time));
      output.insert(output.end(), std::next(to_remove_second, 2), cur_plan_end);
    }
    else
    {
      output.insert(output.end(), std::next(to_remove_second), cur_plan_end);
    }
  }

  return std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(Eigen::Map<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(output.data(), output.size()));
}

template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::mo::changeDwellTime(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                            const graph::PlanningGraph&                   graph,
                            const uint32_t                                max_dwell_time_hotspot,
                            const uint32_t                                max_dwell_time_depot,
                            RAND_GEN_TYPE&                                rand_gen) noexcept
{
  const uint32_t cur_plan_len = cur_plan.size();

  // Pick visit to modify
  std::uniform_int_distribution<uint32_t> rand_dist(0, cur_plan_len - 1);
  const uint32_t                          to_mod_ind = rand_dist(rand_gen);
  const uint32_t                          new_dwell_time = pickRandDwellTime<RAND_GEN_TYPE>((graph::isDepot(cur_plan[to_mod_ind].vertex_ind)) ? max_dwell_time_depot : max_dwell_time_hotspot, rand_gen);
  if(new_dwell_time == cur_plan[to_mod_ind].dwell_time)
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }
  // Copy new plan over
  std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> output = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(cur_plan);
  (*output)[to_mod_ind].dwell_time = new_dwell_time;

  return output;
}

template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::mo::tweakDwellTime(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                           const graph::PlanningGraph&                   graph,
                           const uint32_t                                max_dwell_time_hotspot,
                           const uint32_t                                max_dwell_time_depot,
                           const uint32_t                                tweak_dwell_time_max_change,
                           RAND_GEN_TYPE&                                rand_gen) noexcept
{
  const uint32_t cur_plan_len = cur_plan.size();

  // Pick visit to modify
  std::uniform_int_distribution<uint32_t> rand_dist(0, cur_plan_len - 1);
  const uint32_t                          to_mod_ind = rand_dist(rand_gen);
  // Make new dwell time
  const uint32_t max_dwell_time = (graph::isDepot(cur_plan[to_mod_ind].vertex_ind)) ? max_dwell_time_depot : max_dwell_time_hotspot;

  std::uniform_int_distribution<uint32_t> time_rand_dist(0, tweak_dwell_time_max_change);
  const uint32_t                          new_dwell_time = std::min<uint32_t>(std::max<uint32_t>(cur_plan[to_mod_ind].dwell_time + time_rand_dist(rand_gen), 0), max_dwell_time);
  if(new_dwell_time == cur_plan[to_mod_ind].dwell_time)
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }
  // Copy new plan over
  std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> output = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(cur_plan);
  (*output)[to_mod_ind].dwell_time = new_dwell_time;

  return output;
}

template<plan::mo::PathPickType PATH_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::mo::swapPath(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                     const graph::PlanningGraph&                   graph,
                     RAND_GEN_TYPE&                                rand_gen) noexcept
{
  const uint32_t cur_plan_len = cur_plan.size();

  // Find all visits that are valid for modifying
  std::vector<Eigen::Index> mod_inds;
  mod_inds.reserve(cur_plan_len);
  for(uint32_t cur_plan_ind = 0; cur_plan_ind < cur_plan_len; ++cur_plan_ind)
  {
    if(nullptr != cur_plan[cur_plan_ind].prev_path)
    {
      mod_inds.emplace_back(cur_plan_ind);
    }
  }
  // Check that there is a valid action to modify
  const uint32_t num_for_mod = mod_inds.size();
  if(0 == num_for_mod)
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }
  // Pick an action for modification
  std::uniform_int_distribution<uint32_t> rand_dist(0, num_for_mod - 1);
  const Eigen::Index to_mod_ind = mod_inds[rand_dist(rand_gen)];
  // Pick new path
  const std::vector<graph::Path>& ptp_paths     = graph.cgetPointToPointPaths()(cur_plan[to_mod_ind].prev_path->from_vertex, cur_plan[to_mod_ind].prev_path->to_vertex);
  const uint32_t                  num_ptp_paths = ptp_paths.size();
  if(num_ptp_paths < 2)
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }
  std::uniform_int_distribution<uint32_t> path_rand_dist(0, num_ptp_paths - 1);
  uint32_t                              new_path_ind = path_rand_dist(rand_gen);
  while(cur_plan[to_mod_ind].prev_path == &ptp_paths[new_path_ind])
  {
    new_path_ind = path_rand_dist(rand_gen);
  }
  // Copy all but the mod one
  std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>> output = std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(cur_plan);
  (*output)[to_mod_ind].prev_path = &ptp_paths[new_path_ind];

  return output;
}

template<plan::mo::PathPickType PATH_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::mo::swapVisits(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                       const graph::PlanningGraph&                   graph,
                       RAND_GEN_TYPE&                                rand_gen) noexcept
{
  const uint32_t cur_plan_len = cur_plan.size();
  const auto   cur_plan_end = cur_plan.cend();

  // Check that there is enough visits to perform a swap
  if(3 > cur_plan_len)
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }

  // Pick actions for swapping
  const boost::integer_range<uint32_t> cur_plan_inds(0, cur_plan_len);
  std::array<uint32_t,2> to_swap;
  std::sample(std::next(cur_plan_inds.begin()), cur_plan_inds.end(), to_swap.begin(), 2, rand_gen);

  typename PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>::const_iterator to_swap_first;
  typename PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>::const_iterator to_swap_second;
  if(to_swap[0] < to_swap[1])
  {
    to_swap_first  = cur_plan.cbegin() + to_swap[0];
    to_swap_second = cur_plan.cbegin() + to_swap[1];
  }
  else // end comes first
  {
    to_swap_first  = cur_plan.cbegin() + to_swap[1];
    to_swap_second = cur_plan.cbegin() + to_swap[0];
  }

  // Check for validity
  if(((nullptr != to_swap_second->prev_path) and (std::prev(to_swap_first)->vertex_ind == to_swap_second->vertex_ind)) or
     ((std::next(to_swap_first) != to_swap_second) and
      (((nullptr != std::next(to_swap_first)->prev_path) and (std::next(to_swap_first)->vertex_ind == to_swap_second->vertex_ind)) or
       ((nullptr != to_swap_first->prev_path) and (std::prev(to_swap_second)->vertex_ind == to_swap_first->vertex_ind)))) or
     ((std::next(to_swap_first) == to_swap_second) and (nullptr != to_swap_first->prev_path) and (to_swap_first->vertex_ind == to_swap_second->vertex_ind)) or
     ((cur_plan_end != std::next(to_swap_second)) and (nullptr != std::next(to_swap_second)->prev_path) and (std::next(to_swap_second)->vertex_ind == to_swap_first->vertex_ind)))
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }

  // Perform swap
  std::vector<Action> output;
  output.reserve(cur_plan_len);
  // Add until right before first
  output.insert(output.begin(), cur_plan.cbegin(), to_swap_first);
  // Add second in first's place
  if((nullptr                  == to_swap_second->prev_path) or
     (output.back().vertex_ind == to_swap_second->prev_path->from_vertex))
  {
    output.emplace_back(*to_swap_second);
  }
  else // not a agent plan start
  {
    output.emplace_back(to_swap_second->vertex_ind,
                        pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                   to_swap_second->vertex_ind,
                                                                   graph,
                                                                   rand_gen),
                        to_swap_second->dwell_time);
  }
  // Update following point if needed / Copy to right before second to swap
  if(std::next(to_swap_first) != to_swap_second)
  {
    if((nullptr                  == std::next(to_swap_first)->prev_path) or
       (output.back().vertex_ind == std::next(to_swap_first)->prev_path->from_vertex))
    {
      output.insert(output.end(), std::next(to_swap_first), to_swap_second);
    }
    else // Need to adjust the prev edge
    {
      output.emplace_back(std::next(to_swap_first)->vertex_ind,
                          pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                     std::next(to_swap_first)->vertex_ind,
                                                                     graph,
                                                                     rand_gen),
                          std::next(to_swap_first)->dwell_time);
      output.insert(output.end(), std::next(to_swap_first, 2), to_swap_second);
    }
  }
  // Add first in second's place
  if((nullptr                  == to_swap_first->prev_path) or
     (output.back().vertex_ind == to_swap_first->prev_path->from_vertex))
  {
    output.emplace_back(*to_swap_first);
  }
  else // not a agent plan start
  {
    output.emplace_back(to_swap_first->vertex_ind,
                        pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                   to_swap_first->vertex_ind,
                                                                   graph,
                                                                   rand_gen),
                        to_swap_first->dwell_time);
  }
  // Update following point if needed
  if(cur_plan_end != std::next(to_swap_second))
  {
    if((nullptr                  == std::next(to_swap_second)->prev_path) or
       (output.back().vertex_ind == std::next(to_swap_second)->prev_path->from_vertex))
    {
      output.insert(output.end(), std::next(to_swap_second), cur_plan_end);
    }
    else // Need to adjust the prev edge
    {
      output.emplace_back(std::next(to_swap_second)->vertex_ind,
                          pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                     std::next(to_swap_second)->vertex_ind,
                                                                     graph,
                                                                     rand_gen),
                          std::next(to_swap_second)->dwell_time);
      output.insert(output.end(), std::next(to_swap_second, 2), cur_plan_end);
    }
  }

  assert(cur_plan_len == output.size());
  return std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(Eigen::Map<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(output.data(), output.size()));
}

template<plan::mo::PathPickType PATH_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::mo::swapMultipleVisits(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                               const graph::PlanningGraph&                   graph,
                               RAND_GEN_TYPE&                                rand_gen) noexcept
{
  const uint32_t cur_plan_len = cur_plan.size();
  const auto     cur_plan_end = cur_plan.cend();

  // Check that there is enough visits to perform a swap
  if(5 > cur_plan_len)
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }
  // Pick actions for swapping
  const boost::integer_range<uint32_t> cur_plan_inds(0, cur_plan_len);
  std::array<uint32_t,4> to_swap;
  std::sample(std::next(cur_plan_inds.begin()), cur_plan_inds.end(), to_swap.begin(), 4, rand_gen);
  std::sort(std::execution::unseq, to_swap.begin(), to_swap.end());

  // Check for validity
  if(((nullptr != cur_plan[to_swap[2]].prev_path) and (cur_plan[to_swap[0] - 1].vertex_ind == cur_plan[to_swap[2]].vertex_ind)) or
     ((cur_plan_len != (to_swap[3] + 1)) and (nullptr != cur_plan[to_swap[3] + 1].prev_path) and (cur_plan[to_swap[1]].vertex_ind == cur_plan[to_swap[3] + 1].vertex_ind)))
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }
  if((to_swap[1] + 1) == to_swap[2])
  {
    if((nullptr != cur_plan[to_swap[0]].prev_path) and (cur_plan[to_swap[3]].vertex_ind == cur_plan[to_swap[0]].vertex_ind))
    {
      return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
    }
  }
  else
  {
    if(((nullptr != cur_plan[to_swap[1] + 1].prev_path) and (cur_plan[to_swap[1] + 1].vertex_ind == cur_plan[to_swap[3]].vertex_ind)) or
       ((nullptr != cur_plan[to_swap[0]].    prev_path) and (cur_plan[to_swap[2] - 1].vertex_ind == cur_plan[to_swap[0]].vertex_ind)))
    {
      return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
    }
  }
  // Perform swap
  std::vector<Action> output;
  output.reserve(cur_plan_len);
  // Add until right before first
  output.insert(output.begin(), cur_plan.cbegin(), std::next(cur_plan.cbegin(), to_swap[0]));
  // Add second chain here
  if((nullptr                  == cur_plan[to_swap[2]].prev_path) or
     (output.back().vertex_ind == cur_plan[to_swap[2]].prev_path->from_vertex))
  {
    output.emplace_back(cur_plan[to_swap[2]]);
  }
  else // not a agent plan start
  {
    output.emplace_back(cur_plan[to_swap[2]].vertex_ind,
                        pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                   cur_plan[to_swap[2]].vertex_ind,
                                                                   graph,
                                                                   rand_gen),
                        cur_plan[to_swap[2]].dwell_time);
  }
  output.insert(output.end(), std::next(cur_plan.cbegin(), 1 + to_swap[2]), std::next(cur_plan.cbegin(), 1 + to_swap[3]));
  // Add what comes in between
  if((to_swap[1] + 1) != to_swap[2])
  {
    if((nullptr                  == cur_plan[to_swap[1] + 1].prev_path) or
       (output.back().vertex_ind == cur_plan[to_swap[1] + 1].prev_path->from_vertex))
    {
      output.emplace_back(cur_plan[to_swap[1] + 1]);
    }
    else // not a agent plan start
    {
      output.emplace_back(cur_plan[to_swap[1] + 1].vertex_ind,
                          pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                     cur_plan[to_swap[1] + 1].vertex_ind,
                                                                     graph,
                                                                     rand_gen),
                          cur_plan[to_swap[1] + 1].dwell_time);
    }
    if((to_swap[1] + 2) < to_swap[2])
    {
      output.insert(output.end(), std::next(cur_plan.cbegin(), to_swap[1] + 2), std::next(cur_plan.cbegin(), to_swap[2]));
    }
  }
  // Add second chain
  if((nullptr                  == cur_plan[to_swap[0]].prev_path) or
     (output.back().vertex_ind == cur_plan[to_swap[0]].prev_path->from_vertex))
  {
    output.emplace_back(cur_plan[to_swap[0]]);
  }
  else // not a agent plan start
  {
    output.emplace_back(cur_plan[to_swap[0]].vertex_ind,
                        pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                   cur_plan[to_swap[0]].vertex_ind,
                                                                   graph,
                                                                   rand_gen),
                        cur_plan[to_swap[0]].dwell_time);
  }
  output.insert(output.end(), std::next(cur_plan.cbegin(), to_swap[0] + 1), std::next(cur_plan.cbegin(), to_swap[1] + 1));
  // Add what is left
  if(cur_plan_end != std::next(cur_plan.cbegin(), to_swap[3] + 1))
  {
    if((nullptr                  == cur_plan[to_swap[3] + 1].prev_path) or
       (output.back().vertex_ind == cur_plan[to_swap[3] + 1].prev_path->from_vertex))
    {
      output.emplace_back(*std::next(cur_plan.cbegin(), to_swap[3] + 1));
    }
    else // not a agent plan start
    {
      output.emplace_back(cur_plan[to_swap[3] + 1].vertex_ind,
                          pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                     cur_plan[to_swap[3] + 1].vertex_ind,
                                                                     graph,
                                                                     rand_gen),
                          cur_plan[to_swap[3] + 1].dwell_time);
    }
    output.insert(output.end(), std::next(cur_plan.cbegin(), to_swap[3] + 2), cur_plan_end);
  }

  assert(cur_plan_len == output.size());
  return std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(Eigen::Map<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(output.data(), output.size()));
}

template<plan::mo::PathPickType PATH_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::mo::moveVisit(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                      const graph::PlanningGraph&                   graph,
                      RAND_GEN_TYPE&                                rand_gen) noexcept
{
  const uint32_t cur_plan_len = cur_plan.size();
  const auto     cur_plan_end = cur_plan.cend();

  // Check that there is enough visits to perform a swap
  if(3 > cur_plan_len)
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }
  // Pick locations
  const boost::integer_range<uint32_t> cur_plan_inds(0, cur_plan_len);
  std::array<uint32_t,2> from_to;
  std::sample(std::next(cur_plan_inds.begin()), cur_plan_inds.end(), from_to.begin(), 2, rand_gen);

  // Check for validity
  if(((nullptr != cur_plan[from_to[0]].prev_path) and (cur_plan[from_to[0]].vertex_ind == cur_plan[from_to[1]].vertex_ind)) or
     (((from_to[1] + 1) != cur_plan_len) and (nullptr != cur_plan[from_to[1] + 1].prev_path) and (cur_plan[from_to[0]].vertex_ind == cur_plan[from_to[1] + 1].vertex_ind)) or
     (((from_to[0] + 1) != cur_plan_len) and (nullptr != cur_plan[from_to[0] + 1].prev_path) and (cur_plan[from_to[0] - 1].vertex_ind == cur_plan[from_to[0] + 1].vertex_ind)))
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }

  // Perform move
  std::vector<Action> output;
  output.reserve(cur_plan_len);
  if(from_to[0] < from_to[1])
  {
    // Copy to first location
    output.insert(output.end(), cur_plan.cbegin(), std::next(cur_plan.cbegin(), from_to[0]));
    // Copy to second location
    if((nullptr                  == std::next(cur_plan.cbegin(), from_to[0] + 1)->prev_path) or
       (output.back().vertex_ind == std::next(cur_plan.cbegin(), from_to[0] + 1)->prev_path->from_vertex))
    {
      output.emplace_back(cur_plan[from_to[0] + 1]);
    }
    else // Need to mod prev path
    {
      output.emplace_back(cur_plan[from_to[0] + 1].vertex_ind,
                          pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                     cur_plan[from_to[0] + 1].vertex_ind,
                                                                     graph,
                                                                     rand_gen),
                          cur_plan[from_to[0] + 1].dwell_time);
    }
    output.insert(output.end(), std::next(cur_plan.cbegin(), from_to[0] + 2), std::next(cur_plan.cbegin(), from_to[1] + 1));
    // Copy moved action
    if((nullptr                  == cur_plan[from_to[0]].prev_path) or
       (output.back().vertex_ind == cur_plan[from_to[0]].prev_path->from_vertex))
    {
      output.emplace_back(cur_plan[from_to[0]]);
    }
    else // Need to mod prev path
    {
      output.emplace_back(cur_plan[from_to[0]].vertex_ind,
                          pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                     cur_plan[from_to[0]].vertex_ind,
                                                                     graph,
                                                                     rand_gen),
                          cur_plan[from_to[0]].dwell_time);
    }
    // Copy the rest
    if((from_to[1] + 1) != cur_plan_len)
    {
      if((nullptr                  == cur_plan[from_to[1] + 1].prev_path) or
         (output.back().vertex_ind == cur_plan[from_to[1] + 1].prev_path->from_vertex))
      {
        output.insert(output.end(), std::next(cur_plan.cbegin(), from_to[1] + 1), cur_plan_end);
      }
      else
      {
        output.emplace_back(cur_plan[from_to[1] + 1].vertex_ind,
                            pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                       cur_plan[from_to[1] + 1].vertex_ind,
                                                                       graph,
                                                                       rand_gen),
                            cur_plan[from_to[1] + 1].dwell_time);
        output.insert(output.end(), std::next(cur_plan.cbegin(), from_to[1] + 2), cur_plan_end);
      }
    }
  }
  else // original location is second
  {
    // Copy to first location
    output.insert(output.end(), cur_plan.cbegin(), std::next(cur_plan.cbegin(), from_to[1]));
    // Copy moved action
    if((nullptr                  == cur_plan[from_to[0]].prev_path) or
       (output.back().vertex_ind == cur_plan[from_to[0]].prev_path->from_vertex))
    {
      output.emplace_back(cur_plan[from_to[0]]);
    }
    else // Need to mod prev path
    {
      output.emplace_back(cur_plan[from_to[0]].vertex_ind,
                          pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                     cur_plan[from_to[0]].vertex_ind,
                                                                     graph,
                                                                     rand_gen),
                          cur_plan[from_to[0]].dwell_time);
    }
    // Copy to second location
    if((from_to[1] + 1) != from_to[0])
    {
      if((nullptr                  == cur_plan[from_to[1] + 1].prev_path) or
         (output.back().vertex_ind == cur_plan[from_to[1] + 1].prev_path->from_vertex))
      {
        output.emplace_back(cur_plan[from_to[1] + 1]);
      }
      else // Need to mod prev path
      {
        output.emplace_back(cur_plan[from_to[1] + 1].vertex_ind,
                            pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                       cur_plan[from_to[1] + 1].vertex_ind,
                                                                       graph,
                                                                       rand_gen),
                            cur_plan[from_to[1] + 1].dwell_time);
      }
      output.insert(output.end(), std::next(cur_plan.cbegin(), from_to[1] + 2), std::next(cur_plan.cbegin(), from_to[0]));
    }
    // Copy the rest
    output.insert(output.end(), std::next(cur_plan.cbegin(), from_to[0] + 1), cur_plan_end);
  }

  assert(cur_plan_len == output.size());
  return std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(Eigen::Map<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(output.data(), output.size()));
}

template<plan::mo::PathPickType PATH_PICK_TYPE, Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS, typename RAND_GEN_TYPE>
inline std::unique_ptr<plan::PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>
  plan::mo::simpleInversionMutation(const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>& cur_plan,
                                    const graph::PlanningGraph&                   graph,
                                    RAND_GEN_TYPE&                                rand_gen) noexcept
{
  const uint32_t cur_plan_len = cur_plan.size();
  const auto     cur_plan_end = cur_plan.cend();

  // Check that there is enough visits to perform a swap
  if(4 > cur_plan_len)
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }
  // Pick locations
  const boost::integer_range<uint32_t> cur_plan_inds(0, cur_plan_len);
  std::array<uint32_t,2> rev_seq;
  std::sample(std::next(cur_plan_inds.begin()), std::prev(cur_plan_inds.end()), rev_seq.begin(), 2, rand_gen);
  std::sort(std::execution::unseq, rev_seq.begin(), rev_seq.end());

  const uint32_t len_of_rev_seq = rev_seq[1] - rev_seq[0] + 1;
  uint32_t       move_ind       = pickRandInsertLocation<RAND_GEN_TYPE>(cur_plan_len - len_of_rev_seq - 1, rand_gen);
  bool           move_comes_first;
  if(rev_seq[0] <= move_ind)
  {
    move_ind += len_of_rev_seq;
    move_comes_first = false;
  }
  else
  {
    move_comes_first = true;
  }
  const typename PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>::const_iterator move_it = std::next(cur_plan.cbegin(), move_ind);

  // Check for validity
  if(((nullptr != cur_plan[rev_seq[1]].prev_path) and (cur_plan[rev_seq[1]].vertex_ind == cur_plan[move_ind].vertex_ind)) or
     (((move_ind + 1) < cur_plan_len) and (nullptr != cur_plan[move_ind + 1].prev_path) and (cur_plan[rev_seq[0]].vertex_ind == cur_plan[move_ind + 1].vertex_ind)) or
     (((rev_seq[1] + 1) < cur_plan_len) and (nullptr != cur_plan[rev_seq[1] + 1].prev_path) and ((cur_plan[rev_seq[0] - 1].vertex_ind == cur_plan[rev_seq[1] + 1].vertex_ind) or (cur_plan[rev_seq[0]].vertex_ind == cur_plan[rev_seq[1] + 1].vertex_ind))) or
     ((0 == move_it->vertex_ind) and (0 == cur_plan[rev_seq[1]].vertex_ind) and (0 == cur_plan[rev_seq[1] - 1].vertex_ind)))
  {
    return std::unique_ptr<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>();
  }

  // Perform SIM
  std::vector<Action> output;
  output.reserve(cur_plan_len);
  if(move_comes_first)
  {
    // Copy to the location to move the sequence to
    output.insert(output.end(), cur_plan.cbegin(), std::next(move_it));
    // Copy the sequence
    for(auto rev_it = std::next(cur_plan.cbegin(), rev_seq[1]); rev_it != std::next(cur_plan.cbegin(), rev_seq[0] - 1); --rev_it)
    {
      if((nullptr                  == rev_it->prev_path) or
         (output.back().vertex_ind == rev_it->prev_path->from_vertex))
      {
        output.emplace_back(*rev_it);
      }
      else // Need to modify prev path
      {
        if((0 == output.back().vertex_ind) and (0 == rev_it->vertex_ind))
        {
          auto last_not_agent_plan_end = std::prev(output.end());
          for( ; nullptr == last_not_agent_plan_end->prev_path; --last_not_agent_plan_end);
          *std::next(last_not_agent_plan_end) = Action(0,
                                                       pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(last_not_agent_plan_end->vertex_ind,
                                                                                                  0,
                                                                                                  graph,
                                                                                                  rand_gen),
                                                       std::next(last_not_agent_plan_end)->dwell_time);
          output.emplace_back(rev_it->vertex_ind,
                              nullptr,
                              rev_it->dwell_time);
        }
        else
        {
          output.emplace_back(rev_it->vertex_ind,
                              pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                         rev_it->vertex_ind,
                                                                         graph,
                                                                         rand_gen),
                              rev_it->dwell_time);
        }
      }
    }
    // Copy until old seq start
    if((move_ind + 1) < rev_seq[0])
    {
      if((nullptr                  == std::next(move_it)->prev_path) or
         (output.back().vertex_ind == std::next(move_it)->prev_path->from_vertex))
      {
        output.emplace_back(*std::next(move_it));
      }
      else // Need to modify prev path
      {
        output.emplace_back(std::next(move_it)->vertex_ind,
                            pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                       std::next(move_it)->vertex_ind,
                                                                       graph,
                                                                       rand_gen),
                            std::next(move_it)->dwell_time);
      }
      output.insert(output.end(), std::next(move_it, 2), std::next(cur_plan.cbegin(), rev_seq[0]));
    }
    // Copy the rest
    if(cur_plan_len != (rev_seq[1] + 1))
    {
      if((nullptr                  == cur_plan[rev_seq[1] + 1].prev_path) or
         (output.back().vertex_ind == cur_plan[rev_seq[1] + 1].prev_path->from_vertex))
      {
        output.emplace_back(cur_plan[rev_seq[1] + 1]);
      }
      else // Need to modify prev path
      {
        output.emplace_back(cur_plan[rev_seq[1] + 1].vertex_ind,
                            pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                       cur_plan[rev_seq[1] + 1].vertex_ind,
                                                                       graph,
                                                                       rand_gen),
                            cur_plan[rev_seq[1] + 1].dwell_time);
      }
      output.insert(output.end(), std::next(cur_plan.cbegin(), rev_seq[1] + 2), cur_plan_end);
    }
  }
  else // Move location is second
  {
    // Copy to the location of the sequence
    output.insert(output.end(), cur_plan.cbegin(), std::next(cur_plan.cbegin(), rev_seq[0]));
    // Copy to the location to move the sequence to
    if((nullptr                  == cur_plan[rev_seq[1] + 1].prev_path) or
       (output.back().vertex_ind == cur_plan[rev_seq[1] + 1].prev_path->from_vertex))
    {
      output.emplace_back(cur_plan[rev_seq[1] + 1]);
    }
    else // Need to modify prev path
    {
      output.emplace_back(cur_plan[rev_seq[1] + 1].vertex_ind,
                          pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                     cur_plan[rev_seq[1] + 1].vertex_ind,
                                                                     graph,
                                                                     rand_gen),
                          cur_plan[rev_seq[1] + 1].dwell_time);
    }
    output.insert(output.end(), std::next(cur_plan.cbegin(), rev_seq[1] + 2), std::next(move_it));
    // Copy the sequence
    for(auto rev_it = std::next(cur_plan.cbegin(), rev_seq[1]); rev_it != std::next(cur_plan.cbegin(), rev_seq[0] - 1); --rev_it)
    {
      if((nullptr                  == (*rev_it).prev_path) or
         (output.back().vertex_ind == (*rev_it).prev_path->from_vertex))
      {
        output.emplace_back(*rev_it);
      }
      else // Need to modify prev path
      {
        if((0 == output.back().vertex_ind) and (0 == rev_it->vertex_ind))
        {
          auto last_not_agent_plan_end = std::prev(output.end());
          for( ; nullptr == last_not_agent_plan_end->prev_path; --last_not_agent_plan_end);
          *std::next(last_not_agent_plan_end) = Action(0,
                                                       pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(last_not_agent_plan_end->vertex_ind,
                                                                                                  0,
                                                                                                  graph,
                                                                                                  rand_gen),
                                                       std::next(last_not_agent_plan_end)->dwell_time);
          output.emplace_back(rev_it->vertex_ind,
                              nullptr,
                              rev_it->dwell_time);
        }
        else
        {
          output.emplace_back(rev_it->vertex_ind,
                              pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                         rev_it->vertex_ind,
                                                                         graph,
                                                                         rand_gen),
                              rev_it->dwell_time);
        }
      }
    }
    // Copy the rest
    if((move_ind + 1) < cur_plan_len)
    {
      if((nullptr                  == std::next(move_it)->prev_path) or
         (output.back().vertex_ind == std::next(move_it)->prev_path->from_vertex))
      {
        output.emplace_back(*std::next(move_it));
      }
      else // Need to modify prev path
      {
        output.emplace_back(std::next(move_it)->vertex_ind,
                            pickRandPath<PATH_PICK_TYPE,RAND_GEN_TYPE>(output.back().vertex_ind,
                                                                       std::next(move_it)->vertex_ind,
                                                                       graph,
                                                                       rand_gen),
                            std::next(move_it)->dwell_time);
      }
      output.insert(output.end(), std::next(move_it, 2), cur_plan_end);
    }
  }

  assert(cur_plan_len == output.size());
  return std::make_unique<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(Eigen::Map<PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>(output.data(), output.size()));
}

#endif
/* mutation_operators.hpp */
