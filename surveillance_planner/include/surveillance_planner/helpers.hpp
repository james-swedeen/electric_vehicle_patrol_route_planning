/**
 * @File: helpers.hpp
 * @Date: June 2024
 * @Author: James Swedeen
 *
 * @brief
 * Defines a few helper functions for use thought the package.
 **/

#ifndef SURVEILLANCE_PLANNING_HELPERS_HPP
#define SURVEILLANCE_PLANNING_HELPERS_HPP

/* C++ Headers */
#include<ostream>

/* Boost Headers */

/* Eigen Headers */
#include<Eigen/Core>

/* Local Headers */
#include<surveillance_planner/action.hpp>
#include<surveillance_planner/agent.hpp>
#include<surveillance_planner/hotspot.hpp>

namespace plan
{
/**
 * @ConstraintOptions
 *
 * @brief
 * Used to control when distance-based or SOC-based constraints are to be used.
 **/
enum class ConstraintOptions : uint64_t
{
  /**
   * @NULL_FLAG
   *
   * This enumeration explicitly represents nothing.
   * Will result in no constraints being used.
   **/
  NULL_FLAG = 0x0000'0000'0000'0000,
  /**
   * @USE_SOC_CONSTRAINTS
   *
   * @brief
   * Enable when you want SOC-based constraints.
   **/
  USE_SOC_CONSTRAINTS = 0x0000'0000'0000'0001,
  /**
   * @USE_DISTANCE_CONSTRAINTS
   *
   * @brief
   * Enable when you want distance-based constraints.
   **/
  USE_DISTANCE_CONSTRAINTS = 0x0000'0000'0000'0002,
  /**
   * @VERBOSE
   *
   * @note
   * Only useful when both SOC and distance-based constraints are enabled.
   *
   * @brief
   * When enabled a warning will print every time distance constraints are satisfied but SOC constraints are not, and vice versa.
   **/
  VERBOSE = 0x0000'0000'0000'0004,
  /**
   * @USE_CONSTRAINTS_IN_RESPONSE_TIME_CALCULATION
   *
   * @brief
   * When enabled the constraints will be considered during response time calculations.
   **/
  USE_CONSTRAINTS_IN_RESPONSE_TIME_CALCULATION = 0x0000'0000'0000'0008,
};
constexpr ConstraintOptions operator |(ConstraintOptions a, ConstraintOptions b)
{
  return static_cast<ConstraintOptions>(static_cast<uint64_t>(a) | static_cast<uint64_t>(b));
}
constexpr ConstraintOptions operator &(ConstraintOptions a, ConstraintOptions b)
{
  return static_cast<ConstraintOptions>(static_cast<uint64_t>(a) & static_cast<uint64_t>(b));
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
constexpr bool nullFlag(                     const ConstraintOptions config) noexcept;
constexpr bool socConstraintsFlag(           const ConstraintOptions config) noexcept;
constexpr bool distanceConstraintsFlag(      const ConstraintOptions config) noexcept;
constexpr bool verboseFlag(                  const ConstraintOptions config) noexcept;
constexpr bool constraintsInResponseTimeFlag(const ConstraintOptions config) noexcept;
/**
 * @templates
 * MAX_PLAN_VISITS: The max length possible for a plan
 * EIG_OPTIONS: Eigen storage options
 * NUM_AGENTS: The number of agents being planned for
 * NUM_HOTSPOTS: The number of hotspots
 * NUM_EVENT_POINTS: The number of event points
 **/
template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS = Eigen::ColMajor bitor Eigen::AutoAlign>
using PLAN_TYPE = Eigen::Matrix<Action,Eigen::Dynamic,1,EIG_OPTIONS,MAX_PLAN_VISITS,1>;
/**
 * @To Stream Operator
 **/
template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS = Eigen::ColMajor bitor Eigen::AutoAlign>
inline std::ostream& printPlan(std::ostream& os, const Eigen::Ref<const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& plan) noexcept;

template<Eigen::Index NUM_AGENTS, Eigen::StorageOptions EIG_OPTIONS = Eigen::ColMajor bitor Eigen::AutoAlign>
using AGENTS_VEC_TYPE = Eigen::Matrix<Agent,NUM_AGENTS,1,EIG_OPTIONS,NUM_AGENTS,1>;

template<Eigen::Index NUM_HOTSPOTS, Eigen::StorageOptions EIG_OPTIONS = Eigen::ColMajor bitor Eigen::AutoAlign>
using HOTSPOTS_VEC_TYPE = Eigen::Matrix<Hotspot,NUM_HOTSPOTS,1,EIG_OPTIONS,NUM_HOTSPOTS,1>;

template<Eigen::Index NUM_EVENT_POINTS, Eigen::StorageOptions EIG_OPTIONS = Eigen::ColMajor bitor Eigen::AutoAlign>
using EVENT_POINTS_VEC_TYPE = Eigen::Matrix<uint32_t,NUM_EVENT_POINTS,1,EIG_OPTIONS,NUM_EVENT_POINTS,1>;

/**
 * @weightedRandomSampling
 *
 * @brief
 * Samples a given number of point weighted according to the given weighting vector.
 *
 * @templates
 * NUM_TO_SAMPLE: Overrides the num_to_sample parameter at compile time
 * RAND_GEN_TYPE: The type of the random number generator
 * DERIVED: The type of the weights vector
 *
 * @parameters
 * weights: Vector of weights
 * num_to_sample: The number of samples to draw
 * rand_gen: The random number generator to use
 *
 * @return
 * Indexes into the weights that correlate to the points sampled.
 **/
template<Eigen::Index NUM_TO_SAMPLE, typename RAND_GEN_TYPE, typename DERIVED>
inline Eigen::Matrix<uint32_t,NUM_TO_SAMPLE,1,Eigen::AutoAlign|(Eigen::internal::traits<DERIVED>::Flags & Eigen::RowMajorBit ? Eigen::RowMajor : Eigen::ColMajor),(Eigen::Dynamic == NUM_TO_SAMPLE) ? Eigen::internal::traits<DERIVED>::MaxRowsAtCompileTime : NUM_TO_SAMPLE,1>
  weightedRandomSampling(const Eigen::MatrixBase<DERIVED>& weights,
                         const uint32_t                    num_to_sample,
                         RAND_GEN_TYPE&                    rand_gen) noexcept;
/**
 * @secondsToMinutes
 **/
inline uint32_t secondsToMinutes(const float seconds) noexcept;
/**
 * @minutesToSeconds
 **/
inline float minutesToSeconds(const uint32_t minutes) noexcept;
/**
 * @factorial
 **/
inline size_t factorial(const size_t n) noexcept;
} // plan


constexpr bool plan::nullFlag(const ConstraintOptions config) noexcept
{
  return ConstraintOptions::NULL_FLAG == (config bitand ConstraintOptions::NULL_FLAG);
}

constexpr bool plan::socConstraintsFlag(const ConstraintOptions config) noexcept
{
  return ConstraintOptions::USE_SOC_CONSTRAINTS == (config bitand ConstraintOptions::USE_SOC_CONSTRAINTS);
}

constexpr bool plan::distanceConstraintsFlag(const ConstraintOptions config) noexcept
{
  return ConstraintOptions::USE_DISTANCE_CONSTRAINTS == (config bitand ConstraintOptions::USE_DISTANCE_CONSTRAINTS);
}

constexpr bool plan::verboseFlag(const ConstraintOptions config) noexcept
{
  return ConstraintOptions::VERBOSE == (config bitand ConstraintOptions::VERBOSE);
}

constexpr bool plan::constraintsInResponseTimeFlag(const ConstraintOptions config) noexcept
{
  return ConstraintOptions::USE_CONSTRAINTS_IN_RESPONSE_TIME_CALCULATION == (config bitand ConstraintOptions::USE_CONSTRAINTS_IN_RESPONSE_TIME_CALCULATION);
}

template<Eigen::Index MAX_PLAN_VISITS, Eigen::StorageOptions EIG_OPTIONS>
inline std::ostream& plan::printPlan(std::ostream& os, const Eigen::Ref<const PLAN_TYPE<MAX_PLAN_VISITS,EIG_OPTIONS>>& plan) noexcept
{
  const uint32_t len = plan.size();
  uint32_t agent_ind = -1;
  for(uint32_t ind = 0; ind < len; ++ind)
  {
    if(nullptr == plan[ind].prev_path) { ++agent_ind; }
    os << "Agent " << agent_ind << "\tAction Ind: " << ind << " Action: " << plan[ind];
    if(ind != (len - 1)) { os << "\n"; }
  }
  return os;
}

template<Eigen::Index NUM_TO_SAMPLE, typename RAND_GEN_TYPE, typename DERIVED>
inline Eigen::Matrix<uint32_t,NUM_TO_SAMPLE,1,Eigen::AutoAlign|(Eigen::internal::traits<DERIVED>::Flags & Eigen::RowMajorBit ? Eigen::RowMajor : Eigen::ColMajor),(Eigen::Dynamic == NUM_TO_SAMPLE) ? Eigen::internal::traits<DERIVED>::MaxRowsAtCompileTime : NUM_TO_SAMPLE,1>
  plan::weightedRandomSampling(const Eigen::MatrixBase<DERIVED>& weights,
                               const uint32_t                    num_to_sample,
                               RAND_GEN_TYPE&                    rand_gen) noexcept
{
  static constexpr const Eigen::StorageOptions EIG_OPTIONS      = Eigen::StorageOptions(Eigen::AutoAlign|(Eigen::internal::traits<DERIVED>::Flags & Eigen::RowMajorBit ? Eigen::RowMajor : Eigen::ColMajor));
  static constexpr const Eigen::Index          DERIVED_NUM_ROWS = Eigen::internal::traits<DERIVED>::RowsAtCompileTime;
  static constexpr const Eigen::Index          DERIVED_MAX_ROWS = Eigen::internal::traits<DERIVED>::MaxRowsAtCompileTime;

  static_assert(1 == Eigen::internal::traits<DERIVED>::ColsAtCompileTime);
  assert((Eigen::Dynamic == NUM_TO_SAMPLE) or (NUM_TO_SAMPLE <= weights.rows()));
  assert(num_to_sample <= weights.rows());

  const uint32_t                         num_options = weights.rows();
  std::uniform_real_distribution<double> probability_dist(0, 1);

  if constexpr(1 == NUM_TO_SAMPLE)
  {
    std::discrete_distribution<uint32_t> sampler(weights.begin(), weights.end());
    return Eigen::Matrix<uint32_t,1,1,EIG_OPTIONS,1,1>::Constant(sampler(rand_gen));
  }
  else // Sample more then one
  {
    // Make list of all inds
    Eigen::Matrix<uint32_t,DERIVED_NUM_ROWS,1,EIG_OPTIONS,DERIVED_MAX_ROWS,1> inds(num_options);
    std::iota(inds.begin(), inds.end(), 0);

    // Selection from Efraimidis, Pavlos S., and Paul G. Spirakis. "Weighted random sampling with a reservoir." Information processing   letters 97.5 (2006): 181-185.
    const Eigen::Array< double,DERIVED_NUM_ROWS,1,EIG_OPTIONS,DERIVED_MAX_ROWS,1> u    = Eigen::Matrix<double,DERIVED_NUM_ROWS,1,EIG_OPTIONS,DERIVED_MAX_ROWS,1>::NullaryExpr(num_options, [&] () { return probability_dist(rand_gen); });
    const Eigen::Matrix<double,DERIVED_NUM_ROWS,1,EIG_OPTIONS,DERIVED_MAX_ROWS,1> keys = u.pow(weights.array().inverse());
    if constexpr(Eigen::Dynamic == NUM_TO_SAMPLE)
    {
      Eigen::Matrix<uint32_t,Eigen::Dynamic,1,EIG_OPTIONS,DERIVED_MAX_ROWS,1> output(num_to_sample);

      std::partial_sort(std::execution::unseq, inds.begin(), inds.begin() + num_to_sample, inds.end(),
                        [&] (const uint32_t i, const uint32_t j) -> bool { return keys[i] > keys[j]; });
      std::copy_n(std::execution::unseq, inds.begin(), num_to_sample, output.begin());

      return output;
    }
    else // Its fixed at compile time
    {
      Eigen::Matrix<uint32_t,NUM_TO_SAMPLE,1,EIG_OPTIONS,NUM_TO_SAMPLE,1> output;

      std::partial_sort(std::execution::unseq, inds.begin(), inds.begin() + NUM_TO_SAMPLE, inds.end(),
                        [&] (const uint32_t i, const uint32_t j) -> bool { return keys[i] > keys[j]; });
      std::copy_n(std::execution::unseq, inds.begin(), NUM_TO_SAMPLE, output.begin());

      return output;
    }
  }
}

inline uint32_t plan::secondsToMinutes(const float seconds) noexcept
{
  return uint32_t(std::ceil((float(1) / float(60)) * seconds));
}

inline float plan::minutesToSeconds(const uint32_t minutes) noexcept
{
  return float(60 * minutes);
}

inline size_t plan::factorial(const size_t n) noexcept
{
  size_t out = 1;
  for(size_t i = n; i > 1; --i)
  {
    out *= i;
  }
  return out;
}

#endif
/* helpers.hpp */
