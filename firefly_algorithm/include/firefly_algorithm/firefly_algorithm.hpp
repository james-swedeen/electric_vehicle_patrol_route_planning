/**
 * @File: firefly_algorithm.hpp
 * @Date: August 2024
 * @Author: James Swedeen
 *
 * @brief
 * Implements the Firefly Algorithm and the Bidirectional Firefly algorithm.
 **/

#ifndef FIREFLY_ALGORITHM_FIREFLY_ALGORITHM_HPP
#define FIREFLY_ALGORITHM_FIREFLY_ALGORITHM_HPP

/* C++ Headers */
#include<vector>
#include<list>
#include<memory>
#include<random>
#include<chrono>
#include<algorithm>
#include<execution>
#include<mutex>
#include<iostream> // TODO

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>


namespace fa
{
/**
 * @FaFlags
 *
 * @brief
 * Used to control parts of the Firefly algorithm.
 **/
enum FaFlags : uint64_t
{
  /**
   * @NULL_FLAG
   *
   * This enumeration explicitly represents nothing.
   **/
  NULL_FLAG = 0x0000'0000'0000'0000,
  /**
   * @USE_MUTATIONS
   *
   * Set to use randomization.
   **/
  USE_MUTATIONS = 0x0000'0000'0000'0001,
  /**
   * @USE_FIREFLY_OPERATOR
   *
   * Set to use the Firefly, steer to better solutions, operator.
   **/
  USE_FIREFLY_OPERATOR = 0x0000'0001'0000'0000,
  /**
   * @USE_BIDIRECTIONAL_FIREFLY_OPERATOR
   *
   * Set to use the Firefly, steer to better solutions, operator in bidirectional mode.
   **/
  USE_BIDIRECTIONAL_FIREFLY_OPERATOR = 0x0000'0002'0000'0000,
  /**
   * @USE_DEFAULT_LUMINOSITY_MULTIPLAYER
   *
   * Set to use the default luminosity multiplayer e^{-gamma r^m} instead of the flatter option 1/(1+gamma r^m).
   **/
  USE_DEFAULT_LUMINOSITY_MULTIPLAYER = 0x0000'0004'0000'0000,
  /**
   * @FORCE_NO_DUPLICATES
   *
   * If set there will never be any duplicate solutions in the population.
   **/
  FORCE_NO_DUPLICATES = 0x0000'0008'0000'0000,
  /**
   * @USE_VEC_FIREFLY_OPERATOR
   *
   * Set to use the Firefly, steer to better solutions, operator.
   **/
  USE_VEC_FIREFLY_OPERATOR = 0x0000'0010'0000'0000,
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
constexpr bool mutationsFlag(                           const FaFlags config) noexcept;
constexpr bool fireflyOperatorFlag(                     const FaFlags config) noexcept;
constexpr bool bidirectionalFireflyOperatorFlag(        const FaFlags config) noexcept;
constexpr bool defaultLuminosityMultiplayerFlag(        const FaFlags config) noexcept;
constexpr bool noDuplicatesFlag(                        const FaFlags config) noexcept;
constexpr bool vectorFireflyOperatorFlag(               const FaFlags config) noexcept;
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
 * DIM: The number of elements in a firefly
 **/
template<Eigen::Index DIM>
struct FaParams
{
public:
  /// General Parameters
  // The size of the population to keep after every generation
  size_t population_size;

  /// Firefly Parameters
  // The value that solution distances are raised to the power of in attractiveness calculation
  double dist_pow;
  // The value that solution distances are multiplied by in attractiveness calculation
  double dist_mult;
  // Base attractiveness multiplayer (also the max move distance)
  double base_attractiveness_mult;
  // Randomization/Mutation magnitude per a dimension
  Eigen::Matrix<double,DIM,1> mutation_magnitude;

  /// Firefly Algorithm Bidirectional Parameters
  // Minimal attractiveness multiplayer
  double min_attractiveness_mult;
  // Term that is multiplied by normalized cost because clamping in movement distance calculation
  double cost_scaling_mult;
  double from_cost_scaling_mult;
  double soft_max_mult;

  /// For K-Means selection
  size_t number_clusters;
  size_t number_cluster_iterations;

  inline FaParams()                           noexcept;
  inline FaParams(const FaParams&)            noexcept = default;
  inline FaParams(FaParams&&)                 noexcept = default;
  inline FaParams& operator=(const FaParams&) noexcept = default;
  inline FaParams& operator=(FaParams&&)      noexcept = default;
  inline ~FaParams()                          noexcept = default;
};
template<Eigen::Index DIM>
using FaParamsPtr = std::shared_ptr<FaParams<DIM>>;
/**
 * @generateFireflyPlan
 *
 * @brief
 * Uses the Firefly algorithm to plan.
 *
 * @parameters
 * params: The Firefly parameters
 * init_func: This function is responsible for producing the initial population, takes a random seed as arguments
 * cost_func: The objective function
 * stopping_condition: Tells the algorithm when to stop, takes the number of generation executed and the cost of the best plan found as arguments
 * log_func: This function will be called after every generation on the population, takes the population as arguments
 * random_seed: Seed for random action selection
 *
 * @templates
 * DIM: The number of elements in a firefly
 * CONFIG: Controls what version of the algorithm will be ran
 *
 * @return
 * The best solution found.
 **/
template<Eigen::Index DIM, fa::FaFlags CONFIG>
Eigen::Matrix<double,DIM,1>
  generateFireflyPlan(const FaParamsPtr<DIM>&                                                                                                           params,
                      const std::function<Eigen::Matrix<double,DIM,1>(const unsigned)>&                                                                 init_func,
                      const std::function<std::pair<bool,double>(const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>&)>&                                cost_func,
                      const std::function<bool(const size_t,const double)>&                                                                             stopping_condition,
                      const std::function<void(const Eigen::Matrix<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>,Eigen::Dynamic,1>&)>& log_func,
                      const unsigned                                                                                                                    random_seed = std::chrono::system_clock::now().time_since_epoch().count());
} // fa


constexpr bool fa::nullFlag(const FaFlags config) noexcept
{
  return FaFlags::NULL_FLAG == (config bitand FaFlags::NULL_FLAG);
}

constexpr bool fa::mutationsFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_MUTATIONS == (config bitand FaFlags::USE_MUTATIONS);
}

constexpr bool fa::fireflyOperatorFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_FIREFLY_OPERATOR == (config bitand FaFlags::USE_FIREFLY_OPERATOR);
}

constexpr bool fa::bidirectionalFireflyOperatorFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR == (config bitand FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR);
}

constexpr bool fa::defaultLuminosityMultiplayerFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER == (config bitand FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
}

constexpr bool fa::noDuplicatesFlag(const FaFlags config) noexcept
{
  return FaFlags::FORCE_NO_DUPLICATES == (config bitand FaFlags::FORCE_NO_DUPLICATES);
}

constexpr bool fa::vectorFireflyOperatorFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_VEC_FIREFLY_OPERATOR == (config bitand FaFlags::USE_VEC_FIREFLY_OPERATOR);
}

constexpr bool fa::elitismSelectionFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_ELITISM_SELECTION == (config bitand FaFlags::USE_ELITISM_SELECTION);
}

constexpr bool fa::fitnessSelectionFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_FITNESS_SELECTION == (config bitand FaFlags::SELECTION_BIT_FLAG);
}

constexpr bool fa::stochasticUniversalSamplingSelectionFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION == (config bitand FaFlags::SELECTION_BIT_FLAG);
}

constexpr bool fa::fitnessProportionalSelectionFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION == (config bitand FaFlags::SELECTION_BIT_FLAG);
}

constexpr bool fa::kMeansSelectionFlag(const FaFlags config) noexcept
{
  return FaFlags::USE_K_MEANS_SELECTION == (config bitand FaFlags::SELECTION_BIT_FLAG);
}

template<Eigen::Index DIM>
fa::FaParams<DIM>::FaParams() noexcept
 : population_size(0),
   dist_pow(std::numeric_limits<double>::quiet_NaN()),
   dist_mult(std::numeric_limits<double>::quiet_NaN()),
   base_attractiveness_mult(std::numeric_limits<double>::quiet_NaN()),
   mutation_magnitude(Eigen::Matrix<double,DIM,1>::Constant(std::numeric_limits<double>::quiet_NaN())),
   min_attractiveness_mult(std::numeric_limits<double>::quiet_NaN()),
   cost_scaling_mult(std::numeric_limits<double>::quiet_NaN()),
   number_clusters(std::numeric_limits<size_t>::quiet_NaN()),
   number_cluster_iterations(std::numeric_limits<size_t>::quiet_NaN())
{}

template<Eigen::Index DIM, fa::FaFlags CONFIG>
Eigen::Matrix<double,DIM,1>
  fa::generateFireflyPlan(const FaParamsPtr<DIM>&                                                                                                           params,
                          const std::function<Eigen::Matrix<double,DIM,1>(const unsigned)>&                                                                 init_func,
                          const std::function<std::pair<bool,double>(const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>&)>&                                cost_func,
                          const std::function<bool(const size_t,const double)>&                                                                             stopping_condition,
                          const std::function<void(const Eigen::Matrix<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>,Eigen::Dynamic,1>&)>& log_func,
                          const unsigned                                                                                                                    random_seed)
{
  // At least one selection and only one
  static_assert(fitnessSelectionFlag(CONFIG) or stochasticUniversalSamplingSelectionFlag(CONFIG) or fitnessProportionalSelectionFlag(CONFIG) or kMeansSelectionFlag(CONFIG) or vectorFireflyOperatorFlag(CONFIG));
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
  if constexpr(bidirectionalFireflyOperatorFlag(CONFIG))
  {
    static_assert(fireflyOperatorFlag(CONFIG) or vectorFireflyOperatorFlag(CONFIG));
  }
  if constexpr(vectorFireflyOperatorFlag(CONFIG))
  {
    static_assert(not fireflyOperatorFlag(CONFIG));
    static_assert(not fitnessSelectionFlag(CONFIG));
    static_assert(not stochasticUniversalSamplingSelectionFlag(CONFIG));
    static_assert(not fitnessProportionalSelectionFlag(CONFIG));
    static_assert(not kMeansSelectionFlag(CONFIG));
    static_assert(not mutationsFlag(CONFIG));
  }
  if constexpr(fireflyOperatorFlag(CONFIG))
  {
    static_assert(not vectorFireflyOperatorFlag(CONFIG));
  }


  Eigen::Matrix<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>,Eigen::Dynamic,1> population(params->population_size, 1);
  const boost::integer_range<size_t>                                                             population_inds(0, params->population_size);
  std::default_random_engine                                                                     rand_gen(random_seed);
  std::uniform_real_distribution<double>                                                         probability_dist(-1, 1);
  std::uniform_real_distribution<double>                                                         fab_soft_max_dist(0, params->from_cost_scaling_mult);

  /// Generate initial population
  std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
  [&] (const size_t population_ind) -> void
  {
    const Eigen::Matrix<double,DIM,1> rand_plan = init_func(random_seed + population_ind);
    const std::pair<bool,double>      cost      = cost_func(rand_plan);
    assert(cost.first);
    assert(not (rand_plan.array().abs() > 1.0e+15).any());
    assert(0 == rand_plan.array().isNaN().count());
    population[population_ind] = std::make_unique<std::pair<Eigen::Matrix<double,DIM,1>,double>>(std::move(rand_plan), cost.second);
  });
  if constexpr(noDuplicatesFlag(CONFIG))
  {
    // Remove duplicates
    std::for_each(population_inds.begin(), population_inds.end(),
    [&] (const size_t population_ind) -> void
    {
      // While there is a duplicate
      size_t attempt_count = 1;
      while(std::any_of(std::execution::par_unseq, std::next(population_inds.begin(), population_ind + 1), population_inds.end(),
                        [&] (const size_t comp_ind) -> bool
                        {
                          return (population[population_ind]->first.array() == population[comp_ind]->first.array()).all();
                        }))
      {
        const Eigen::Matrix<double,DIM,1> rand_plan = init_func(random_seed + population_ind + 1 + (params->population_size * attempt_count++));
        const std::pair<bool,double>      cost      = cost_func(rand_plan);
        assert(cost.first);
        population[population_ind] = std::make_unique<std::pair<Eigen::Matrix<double,DIM,1>,double>>(std::move(rand_plan), cost.second);
      }
    });
  }
  assert(not population.unaryExpr([] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& i) -> bool { return i->first.hasNaN(); }).any());
  /// Bookkeeping
  log_func(population);
  size_t best_sol_found_ind;//, worst_sol_found_ind;
//  {
//    const auto best_worst_ind_its = std::minmax_element(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
//                                                        [&population] (const size_t i, const size_t j) -> bool
//                                                        { return population[i]->second < population[j]->second; });
//    best_sol_found_ind  = *best_worst_ind_its.first;
//    worst_sol_found_ind = *best_worst_ind_its.second;
//  }
  {
    const auto best_ind_it = std::min_element(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
                                              [&population] (const size_t i, const size_t j) -> bool
                                              { return population[i]->second < population[j]->second; });
    best_sol_found_ind  = *best_ind_it;
  }

  for(size_t generation_count = 0; not stopping_condition(generation_count, population[best_sol_found_ind]->second); ++generation_count)
  {
    if constexpr(vectorFireflyOperatorFlag(CONFIG))
    {
      // Find distances between all solutions
      // From ind, then to ind
      Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>                      distances(      params->population_size, params->population_size);
      Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>                      luminosity_mult(params->population_size, params->population_size);
      Eigen::Matrix<Eigen::Matrix<double,DIM,1>,Eigen::Dynamic,Eigen::Dynamic> diffs(          params->population_size, params->population_size);

      std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
      [&] (const size_t from_ind) -> void
      {
        distances(      from_ind, from_ind) = 0;
        luminosity_mult(from_ind, from_ind) = 1;
        diffs(          from_ind, from_ind).setZero();
        const boost::integer_range<size_t> the_rest_of_the_population_inds(from_ind + 1, params->population_size);
        std::for_each(std::execution::par_unseq, the_rest_of_the_population_inds.begin(), the_rest_of_the_population_inds.end(),
        [&] (const size_t to_ind) -> void
        {
          // Calculation for one way
          diffs(    from_ind, to_ind) = population[to_ind]->first - population[from_ind]->first;
          distances(from_ind, to_ind) = diffs(from_ind, to_ind).norm();

          if(double(0.0) == distances(from_ind, to_ind))
          {
            luminosity_mult(from_ind, to_ind) = 1;

            distances(      to_ind, from_ind) = 0;
            luminosity_mult(to_ind, from_ind) = 1;
            diffs(          to_ind, from_ind).setZero();
          }
          else // distance is not zero
          {
//            if constexpr(bidirectionalFireflyOperatorFlag(CONFIG)) // TODO: Temp
//            {
//              diffs(from_ind, to_ind).array() /= distances(from_ind, to_ind);
//            }
            if constexpr(defaultLuminosityMultiplayerFlag(CONFIG))
            {
              luminosity_mult(from_ind, to_ind) = std::exp(-params->dist_mult * std::pow(distances(from_ind, to_ind), params->dist_pow));
            }
            else // Flatter luminosity multiplayer
            {
              luminosity_mult(from_ind, to_ind) = double(1) / (double(1) + (params->dist_mult * std::pow(distances(from_ind, to_ind), params->dist_pow)));
            }
            // Now the other way
            distances(      to_ind, from_ind) = distances(from_ind, to_ind);
            luminosity_mult(to_ind, from_ind) = luminosity_mult(from_ind, to_ind);
            diffs(          to_ind, from_ind) = -diffs(from_ind, to_ind);
          }
        });
      });

      // Find move distances
      Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> move_dists;
      Eigen::Matrix<double,DIM,           Eigen::Dynamic> move_vecs;
      if constexpr(bidirectionalFireflyOperatorFlag(CONFIG))
      {
//        const Eigen::Matrix<double,Eigen::Dynamic,1> obj_minus_min = (population.unaryExpr([] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& i) -> double { return i->second; }).array() - population[best_sol_found_ind]->second) * params->cost_scaling_mult;
//        const Eigen::Matrix<double,Eigen::Dynamic,1> obj_minus_min_p1_inv = double(1) / (obj_minus_min.array() + double(1));
//        const Eigen::Matrix<double,Eigen::Dynamic,1> obj_minus_min_clamped = obj_minus_min.array().min(params->base_attractiveness_mult).max(params->min_attractiveness_mult);
//        assert((obj_minus_min_clamped.array() >= params->min_attractiveness_mult). all());
//        assert((obj_minus_min_clamped.array() <= params->base_attractiveness_mult).all());
//        move_dists = (obj_minus_min_clamped * obj_minus_min_p1_inv.transpose()).array() * luminosity_mult.array();

//        const Eigen::Matrix<double,Eigen::Dynamic,1> obj_minus_min_exp = (-(population.unaryExpr([] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& i) -> double { return i->second; }).array() - population[best_sol_found_ind]->second) * params->cost_scaling_mult).exp();
//        move_dists = luminosity_mult.array().rowwise() * obj_minus_min_exp.array().transpose();
//        assert(std::fabs(move_dists(1, 2) - (luminosity_mult(1, 2) * obj_minus_min_exp[2])) < 1.0e-10);
//        move_dists = (move_dists.array() * params->min_attractiveness_mult).exp();
//        move_dists.array().colwise() /= move_dists.rowwise().sum().array();
//        assert(std::fabs(1.0 - move_dists.row(0).sum()) < 1.0e-10);
//        move_dists.array() *= params->base_attractiveness_mult;

        const Eigen::Matrix<double,Eigen::Dynamic,1> obj_minus_min = (population.unaryExpr([] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& i) -> double { return i->second; }).array() - population[best_sol_found_ind]->second) * params->cost_scaling_mult;

        const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> dist_cost_value = (luminosity_mult.array().rowwise() * (-obj_minus_min.array()).exp().transpose()) * Eigen::Array<double,Eigen::Dynamic,Eigen::Dynamic>::NullaryExpr(params->population_size, params->population_size, [&] () { return fab_soft_max_dist(rand_gen); });
//        Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> dist_cost_value = (luminosity_mult.array().rowwise() * (-obj_minus_min.array()).exp().transpose()) * Eigen::Array<double,Eigen::Dynamic,Eigen::Dynamic>::NullaryExpr(params->population_size, params->population_size, [&] () { return fab_soft_max_dist(rand_gen); });
//        dist_cost_value.diagonal().setZero(); // TODO: Decide if I like this ( I don't )

//        move_dists = Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>::Zero(params->population_size, params->population_size);
        move_vecs.resize(DIM, params->population_size);
        std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
        [&] (const size_t from_ind) -> void
        {
          Eigen::Index max_to_ind;
          const double max_val = dist_cost_value.row(from_ind).template maxCoeff<Eigen::NaNPropagationOptions::PropagateFast>(&max_to_ind);
          move_vecs.col(from_ind) = (params->base_attractiveness_mult * max_val) * diffs(from_ind, max_to_ind);
        });
      }
      else // Normal Firefly Algorithm
      {
        move_dists.resize(params->population_size, params->population_size);
        std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
        [&] (const size_t to_ind) -> void
        {
          std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
          [&] (const size_t from_ind) -> void
          {
            if(population[from_ind]->second > population[to_ind]->second)
            {
              move_dists(from_ind, to_ind) = luminosity_mult(from_ind, to_ind) * params->base_attractiveness_mult;
            }
            else
            {
              move_dists(from_ind, to_ind) = 0;
            }
          });
        });
      }
      // Perform movement
      Eigen::Matrix<double,DIM,Eigen::Dynamic> potential_new_population;
      if constexpr(bidirectionalFireflyOperatorFlag(CONFIG))
      {
        potential_new_population = std::move(move_vecs);
      }
      else
      {
        potential_new_population = Eigen::Matrix<double,DIM,Eigen::Dynamic>::Zero(DIM, params->population_size);
        std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
        [&] (const size_t from_ind) -> void
        {
          std::for_each(population_inds.begin(), population_inds.end(),
          [&] (const size_t to_ind) -> void
          {
            potential_new_population.col(from_ind) += (move_dists(from_ind, to_ind) * diffs(from_ind, to_ind));
          });
        });
      }
      potential_new_population.array() += Eigen::Array<double,DIM,Eigen::Dynamic>::NullaryExpr(DIM, params->population_size, [&] () { return probability_dist(rand_gen); }).colwise() * params->mutation_magnitude.array();
      std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
      [&] (const size_t from_ind) -> void
      {
        const Eigen::Matrix<double,DIM,1> new_plan = potential_new_population.col(from_ind) + population[from_ind]->first;
        // Check the plan
        const std::pair<bool,double> cost = cost_func(new_plan);
        if(cost.first)
        {
          if constexpr(elitismSelectionFlag(CONFIG))
          {
            if(from_ind == best_sol_found_ind)
            {
              if(cost.second < population[from_ind]->second)
              {
                // Store the plan
                population[from_ind]->first  = std::move(new_plan);
                population[from_ind]->second = cost.second;
              }
            }
            else
            {
              // Store the plan
              population[from_ind]->first  = std::move(new_plan);
              population[from_ind]->second = cost.second;
            }
          }
          else // No elitism
          {
            // Store the plan
            population[from_ind]->first  = std::move(new_plan);
            population[from_ind]->second = cost.second;
          }
        }
      });
    }
    else
    {
      std::vector<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>> next_generation_mutation;
      /// Apply mutations
      std::thread mutations_thread;
      if constexpr(mutationsFlag(CONFIG))
      {
        next_generation_mutation.resize(params->population_size);
        mutations_thread = std::thread([&] () -> void
        {
          std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
          [&] (const size_t population_ind) -> void
          {
            const Eigen::Matrix<double,DIM,1> mutated_plan = population[population_ind]->first +
                                                             (params->mutation_magnitude.array() * Eigen::Array<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); })).matrix();
            assert(not ((mutated_plan.array() - population[population_ind]->first.array()).abs() > params->mutation_magnitude.array()).any());
            assert(not (mutated_plan.array().abs() > 1.0e+15).any());
            assert(0 == mutated_plan.array().isNaN().count());
            if constexpr(noDuplicatesFlag(CONFIG))
            {
              // If plan unique in population
              if(not std::none_of(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
                                  [&] (const size_t comp_ind) -> bool
                                  {
                                    return (mutated_plan.array() == population[comp_ind]->first.array()).all();
                                  }))
              {
                return;
              }
            }
            // Evaluate cost
            const std::pair<bool,double> cost = cost_func(mutated_plan);
            if(cost.first)
            {
              // Add plan to next generation
              next_generation_mutation[population_ind] = std::make_unique<std::pair<Eigen::Matrix<double,DIM,1>,double>>(std::move(mutated_plan), cost.second);
            }
          });
          next_generation_mutation.erase(std::remove_if(std::execution::par_unseq, next_generation_mutation.begin(), next_generation_mutation.end(),
                                                        [] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& ptr) -> bool { return not ptr; }),
                                         next_generation_mutation.end());
          if constexpr(noDuplicatesFlag(CONFIG))
          {
            // Make sure everything is unique
            for(auto next_gen_it = std::next(next_generation_mutation.cbegin()); next_gen_it != next_generation_mutation.cend(); ++next_gen_it)
            {
              if(std::any_of(std::execution::par_unseq, next_generation_mutation.cbegin(), next_gen_it,
                             [&] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& comp_plan) -> bool
                             {
                               return ((*next_gen_it)->first.array() == comp_plan->first.array()).all();
                             }))
              {
                next_gen_it = std::prev(next_generation_mutation.erase(next_gen_it));
              }
            }
          }
        });
      }
      /// Apply crossovers
      /// Apply hybridization

      /// Apply Firefly operator
      std::vector<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>> next_generation_firefly;
      std::thread firefly_operator_thread;
      // Find distances between all solutions
      // From ind, then to ind
      Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> distances;
      if constexpr(fireflyOperatorFlag(CONFIG))
      {
        firefly_operator_thread = std::thread([&] () -> void
        {
          // Find distances between all solutions
          // From ind, then to ind
          distances.resize(                                                                        params->population_size, params->population_size);
          Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>                      luminosity_mult(params->population_size, params->population_size);
          Eigen::Matrix<Eigen::Matrix<double,DIM,1>,Eigen::Dynamic,Eigen::Dynamic> diffs(          params->population_size, params->population_size);

          std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
          [&] (const size_t from_ind) -> void
          {
            distances(from_ind, from_ind) = 0;
            const boost::integer_range<size_t> the_rest_of_the_population_inds(from_ind + 1, params->population_size);
            std::for_each(std::execution::par_unseq, the_rest_of_the_population_inds.begin(), the_rest_of_the_population_inds.end(),
            [&] (const size_t to_ind) -> void
            {
              // Calculation for one way
              diffs(    from_ind, to_ind) = population[to_ind]->first - population[from_ind]->first;
              distances(from_ind, to_ind) = diffs(from_ind, to_ind).norm();

              if constexpr(defaultLuminosityMultiplayerFlag(CONFIG))
              {
                luminosity_mult(from_ind, to_ind) = std::exp(-params->dist_mult * std::pow(distances(from_ind, to_ind), params->dist_pow));
              }
              else // Flatter luminosity multiplayer
              {
                luminosity_mult(from_ind, to_ind) = double(1) / (double(1) + (params->dist_mult * std::pow(distances(from_ind, to_ind), params->dist_pow)));
              }

              // Now the other way
              distances(      to_ind, from_ind) = distances(from_ind, to_ind);
              luminosity_mult(to_ind, from_ind) = luminosity_mult(from_ind, to_ind);
              diffs(          to_ind, from_ind) = -diffs(from_ind, to_ind);
            });
          });

          // Perform movement
          Eigen::Matrix<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>,Eigen::Dynamic,Eigen::Dynamic> next_generation_firefly_mat(params->population_size, params->population_size);
          Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> move_dists;
          if constexpr(bidirectionalFireflyOperatorFlag(CONFIG))
          {
             const Eigen::Matrix<double,Eigen::Dynamic,1> obj_minus_min = (population.unaryExpr([] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& i) -> double { return i->second; }).array() - population[best_sol_found_ind]->second) * params->cost_scaling_mult;
             const Eigen::Matrix<double,Eigen::Dynamic,1> obj_minus_min_p1_inv = double(1) / (obj_minus_min.array() + double(1));
             const Eigen::Matrix<double,Eigen::Dynamic,1> obj_minus_min_clamped = obj_minus_min.array().min(params->base_attractiveness_mult).max(params->min_attractiveness_mult);
            assert((obj_minus_min_clamped.array() >= params->min_attractiveness_mult). all());
            assert((obj_minus_min_clamped.array() <= params->base_attractiveness_mult).all());
            move_dists = (obj_minus_min_clamped * obj_minus_min_p1_inv.transpose()).array() * luminosity_mult.array();
          }
          else // Normal Firefly Algorithm
          {
            luminosity_mult.array() *= params->base_attractiveness_mult;
          }
          std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
          [&] (const size_t from_ind) -> void
          {
            std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
            [&] (const size_t to_ind) -> void
            {
              if(from_ind == to_ind) { return; }
              if constexpr(bidirectionalFireflyOperatorFlag(CONFIG))
              {
                // Find distance to move
  //              const double move_dist = distances(from_ind, to_ind)
  //                                     * luminosity_mult(from_ind, to_ind)
  //                                     * population[from_ind]->second
  //                                     * obj_p1_inv[to_ind];
  //              const double move_dist = distances(from_ind, to_ind)
  //                                     * luminosity_mult(from_ind, to_ind)
  //                                     * ((population[best_sol_found_ind]->second * population[from_ind]->second) / (population[worst_sol_found_ind]->second * population[to_ind]->second));
                // Move the plan
                const Eigen::Matrix<double,DIM,1> new_plan = population[from_ind]->first.array() + (move_dists(from_ind, to_ind) * diffs(from_ind, to_ind).array());
                //assert((new_plan - population[to_ind]->first).norm() <= (population[from_ind]->first - population[to_ind]->first).norm());
                assert(not new_plan.hasNaN());
                assert(not (new_plan.array().abs() > 1.0e+15).any());

                if constexpr(noDuplicatesFlag(CONFIG))
                {
                  // If plan unique in population
                  if(std::any_of(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
                                 [&] (const size_t comp_ind) -> bool
                                 {
                                  return (new_plan.array() == population[comp_ind]->first.array()).all();
                                 }))
                  { return; }
                }
                // Check the plan
                const std::pair<bool,double> cost = cost_func(new_plan);
                if(cost.first)
                {
                  // Store the plan
                  next_generation_firefly_mat(from_ind, to_ind) = std::make_unique<std::pair<Eigen::Matrix<double,DIM,1>,double>>(std::move(new_plan), cost.second);
                }
              }
              else // Normal Firefly algorithm
              {
                // Check if movement is desired
                if(population[from_ind]->second > population[to_ind]->second)
                {
                  // Find distance to move
                  const double move_dist = luminosity_mult(from_ind, to_ind);
                  // Move the plan
                  const Eigen::Matrix<double,DIM,1> new_plan = population[from_ind]->first.array() + (move_dist * diffs(from_ind, to_ind).array());
                  //assert((new_plan - population[to_ind]->first).norm() <= (population[from_ind]->first - population[to_ind]->first).norm());
                  assert(0 == new_plan.array().isNaN().count());
                  assert(not (new_plan.array().abs() > 1.0e+15).any());

                  if constexpr(noDuplicatesFlag(CONFIG))
                  {
                    // If plan unique in population
                    if(std::any_of(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
                                   [&] (const size_t comp_ind) -> bool
                                   {
                                    return (new_plan.array() == population[comp_ind]->first.array()).all();
                                   }))
                    { return; }
                  }
                  // Check the plan
                  const std::pair<bool,double> cost = cost_func(new_plan);
                  if(cost.first)
                  {
                    next_generation_firefly_mat(from_ind, to_ind) = std::make_unique<std::pair<Eigen::Matrix<double,DIM,1>,double>>(std::move(new_plan), cost.second);
                  }
                }
              }
            });
          });

          next_generation_firefly.reserve(params->population_size * params->population_size);
          std::for_each(population_inds.begin(), population_inds.end(),
          [&] (const size_t from_ind) -> void
          {
            std::for_each(population_inds.begin(), population_inds.end(),
            [&] (const size_t to_ind) -> void
            {
              if(not next_generation_firefly_mat(from_ind, to_ind)) { return; }

              if constexpr(noDuplicatesFlag(CONFIG))
              {
                if(std::any_of(std::execution::par_unseq, next_generation_firefly.cbegin(), next_generation_firefly.cend(),
                               [&] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& comp_plan) -> bool
                               {
                                 return (next_generation_firefly_mat(from_ind, to_ind)->first.array() == comp_plan->first.array()).all();
                               }))
                { return; }
              }

              next_generation_firefly.emplace_back(std::move(next_generation_firefly_mat(from_ind, to_ind)));
            });
          });
        });
      }

      if constexpr(mutationsFlag(CONFIG))
      {
        mutations_thread.join();
      }
      if constexpr(fireflyOperatorFlag(CONFIG))
      {
        firefly_operator_thread.join();
      }
      if constexpr(noDuplicatesFlag(CONFIG))
      {
        next_generation_firefly.erase(std::remove_if(std::execution::par_unseq, next_generation_firefly.begin(), next_generation_firefly.end(),
                                                     [&] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& fa_plan) -> bool
                                                     {
                                                       return std::any_of(std::execution::par_unseq, next_generation_mutation.cbegin(), next_generation_mutation.cend(),
                                                                          [&] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& comp_plan) -> bool
                                                                          {
                                                                            return (fa_plan->first.array() == comp_plan->first.array()).all();
                                                                          });
                                                     }),
                                      next_generation_firefly.end());
      }
      std::vector<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>> next_generation;
      next_generation.reserve(next_generation_mutation.size() + next_generation_firefly.size());
      next_generation.insert(next_generation.end(), std::make_move_iterator(next_generation_mutation.begin()), std::make_move_iterator(next_generation_mutation.end()));
      next_generation.insert(next_generation.end(), std::make_move_iterator(next_generation_firefly. begin()), std::make_move_iterator(next_generation_firefly. end()));

      /// Perform selection
      {
        const size_t new_fireflies_size = next_generation.size();
        if(0 == new_fireflies_size) { continue; }

        const size_t full_population_size = params->population_size + new_fireflies_size;

        std::vector<Eigen::Index> full_population_inds(full_population_size);
        std::iota(full_population_inds.begin(), full_population_inds.end(), 0);

        Eigen::Matrix<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>,Eigen::Dynamic,1> full_population(full_population_size, 1);

        std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
        [&] (const Eigen::Index population_ind) -> void
        {
          full_population[population_ind] = std::move(population[population_ind]);
        });
        std::copy(std::execution::par_unseq, std::make_move_iterator(next_generation.begin()), std::make_move_iterator(next_generation.end()), full_population.data() + params->population_size);

        if constexpr(fitnessSelectionFlag(CONFIG))
        {
          std::partial_sort(std::execution::par_unseq, full_population_inds.begin(), full_population_inds.begin() + params->population_size, full_population_inds.end(),
                            [&] (const Eigen::Index i, const Eigen::Index j) -> bool
                            { return full_population[i]->second < full_population[j]->second; });
          std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
          [&] (const Eigen::Index population_ind) -> void
          {
            population[population_ind] = std::move(full_population[full_population_inds[population_ind]]);
          });
        }
        if constexpr(stochasticUniversalSamplingSelectionFlag(CONFIG))
        {
          std::vector<Eigen::Index> new_population_inds;
          new_population_inds.reserve(params->population_size);

          if constexpr(elitismSelectionFlag(CONFIG))
          {
            std::partial_sort(std::execution::par_unseq, full_population_inds.rbegin(), full_population_inds.rbegin() + 1, full_population_inds.rend(),
                              [&] (const Eigen::Index i, const Eigen::Index j) -> bool
                              { return full_population[i]->second < full_population[j]->second; });
            new_population_inds.emplace_back(full_population_inds.back());
            full_population_inds.pop_back();
          }
          std::sample(full_population_inds.begin(), full_population_inds.end(), std::back_inserter(new_population_inds), params->population_size - new_population_inds.size(), rand_gen);
          std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
          [&] (const Eigen::Index population_ind) -> void
          {
            population[population_ind] = std::move(full_population[new_population_inds[population_ind]]);
          });
        }
        if constexpr(fitnessProportionalSelectionFlag(CONFIG))
        {
          std::vector<Eigen::Index> new_population_inds;
          new_population_inds.reserve(params->population_size);

          if constexpr(elitismSelectionFlag(CONFIG))
          {
            std::partial_sort(std::execution::par_unseq, full_population_inds.rbegin(), full_population_inds.rbegin() + 1, full_population_inds.rend(),
                              [&] (const Eigen::Index i, const Eigen::Index j) -> bool
                              { return full_population[i]->second < full_population[j]->second; });
            new_population_inds.emplace_back(full_population_inds.back());
            full_population_inds.pop_back();
          }
          // Find fitness and make is a cumulative probability
          const size_t full_population_inds_size = full_population_inds.size();
          Eigen::Matrix<double,Eigen::Dynamic,1> fitness_values = full_population.unaryExpr([] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& i) -> double { return i->second; });
          fitness_values.array() -= fitness_values.minCoeff<Eigen::NaNPropagationOptions::PropagateFast>();
          fitness_values = (double(1) - (fitness_values.array() / fitness_values.maxCoeff<Eigen::NaNPropagationOptions::PropagateFast>())).eval();

          // Selection from Efraimidis, Pavlos S., and Paul G. Spirakis. "Weighted random sampling with a reservoir." Information processing letters 97.5 (2006): 181-185.
          const Eigen::Matrix<double,Eigen::Dynamic,1> u    = Eigen::Matrix<double,Eigen::Dynamic,1>::NullaryExpr(full_population_size, [&] () { return probability_dist(rand_gen); });
          const Eigen::Matrix<double,Eigen::Dynamic,1> keys = u.binaryExpr(fitness_values, [] (const double ui, const double mpi) -> double { return std::pow(ui, double(1) / mpi); });
          const size_t                                 num_to_sort = params->population_size - new_population_inds.size();
          std::partial_sort(std::execution::par_unseq, full_population_inds.begin(), full_population_inds.begin() + num_to_sort, full_population_inds.end(),
                            [&] (const size_t i, const size_t j) -> bool { return keys[i] > keys[j]; });
          std::copy_n(std::execution::par_unseq, full_population_inds.begin(), num_to_sort, std::back_inserter(new_population_inds));
          assert(params->population_size == new_population_inds.size());

          std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
          [&] (const Eigen::Index population_ind) -> void
          {
            population[population_ind] = std::move(full_population[new_population_inds[population_ind]]);
          });
        }
        if constexpr(kMeansSelectionFlag(CONFIG))
        {
          std::vector<Eigen::Index> new_population_inds;
          new_population_inds.reserve(params->population_size);

          const boost::integer_range<size_t> cluster_inds(0, params->number_clusters);

          /// Find distances between all solutions
          // From ind, then to ind
          Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> full_distances(full_population_size, full_population_size);
          #ifndef NDEBUG
            full_distances.setZero();
          #endif
          if constexpr(fireflyOperatorFlag(CONFIG))
          {
            full_distances.topLeftCorner(params->population_size, params->population_size) = distances;
            std::for_each(std::execution::par_unseq, full_population_inds.begin(), full_population_inds.end(),
            [&] (const size_t from_ind) -> void
            {
              const bool from_set = from_ind < params->population_size;
              full_distances(from_ind, from_ind) = 0;
              const boost::integer_range<size_t> the_rest_of_the_population_inds(from_ind + 1, full_population_size);
              std::for_each(std::execution::par_unseq, the_rest_of_the_population_inds.begin(), the_rest_of_the_population_inds.end(),
              [&] (const size_t to_ind) -> void
              {
                const bool to_set = to_ind < params->population_size;
                if(from_set and to_set) { return; }
                assert(0 == full_distances(from_ind, to_ind));
                // Calculation for one way
                full_distances(from_ind, to_ind) = (full_population[to_ind]->first - full_population[from_ind]->first).norm();
                // Now the other way
                full_distances(to_ind, from_ind) = full_distances(from_ind, to_ind);
              });
            });
          }
          else // No firefly operator
          {
            std::for_each(std::execution::par_unseq, full_population_inds.begin(), full_population_inds.end(),
            [&] (const size_t from_ind) -> void
            {
              full_distances(from_ind, from_ind) = 0;
              const boost::integer_range<size_t> the_rest_of_the_population_inds(from_ind + 1, full_population_size);
              std::for_each(std::execution::par_unseq, the_rest_of_the_population_inds.begin(), the_rest_of_the_population_inds.end(),
              [&] (const size_t to_ind) -> void
              {
                // Calculation for one way
                full_distances(from_ind, to_ind) = (full_population[to_ind]->first - full_population[from_ind]->first).norm();
                // Now the other way
                full_distances(to_ind, from_ind) = full_distances(from_ind, to_ind);
              });
            });
          }
          /// Find cluster centers
          Eigen::Matrix<Eigen::Index,Eigen::Dynamic,1> cluster_center_inds(params->number_clusters);
          std::vector<Eigen::Index> full_population_inds_cluster_center = full_population_inds;
          {
            std::uniform_int_distribution<size_t> rand_dist(0, full_population_size - 1);
            cluster_center_inds[0] = rand_dist(rand_gen);
            full_population_inds_cluster_center.erase(std::next(full_population_inds_cluster_center.begin(), cluster_center_inds[0]));
          }
          for(Eigen::Index cluster_ind = 1; cluster_ind < params->number_clusters; ++cluster_ind)
          {
            const size_t                       full_population_inds_size = full_population_inds_cluster_center.size();
            const boost::integer_range<size_t> full_population_inds_inds(0, full_population_inds_size);

            Eigen::Matrix<double,Eigen::Dynamic,1> dists_to_closest_cluster_p2(full_population_inds_size, 1);
            std::for_each(std::execution::par_unseq, full_population_inds_inds.begin(), full_population_inds_inds.end(),
            [&] (const size_t full_pop_ind_ind) -> void
            {
              const size_t pop_ind = full_population_inds_cluster_center[full_pop_ind_ind];
              const auto   min_cluster_ind_ptr = std::min_element(std::execution::par_unseq, cluster_center_inds.data(), cluster_center_inds.data() + cluster_ind,
                                                                  [&] (const Eigen::Index i, const Eigen::Index j) -> bool { return full_distances(pop_ind, i) < full_distances(pop_ind, j); });
              assert(min_cluster_ind_ptr != (cluster_center_inds.data() + cluster_ind));

              dists_to_closest_cluster_p2[full_pop_ind_ind] = std::pow(full_distances(pop_ind, *min_cluster_ind_ptr), double(2));
            });

            Eigen::Matrix<double,Eigen::Dynamic,1> cum_prob(full_population_inds_size, 1);
            cum_prob[0] = dists_to_closest_cluster_p2[0];
            for(size_t population_ind = 1; population_ind < full_population_inds_size; ++population_ind)
            {
              cum_prob[population_ind] = dists_to_closest_cluster_p2[population_ind] + cum_prob[population_ind - 1];
            }
            cum_prob.array() /= dists_to_closest_cluster_p2.sum();

            const double probability_val = probability_dist(rand_gen);
            const size_t to_add_ind      = (cum_prob.array() < probability_val).count();
            //if(0 == cum_prob[to_add_ind]) { ++to_add_ind; }
            // Make it a cluster center and remove it from old population
            cluster_center_inds[cluster_ind] = full_population_inds_cluster_center[to_add_ind];
            full_population_inds_cluster_center.erase(std::next(full_population_inds_cluster_center.cbegin(), to_add_ind));
          }
          /// Perform Clustering
          Eigen::Matrix<std::vector<Eigen::Index>,Eigen::Dynamic,1> cluster_member_inds(    params->number_clusters);
          Eigen::Matrix<std::mutex,Eigen::Dynamic,1>                cluster_member_inds_mux(params->number_clusters);
          std::for_each(std::execution::par_unseq, cluster_inds.begin(), cluster_inds.end(),
          [&] (const size_t cluster_ind) -> void
          {
            cluster_member_inds[cluster_ind].reserve(full_population_size);
            cluster_member_inds[cluster_ind].emplace_back(cluster_center_inds[cluster_ind]);
          });
          // Assign clusters
          std::for_each(std::execution::par_unseq, full_population_inds_cluster_center.begin(), full_population_inds_cluster_center.end(),
          [&] (const Eigen::Index population_ind) -> void
          {
            // Find closest cluster
            Eigen::Index closest_cluster_ind;
            full_distances(population_ind, cluster_center_inds).minCoeff(&closest_cluster_ind);
            // Add to that cluster
            {
              std::lock_guard<std::mutex> lock(cluster_member_inds_mux[closest_cluster_ind]);
              cluster_member_inds[closest_cluster_ind].emplace_back(population_ind);
            }
          });
          assert(full_population_size == cluster_member_inds.unaryExpr([] (const std::vector<Eigen::Index>& i) -> size_t { return i.size(); }).sum());
          for(size_t clustering_it = 0; clustering_it < params->number_cluster_iterations; ++clustering_it)
          {
            // Find new cluster centers
            std::atomic_size_t num_cluster_centers_moved = 0;
            std::for_each(std::execution::par_unseq, cluster_inds.begin(), cluster_inds.end(),
            [&] (const size_t cluster_ind) -> void
            {
              const size_t                           cluster_size = cluster_member_inds[cluster_ind].size();
              const boost::integer_range<size_t>     cluster_member_inds_inds(0, cluster_size);
              Eigen::Matrix<double,Eigen::Dynamic,1> sum_of_dists(cluster_size);
              std::for_each(std::execution::par_unseq, cluster_member_inds_inds.begin(), cluster_member_inds_inds.end(),
              [&] (const size_t member_ind) -> void
              {
                sum_of_dists[member_ind] = full_distances(member_ind, cluster_member_inds[cluster_ind]).sum();
              });
              // Find new center
              Eigen::Index min_sum_ind;
              sum_of_dists.minCoeff(&min_sum_ind);
              const Eigen::Index new_cluster_center = cluster_member_inds[cluster_ind][min_sum_ind];
              if(new_cluster_center != cluster_center_inds[cluster_ind]) { ++num_cluster_centers_moved; }
              cluster_center_inds[cluster_ind] = new_cluster_center;
              // Clear old cluster info
              cluster_member_inds[cluster_ind].clear();
              cluster_member_inds[cluster_ind].reserve(full_population_size);
              cluster_member_inds[cluster_ind].emplace_back(cluster_center_inds[cluster_ind]);
            });
            if(0 == num_cluster_centers_moved) { break; }
            // Assign clusters
            std::for_each(std::execution::par_unseq, full_population_inds.begin(), full_population_inds.end(),
            [&] (const Eigen::Index population_ind) -> void
            {
              if((cluster_center_inds.array() == population_ind).any()) { return; }
              // Find closest cluster
              Eigen::Index closest_cluster_ind;
              full_distances(population_ind, cluster_center_inds).minCoeff(&closest_cluster_ind);
              // Add to that cluster
              {
                std::lock_guard<std::mutex> lock(cluster_member_inds_mux[closest_cluster_ind]);
                cluster_member_inds[closest_cluster_ind].emplace_back(population_ind);
              }
            });
            #ifndef NDEBUG
              std::for_each(std::execution::par_unseq, cluster_inds.begin(), cluster_inds.end(),
              [&] (const size_t cluster_ind) -> void
              {
                assert(0 < cluster_member_inds[cluster_ind].size());
                std::sort(std::execution::par_unseq, cluster_member_inds[cluster_ind].begin(), cluster_member_inds[cluster_ind].end());
                assert(cluster_member_inds[cluster_ind].end() == std::unique(std::execution::par_unseq, cluster_member_inds[cluster_ind].begin(), cluster_member_inds[cluster_ind].end()));
              });
            #endif
            assert(full_population_size == cluster_member_inds.unaryExpr([] (const std::vector<Eigen::Index>& i) -> size_t { return i.size(); }).sum());
          }
          /// Calculate membership probability index
          Eigen::Matrix<double,Eigen::Dynamic,1> membership_probability_index(full_population_size);
          const double inv_population_size = double(1) / double(full_population_size);
          std::for_each(std::execution::par_unseq, cluster_inds.begin(), cluster_inds.end(),
          [&] (const size_t cluster_ind) -> void
          {
            const size_t cluster_size      = cluster_member_inds[cluster_ind].size();
            const double sum_of_fitness    = full_population(cluster_member_inds[cluster_ind]).unaryExpr([] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& i) -> double { return i->second; }).sum();
            const double size_term         = double(cluster_size) / double(cluster_size - 1);
            const double size_and_pop_term = size_term * inv_population_size;
            std::for_each(std::execution::par_unseq, cluster_member_inds[cluster_ind].cbegin(), cluster_member_inds[cluster_ind].cend(),
            [&] (const Eigen::Index member_ind) -> void
            {
              membership_probability_index[member_ind] = size_and_pop_term * ((sum_of_fitness - full_population[member_ind]->second) / sum_of_fitness);
            });
          });
          //////////////assert(std::fabs(1 - membership_probability_index.sum()) < 0.5);
          /// Perform selection
          if constexpr(elitismSelectionFlag(CONFIG))
          {
            std::for_each(std::execution::par_unseq, cluster_inds.begin(), cluster_inds.end(),
            [&] (const size_t cluster_ind) -> void
            {
              std::partial_sort(std::execution::par_unseq, cluster_member_inds[cluster_ind].rbegin(), cluster_member_inds[cluster_ind].rbegin() + 1, cluster_member_inds[cluster_ind].rend(),
                                [&] (const Eigen::Index i, const Eigen::Index j) -> bool
                                { return full_population[i]->second < full_population[j]->second; });
            });
            std::for_each(cluster_inds.begin(), cluster_inds.end(),
            [&] (const size_t cluster_ind) -> void
            {
              new_population_inds.emplace_back(cluster_member_inds[cluster_ind].back());
            });
            full_population_inds.erase(std::remove_if(std::execution::par_unseq, full_population_inds.begin(), full_population_inds.end(),
                                       [&] (const size_t pop_ind) -> bool { return std::any_of(std::execution::par_unseq, new_population_inds.cbegin(), new_population_inds.cend(), [&] (const size_t new_pop_ind) -> bool { return new_pop_ind == pop_ind; }); }),
                                       full_population_inds.end());
          }
          // Selection from Efraimidis, Pavlos S., and Paul G. Spirakis. "Weighted random sampling with a reservoir." Information processing letters 97.5 (2006): 181-185.
          const Eigen::Matrix<double,Eigen::Dynamic,1> u    = Eigen::Matrix<double,Eigen::Dynamic,1>::NullaryExpr(full_population_size, [&] () { return probability_dist(rand_gen); });
          const Eigen::Matrix<double,Eigen::Dynamic,1> keys = u.binaryExpr(membership_probability_index, [] (const double ui, const double mpi) -> double { return std::pow(ui, double(1) / mpi); });
          const size_t                                 num_to_sort = params->population_size - new_population_inds.size();
          std::partial_sort(std::execution::par_unseq, full_population_inds.begin(), full_population_inds.begin() + num_to_sort, full_population_inds.end(),
                            [&] (const size_t i, const size_t j) -> bool { return keys[i] > keys[j]; });
          std::copy_n(std::execution::par_unseq, full_population_inds.begin(), num_to_sort, std::back_inserter(new_population_inds));
          assert(params->population_size == new_population_inds.size());
          #ifndef NDEBUG
            std::sort(std::execution::par_unseq, new_population_inds.begin(), new_population_inds.end());
            assert(new_population_inds.end() == std::unique(std::execution::par_unseq, new_population_inds.begin(), new_population_inds.end()));
          #endif
          /// Update population for next loop
          std::for_each(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
          [&] (const Eigen::Index population_ind) -> void
          {
            population[population_ind] = std::move(full_population[new_population_inds[population_ind]]);
          });
        }
      }
      assert(params->population_size == population.rows());
      assert(not population.unaryExpr([] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& i) -> bool { return nullptr == i.get(); }).any());
      assert(not population.unaryExpr([] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& i) -> bool { return i->first.hasNaN(); }).any());
    }

    /// Bookkeeping
    log_func(population);
//    {
//      const auto best_worst_ind_its = std::minmax_element(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
//                                                          [&population] (const size_t i, const size_t j) -> bool
//                                                          { return population[i]->second < population[j]->second; });
//      best_sol_found_ind  = *best_worst_ind_its.first;
//      worst_sol_found_ind = *best_worst_ind_its.second;
//    }
    {
      const auto best_ind_it = std::min_element(std::execution::par_unseq, population_inds.begin(), population_inds.end(),
                                                [&population] (const size_t i, const size_t j) -> bool
                                                { return population[i]->second < population[j]->second; });
      best_sol_found_ind  = *best_ind_it;
    }
  }

  return population[best_sol_found_ind]->first;
}

#endif
/* firefly_algorithm.hpp */
