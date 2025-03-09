/**
 * @File: firefly_tuner_server.cpp
 * @Date: August 2024
 * @Author: James Swedeen
 *
 * @brief
 * Simple tuning node for picking the hyper-parameters in the firefly algorithm.
 **/

/* C++ Headers */
#include<malloc.h>
#include<limits>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<graph_surveillance_planning_msgs/srv/try_firefly_algorithm_config.hpp>
#include<firefly_algorithm/firefly_algorithm.hpp>
#include<firefly_algorithm/psocpp.h>

/// Global Parameters
// Michalewicz function
//inline static constexpr const double       min_obj_val = -1.8013 + 2.0;
//inline static constexpr const Eigen::Index DIM         = 2;
// Rastrigin function
inline static constexpr const double       min_obj_val = 0;
//inline static constexpr const Eigen::Index DIM         = 4;
// SHUBERT FUNCTION
//inline static constexpr const double       min_obj_val = -186.730 + 200.0;
//inline static constexpr const Eigen::Index DIM         = 2;

inline static constexpr const double min_obj_val_eps = 1.0e-6;
inline static constexpr const double max_time_sec    = 10;

// Helper functions
template<Eigen::Index DIM>
fa::FaParamsPtr<DIM> make_params(const graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::ConstSharedPtr& req)
{
  fa::FaParamsPtr<DIM> fa_params = std::make_shared<fa::FaParams<DIM>>();
  fa_params->population_size           = req->population_size;
  fa_params->dist_pow                  = req->distance_power;
  fa_params->dist_mult                 = req->distance_multiplier;
  fa_params->base_attractiveness_mult  = req->base_attractiveness_multiplier;
  fa_params->min_attractiveness_mult   = req->min_attractiveness_multiplier;
  fa_params->cost_scaling_mult         = req->cost_scaling_mult;
  fa_params->mutation_magnitude        = Eigen::Matrix<double,DIM,1>::Constant(req->mutation_magnitude);
  fa_params->number_clusters           = req->number_clusters;
  fa_params->number_cluster_iterations = req->number_cluster_iterations;
  fa_params->from_cost_scaling_mult    = req->from_cost_scaling_mult;
  fa_params->soft_max_mult             = req->soft_max_mult;

  return fa_params;
}

template<Eigen::Index DIM>
class Callback
{
public:
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrix;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;
    typedef long int Index;

    std::function<bool(const size_t,const double)> m_stop_func;

    Callback() = default;
    Callback(const std::function<bool(const size_t,const double)>& stop_func)
     : m_stop_func(stop_func)
    {}

    bool operator()(const Index num_iterations, const Matrix& population, const Vector& population_costs, const Index best_ind) const
    {
      return not this->m_stop_func(num_iterations, population_costs[best_ind]);
    }
};
template<Eigen::Index DIM>
struct Objective
{
  public:

  std::function<std::pair<bool,double>(const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>&)> m_obj_func;

  Objective() = default;
  Objective(const std::function<std::pair<bool,double>(const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>&)>& obj_func)
   : m_obj_func(obj_func)
  {}

  template<typename Derived>
  double operator()(const Eigen::MatrixBase<Derived> &xval) const
  {
    return this->m_obj_func(xval).second;
  }
};



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("firefly_tuner_server");


  // Rosenbrock's function
//  objective_func.emplace_back([&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
//  {
//    assert(not x.array().isNaN().any());
//    static constexpr const double max = 5;
//    if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
//    double output = 0;
//    for(Eigen::Index dim_ind = 0; dim_ind < (DIM - 1); ++dim_ind)
//    {
//      output += (std::pow(x[dim_ind] - double(1), double(2)) + (double(100) * std::pow(x[dim_ind + 1] - std::pow(x[dim_ind], double(2)), double(2))));
//    }
//    return std::pair<bool,double>(true, output);
//  });
//  rand_init_func.emplace_back([&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
//  {
//    static thread_local std::default_random_engine rand_gen(rand_seed);
//    static constexpr const double max = 5;
//    std::uniform_real_distribution<double> probability_dist(-max, max);
//    return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//  });
  // Schwefel's function
//  objective_func.emplace_back([&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
//  {
//    assert(not x.array().isNaN().any());
//    static constexpr const double max = 500;
//    if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
//    return std::pair<bool,double>(true, (double(418.9829) * double(DIM)) - (x.array() * x.array().abs().sqrt().sin()).sum());
//  });
//  rand_init_func.emplace_back([&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
//  {
//    static thread_local std::default_random_engine rand_gen(rand_seed);
//    static constexpr const double max = 500;
//    std::uniform_real_distribution<double> probability_dist(-max, max);
//    return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//  });
  // Ackley's function
//  objective_func.emplace_back([&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
//  {
//    assert(not x.array().isNaN().any());
//    static constexpr const double max = 32.768;
//    if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
//    static constexpr const double a = 20;
//    static constexpr const double b = 0.2;
//    static constexpr const double c = double(2) * double(M_PI);
//    return std::pair<bool,double>(true, - a * std::exp(-b * std::sqrt(x.array().square().sum() / double(DIM)))
//           - std::exp((double(1) / double(DIM)) * (x.array() * c).cos().sum())
//           + a + std::exp(1));
//  });
//  rand_init_func.emplace_back([&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
//  {
//    static thread_local std::default_random_engine rand_gen(rand_seed);
//    static constexpr const double max = 32.768;
//    std::uniform_real_distribution<double> probability_dist(-max, max);
//    return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//  });
  // Rastrigin function
//  objective_func.emplace_back([&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
//  {
//    assert(not x.array().isNaN().any());
//    static constexpr const double max = 5.12;
//    if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
//    static constexpr const double A = 10;
//    return std::pair<bool,double>(true, (A * DIM) + (x.array().square() - (A * (x.array() * double(2) * double(M_PI)).cos())).sum());
//  });
//  rand_init_func.emplace_back([&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
//  {
//    static thread_local std::default_random_engine rand_gen(rand_seed);
//    static constexpr const double max = 5.12;
//    std::uniform_real_distribution<double> probability_dist(-max, max);
//    return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//  });
  // Griewank's function
//  objective_func.emplace_back([&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
//  {
//    assert(not x.array().isNaN().any());
//    static constexpr const double max = 600;
//    if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
//    return std::pair<bool,double>(true, double(1)
//           + ((double(1)/double(4000)) * x.array().square().sum())
//           - (x.array() / Eigen::Array<double,DIM,1>::LinSpaced(DIM, 1, DIM).sqrt()).cos().prod());
//  });
//  rand_init_func.emplace_back([&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
//  {
//    static thread_local std::default_random_engine rand_gen(rand_seed);
//    static constexpr const double max = 600;
//    std::uniform_real_distribution<double> probability_dist(-max, max);
//    return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//  });



  // De Jong's sphere function
//  objective_func.emplace_back([&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
//  {
//    assert(not x.array().isNaN().any());
//    static constexpr const double max = 5.12;
//    if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
//    return std::pair<bool,double>(true, x.array().square().sum());
//  });
//  rand_init_func.emplace_back([&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
//  {
//    static thread_local std::default_random_engine rand_gen(rand_seed);
//    static constexpr const double max = 5.12;
//    std::uniform_real_distribution<double> probability_dist(-max, max);
//    return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//  });
//  // De Jong's hyper-ellipsoid function
//  objective_func.emplace_back([&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
//  {
//    double output = 0;
//    for(Eigen::Index dim_ind = 0; dim_ind < DIM; ++dim_ind)
//    {
//      output += (double(dim_ind + 1) * std::pow(x[dim_ind], double(2)));
//    }
//    return output;
//  });
//  rand_init_func.emplace_back([&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
//  {
//    static thread_local std::default_random_engine             rand_gen(rand_seed);
//    static constexpr const double max = 5.12;
//    std::uniform_real_distribution<double> probability_dist(-max, max);
//    return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//  });
  // Michalewicz function
//  const auto objective_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> double
//    {
//      static constexpr const double m = 10;
//      double output = 0;
//      for(Eigen::Index dim_ind = 0; dim_ind < DIM; ++dim_ind)
//      {
//        output += (std::sin(x[dim_ind]) * std::pow(std::sin(((dim_ind+1)*std::pow(x[dim_ind], 2.0)) / double(M_PI)), 2.0 * m));
//      }
//      return -output + 2.0;
//    };
//  const auto rand_init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
//    {
//      std::default_random_engine             rand_gen(rand_seed);
//      std::uniform_real_distribution<double> probability_dist(0, M_PI);
//      return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//    };
  // SHUBERT FUNCTION
//  const auto objective_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> double
//    {
//      static constexpr const double n = 5;
//      Eigen::Matrix<double,2,1> sums = Eigen::Matrix<double,2,1>::Zero();
//      for(size_t i = 1; i <= n; ++i)
//      {
//        sums.array() += double(i) * ((double(i + 1) * x.array()) + double(i)).cos();
//      }
//      return (sums[0] * sums[1]) + double(200.0);
//    };
//  const auto rand_init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
//    {
//      std::default_random_engine             rand_gen(rand_seed);
//      std::uniform_real_distribution<double> probability_dist(-10, 10);
//      return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//    };


  /// Start testing
  std::cout << "Ready to Benchmark" << std::endl;
  std::mutex one_at_a_time_mux;
  rclcpp::Service<graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig>::SharedPtr service =
    node->create_service<graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig>("firefly_config_test",
  [&] (const graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::ConstSharedPtr req,
             graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Response::SharedPtr     res) -> void
  {
    res->costs.reserve(req->number_runs);
    res->times.reserve(req->number_runs);

    std::lock_guard<std::mutex> lock(one_at_a_time_mux);

    switch(req->problem_ind)
    {
      case 0:
        #define DIM 2
        for(size_t run_ind = 0; run_ind < req->number_runs; ++run_ind)
        {
          const double max_val = double(2) * double(M_PI);
          // Easom's function, Xin-She Yang extension
          auto objective_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
          {
            assert(not x.array().isNaN().any());
            static constexpr const double max = double(2) * double(M_PI);
            if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
            return std::pair<bool,double>(true, double(1) + (
                   double(-std::pow(-1, DIM))
                   * x.array().cos().square().prod()
                   * std::exp(-(x.array() - double(M_PI)).square().sum())));
          };
          auto rand_init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
          {
            static thread_local std::default_random_engine rand_gen(rand_seed);
            static constexpr const double max = double(2) * double(M_PI);
            std::uniform_real_distribution<double> probability_dist(-max, max);
            return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
          };
          // Define stopping condition
          auto plan_start_time = std::chrono::high_resolution_clock::now();
          const auto stopping_condition_func =
            [&] (const size_t generation_count, const double best_cost_found) -> bool
            {
              return (best_cost_found <= (min_obj_val + min_obj_val_eps)) or (max_time_sec < (double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9)));
            };
          // Define log function
          const auto logging_func = [] (const Eigen::Matrix<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>,Eigen::Dynamic,1>&) -> void {};

          // Run correct version
          std::pair<bool,double> obj_val(false, std::numeric_limits<double>::quiet_NaN());
          switch(req->use_firefly_operator)
          {
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::NO_USE_FIREFLY_OPERATOR:
              switch(req->use_selection)
              {
                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                  if(req->force_no_duplicates)
//                  {
//                    static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                   fa::FaFlags::USE_FITNESS_SELECTION bitor
//                                                                                   fa::FaFlags::FORCE_NO_DUPLICATES);
//                    plan_start_time = std::chrono::high_resolution_clock::now();
//                    const Eigen::Matrix<double,DIM,1> plan =
//                      fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                           rand_init_func,
//                                                           objective_func,
//                                                           stopping_condition_func,
//                                                           logging_func,
//                                                           42+run_ind);
//                    obj_val = objective_func(plan);
//                  }
//                  else
                  {
                    static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
                                                                                   fa::FaFlags::USE_FITNESS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const Eigen::Matrix<double,DIM,1> plan =
                      fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                           rand_init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           42+run_ind);
                    obj_val = objective_func(plan);
                  }
                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
                default:
                  std::cout << "Not a valid case" << std::endl;
                  break;
              }
              break;
//            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::NORMAL_FIREFLY_OPERATOR:
//              if(req->use_original_luminosity_multiplier)
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              else // Use other luminosity multiplier
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              break;
//            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::BIDIRECTIONAL_FIREFLY_OPERATOR:
//              if(req->use_original_luminosity_multiplier)
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              else // Use other luminosity multiplier
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::VEC_FIREFLY_OPERATOR:
              if(req->use_original_luminosity_multiplier)
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              else // other lum
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::VEC_BIDIRECTIONAL_FIREFLY_OPERATOR:
              if(req->use_original_luminosity_multiplier)
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              else // other lum
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::PSO:
              {
                pso::ParticleSwarmOptimization<double,Objective<DIM>,pso::ConstantWeight<double>,Callback<DIM>> optimizer(42+run_ind);
                optimizer.setThreads(64);
                optimizer.setMaxIterations(0);
                optimizer.setMinParticleChange(0);
                optimizer.setMinFunctionChange(0);
                optimizer.setPhiParticles(req->pso_phi_particles);
                optimizer.setPhiGlobal(req->pso_phi_global);
                optimizer.setMaxVelocity(0);
                optimizer.setVerbosity(0);
                optimizer.setObjective(Objective<DIM>(objective_func));
                optimizer.setCallback(Callback<DIM>(stopping_condition_func));

                Eigen::Matrix<double,2,DIM> bounds;
                bounds.row(0).array() = -max_val;
                bounds.row(1).array() =  max_val;

                plan_start_time = std::chrono::high_resolution_clock::now();
                obj_val = std::pair<bool,double>(true, optimizer.minimize(bounds, req->population_size).fval);
              }
              break;
            default:
              std::cout << "Not a valid case" << std::endl;
              break;
          }
          assert(obj_val.first);
          res->costs.emplace_back(obj_val.second);
          res->times.emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9));
        }
        #undef DIM
        break;
      case 1:
        #define DIM 4
        for(size_t run_ind = 0; run_ind < req->number_runs; ++run_ind)
        {
          const double max_val = 5.12;
          // Rastrigin function
          auto objective_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
          {
            assert(not x.array().isNaN().any());
            static constexpr const double max = 5.12;
            if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
            static constexpr const double A = 10;
            return std::pair<bool,double>(true, (A * DIM) + (x.array().square() - (A * (x.array() * double(2) * double(M_PI)).cos())).sum());
          };
          auto rand_init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
          {
            static thread_local std::default_random_engine rand_gen(rand_seed);
            static constexpr const double max = 5.12;
            std::uniform_real_distribution<double> probability_dist(-max, max);
            return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
          };
          // Define stopping condition
          auto plan_start_time = std::chrono::high_resolution_clock::now();
          const auto stopping_condition_func =
            [&] (const size_t generation_count, const double best_cost_found) -> bool
            {
              return (best_cost_found <= (min_obj_val + min_obj_val_eps)) or (max_time_sec < (double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9)));
            };
          // Define log function
          const auto logging_func = [] (const Eigen::Matrix<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>,Eigen::Dynamic,1>&) -> void {};

          // Run correct version
          std::pair<bool,double> obj_val(false, std::numeric_limits<double>::quiet_NaN());
          switch(req->use_firefly_operator)
          {
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::NO_USE_FIREFLY_OPERATOR:
              switch(req->use_selection)
              {
                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
                  if(req->force_no_duplicates)
                  {
                    static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
                                                                                   fa::FaFlags::USE_FITNESS_SELECTION bitor
                                                                                   fa::FaFlags::FORCE_NO_DUPLICATES);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const Eigen::Matrix<double,DIM,1> plan =
                      fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                           rand_init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           42+run_ind);
                    obj_val = objective_func(plan);
                  }
                  else
                  {
                    static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
                                                                                   fa::FaFlags::USE_FITNESS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const Eigen::Matrix<double,DIM,1> plan =
                      fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                           rand_init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           42+run_ind);
                    obj_val = objective_func(plan);
                  }
                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
                default:
                  std::cout << "Not a valid case" << std::endl;
                  break;
              }
              break;
//            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::NORMAL_FIREFLY_OPERATOR:
//              if(req->use_original_luminosity_multiplier)
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              else // Use other luminosity multiplier
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              break;
//            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::BIDIRECTIONAL_FIREFLY_OPERATOR:
//              if(req->use_original_luminosity_multiplier)
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              else // Use other luminosity multiplier
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::VEC_FIREFLY_OPERATOR:
              if(req->use_original_luminosity_multiplier)
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              else // other lum
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::VEC_BIDIRECTIONAL_FIREFLY_OPERATOR:
              if(req->use_original_luminosity_multiplier)
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              else // other lum
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::PSO:
              {
                pso::ParticleSwarmOptimization<double,Objective<DIM>,pso::ConstantWeight<double>,Callback<DIM>> optimizer(42+run_ind);
                optimizer.setThreads(64);
                optimizer.setMaxIterations(0);
                optimizer.setMinParticleChange(0);
                optimizer.setMinFunctionChange(0);
                optimizer.setPhiParticles(req->pso_phi_particles);
                optimizer.setPhiGlobal(req->pso_phi_global);
                optimizer.setMaxVelocity(0);
                optimizer.setVerbosity(0);
                optimizer.setObjective(Objective<DIM>(objective_func));
                optimizer.setCallback(Callback<DIM>(stopping_condition_func));

                Eigen::Matrix<double,2,DIM> bounds;
                bounds.row(0).array() = -max_val;
                bounds.row(1).array() =  max_val;

                plan_start_time = std::chrono::high_resolution_clock::now();
                obj_val = std::pair<bool,double>(true, optimizer.minimize(bounds, req->population_size).fval);
              }
              break;
            default:
              std::cout << "Not a valid case" << std::endl;
              break;
          }
          assert(obj_val.first);
          res->costs.emplace_back(obj_val.second);
          res->times.emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9));
        }
        #undef DIM
        break;
      case 2:
        #define DIM 8
        for(size_t run_ind = 0; run_ind < req->number_runs; ++run_ind)
        {
          const double max_val = 600;
          // Griewank's function
          auto objective_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
          {
            assert(not x.array().isNaN().any());
            static constexpr const double max = 600;
            if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
            return std::pair<bool,double>(true, double(1)
                   + ((double(1)/double(4000)) * x.array().square().sum())
                   - (x.array() / Eigen::Array<double,DIM,1>::LinSpaced(DIM, 1, DIM).sqrt()).cos().prod());
          };
          auto rand_init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
          {
            static thread_local std::default_random_engine rand_gen(rand_seed);
            static constexpr const double max = 600;
            std::uniform_real_distribution<double> probability_dist(-max, max);
            return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
          };
          // Define stopping condition
          auto plan_start_time = std::chrono::high_resolution_clock::now();
          const auto stopping_condition_func =
            [&] (const size_t generation_count, const double best_cost_found) -> bool
            {
              return (best_cost_found <= (min_obj_val + min_obj_val_eps)) or (max_time_sec < (double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9)));
            };
          // Define log function
          const auto logging_func = [] (const Eigen::Matrix<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>,Eigen::Dynamic,1>&) -> void {};

          // Run correct version
          std::pair<bool,double> obj_val(false, std::numeric_limits<double>::quiet_NaN());
          switch(req->use_firefly_operator)
          {
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::NO_USE_FIREFLY_OPERATOR:
              switch(req->use_selection)
              {
                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
                  if(req->force_no_duplicates)
                  {
                    static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
                                                                                   fa::FaFlags::USE_FITNESS_SELECTION bitor
                                                                                   fa::FaFlags::FORCE_NO_DUPLICATES);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const Eigen::Matrix<double,DIM,1> plan =
                      fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                           rand_init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           42+run_ind);
                    obj_val = objective_func(plan);
                  }
                  else
                  {
                    static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
                                                                                   fa::FaFlags::USE_FITNESS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const Eigen::Matrix<double,DIM,1> plan =
                      fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                           rand_init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           42+run_ind);
                    obj_val = objective_func(plan);
                  }
                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
                default:
                  std::cout << "Not a valid case" << std::endl;
                  break;
              }
              break;
//            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::NORMAL_FIREFLY_OPERATOR:
//              if(req->use_original_luminosity_multiplier)
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              else // Use other luminosity multiplier
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              break;
//            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::BIDIRECTIONAL_FIREFLY_OPERATOR:
//              if(req->use_original_luminosity_multiplier)
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              else // Use other luminosity multiplier
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::VEC_FIREFLY_OPERATOR:
              if(req->use_original_luminosity_multiplier)
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              else // other lum
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::VEC_BIDIRECTIONAL_FIREFLY_OPERATOR:
              if(req->use_original_luminosity_multiplier)
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              else // other lum
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::PSO:
              {
                pso::ParticleSwarmOptimization<double,Objective<DIM>,pso::ConstantWeight<double>,Callback<DIM>> optimizer(42+run_ind);
                optimizer.setThreads(64);
                optimizer.setMaxIterations(0);
                optimizer.setMinParticleChange(0);
                optimizer.setMinFunctionChange(0);
                optimizer.setPhiParticles(req->pso_phi_particles);
                optimizer.setPhiGlobal(req->pso_phi_global);
                optimizer.setMaxVelocity(0);
                optimizer.setVerbosity(0);
                optimizer.setObjective(Objective<DIM>(objective_func));
                optimizer.setCallback(Callback<DIM>(stopping_condition_func));

                Eigen::Matrix<double,2,DIM> bounds;
                bounds.row(0).array() = -max_val;
                bounds.row(1).array() =  max_val;

                plan_start_time = std::chrono::high_resolution_clock::now();
                obj_val = std::pair<bool,double>(true, optimizer.minimize(bounds, req->population_size).fval);
              }
              break;
            default:
              std::cout << "Not a valid case" << std::endl;
              break;
          }
          assert(obj_val.first);
          res->costs.emplace_back(obj_val.second);
          res->times.emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9));
        }
        #undef DIM
        break;
      case 3:
        #define DIM 32
        for(size_t run_ind = 0; run_ind < req->number_runs; ++run_ind)
        {
          const double max_val = 500;
          // Schwefel's function
          auto objective_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
          {
            assert(not x.array().isNaN().any());
            static constexpr const double max = 500;
            if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
            return std::pair<bool,double>(true, (double(418.9829) * double(DIM)) - (x.array() * x.array().abs().sqrt().sin()).sum());
          };
          auto rand_init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
          {
            static thread_local std::default_random_engine rand_gen(rand_seed);
            static constexpr const double max = 500;
            std::uniform_real_distribution<double> probability_dist(-max, max);
            return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
          };
          // Define stopping condition
          auto plan_start_time = std::chrono::high_resolution_clock::now();
          const auto stopping_condition_func =
            [&] (const size_t generation_count, const double best_cost_found) -> bool
            {
              return (best_cost_found <= (min_obj_val + min_obj_val_eps)) or (max_time_sec < (double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9)));
            };
          // Define log function
          const auto logging_func = [] (const Eigen::Matrix<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>,Eigen::Dynamic,1>&) -> void {};

          // Run correct version
          std::pair<bool,double> obj_val(false, std::numeric_limits<double>::quiet_NaN());
          switch(req->use_firefly_operator)
          {
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::NO_USE_FIREFLY_OPERATOR:
              switch(req->use_selection)
              {
                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
                  if(req->force_no_duplicates)
                  {
                    static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
                                                                                   fa::FaFlags::USE_FITNESS_SELECTION bitor
                                                                                   fa::FaFlags::FORCE_NO_DUPLICATES);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const Eigen::Matrix<double,DIM,1> plan =
                      fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                           rand_init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           42+run_ind);
                    obj_val = objective_func(plan);
                  }
                  else
                  {
                    static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
                                                                                   fa::FaFlags::USE_FITNESS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const Eigen::Matrix<double,DIM,1> plan =
                      fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                           rand_init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           42+run_ind);
                    obj_val = objective_func(plan);
                  }
                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
                default:
                  std::cout << "Not a valid case" << std::endl;
                  break;
              }
              break;
//            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::NORMAL_FIREFLY_OPERATOR:
//              if(req->use_original_luminosity_multiplier)
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              else // Use other luminosity multiplier
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              break;
//            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::BIDIRECTIONAL_FIREFLY_OPERATOR:
//              if(req->use_original_luminosity_multiplier)
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              else // Use other luminosity multiplier
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::VEC_FIREFLY_OPERATOR:
              if(req->use_original_luminosity_multiplier)
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              else // other lum
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::VEC_BIDIRECTIONAL_FIREFLY_OPERATOR:
              if(req->use_original_luminosity_multiplier)
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              else // other lum
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::PSO:
              {
                pso::ParticleSwarmOptimization<double,Objective<DIM>,pso::ConstantWeight<double>,Callback<DIM>> optimizer(42+run_ind);
                optimizer.setThreads(64);
                optimizer.setMaxIterations(0);
                optimizer.setMinParticleChange(0);
                optimizer.setMinFunctionChange(0);
                optimizer.setPhiParticles(req->pso_phi_particles);
                optimizer.setPhiGlobal(req->pso_phi_global);
                optimizer.setMaxVelocity(0);
                optimizer.setVerbosity(0);
                optimizer.setObjective(Objective<DIM>(objective_func));
                optimizer.setCallback(Callback<DIM>(stopping_condition_func));

                Eigen::Matrix<double,2,DIM> bounds;
                bounds.row(0).array() = -max_val;
                bounds.row(1).array() =  max_val;

                plan_start_time = std::chrono::high_resolution_clock::now();
                obj_val = std::pair<bool,double>(true, optimizer.minimize(bounds, req->population_size).fval);
              }
              break;
            default:
              std::cout << "Not a valid case" << std::endl;
              break;
          }
          assert(obj_val.first);
          res->costs.emplace_back(obj_val.second);
          res->times.emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9));
        }
        #undef DIM
        break;
      case 5:
        #define DIM 128
        for(size_t run_ind = 0; run_ind < req->number_runs; ++run_ind)
        {
          const double max_val = 5;
          // Rosenbrock's function
          auto objective_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
          {
            assert(not x.array().isNaN().any());
            static constexpr const double max = 5;
            if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
            double output = 0;
            for(Eigen::Index dim_ind = 0; dim_ind < (DIM - 1); ++dim_ind)
            {
              output += (std::pow(x[dim_ind] - double(1), double(2)) + (double(100) * std::pow(x[dim_ind + 1] - std::pow(x[dim_ind], double(2)), double(2))));
            }
            return std::pair<bool,double>(true, output);
          };
          auto rand_init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
          {
            static thread_local std::default_random_engine rand_gen(rand_seed);
            static constexpr const double max = 5;
            std::uniform_real_distribution<double> probability_dist(-max, max);
            return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
          };
          // Define stopping condition
          auto plan_start_time = std::chrono::high_resolution_clock::now();
          const auto stopping_condition_func =
            [&] (const size_t generation_count, const double best_cost_found) -> bool
            {
              return (best_cost_found <= (min_obj_val + min_obj_val_eps)) or (max_time_sec < (double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9)));
            };
          // Define log function
          const auto logging_func = [] (const Eigen::Matrix<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>,Eigen::Dynamic,1>&) -> void {};

          // Run correct version
          std::pair<bool,double> obj_val(false, std::numeric_limits<double>::quiet_NaN());
          switch(req->use_firefly_operator)
          {
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::NO_USE_FIREFLY_OPERATOR:
              switch(req->use_selection)
              {
                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
                  if(req->force_no_duplicates)
                  {
                    static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
                                                                                   fa::FaFlags::USE_FITNESS_SELECTION bitor
                                                                                   fa::FaFlags::FORCE_NO_DUPLICATES);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const Eigen::Matrix<double,DIM,1> plan =
                      fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                           rand_init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           42+run_ind);
                    obj_val = objective_func(plan);
                  }
                  else
                  {
                    static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
                                                                                   fa::FaFlags::USE_FITNESS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const Eigen::Matrix<double,DIM,1> plan =
                      fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                           rand_init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           42+run_ind);
                    obj_val = objective_func(plan);
                  }
                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
                default:
                  std::cout << "Not a valid case" << std::endl;
                  break;
              }
              break;
//            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::NORMAL_FIREFLY_OPERATOR:
//              if(req->use_original_luminosity_multiplier)
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              else // Use other luminosity multiplier
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              break;
//            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::BIDIRECTIONAL_FIREFLY_OPERATOR:
//              if(req->use_original_luminosity_multiplier)
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              else // Use other luminosity multiplier
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::VEC_FIREFLY_OPERATOR:
              if(req->use_original_luminosity_multiplier)
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              else // other lum
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::VEC_BIDIRECTIONAL_FIREFLY_OPERATOR:
              if(req->use_original_luminosity_multiplier)
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              else // other lum
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::PSO:
              {
                pso::ParticleSwarmOptimization<double,Objective<DIM>,pso::ConstantWeight<double>,Callback<DIM>> optimizer(42+run_ind);
                optimizer.setThreads(64);
                optimizer.setMaxIterations(0);
                optimizer.setMinParticleChange(0);
                optimizer.setMinFunctionChange(0);
                optimizer.setPhiParticles(req->pso_phi_particles);
                optimizer.setPhiGlobal(req->pso_phi_global);
                optimizer.setMaxVelocity(0);
                optimizer.setVerbosity(0);
                optimizer.setObjective(Objective<DIM>(objective_func));
                optimizer.setCallback(Callback<DIM>(stopping_condition_func));

                Eigen::Matrix<double,2,DIM> bounds;
                bounds.row(0).array() = -max_val;
                bounds.row(1).array() =  max_val;

                plan_start_time = std::chrono::high_resolution_clock::now();
                obj_val = std::pair<bool,double>(true, optimizer.minimize(bounds, req->population_size).fval);
              }
              break;
            default:
              std::cout << "Not a valid case" << std::endl;
              break;
          }
          assert(obj_val.first);
          res->costs.emplace_back(obj_val.second);
          res->times.emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9));
        }
        #undef DIM
        break;
      case 4:
        #define DIM 64
        for(size_t run_ind = 0; run_ind < req->number_runs; ++run_ind)
        {
          const double max_val = 32.768;
          // Ackley's function
          auto objective_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
          {
            assert(not x.array().isNaN().any());
            static constexpr const double max = 32.768;
            if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
            static constexpr const double a = 20;
            static constexpr const double b = 0.2;
            static constexpr const double c = double(2) * double(M_PI);
            return std::pair<bool,double>(true, - a * std::exp(-b * std::sqrt(x.array().square().sum() / double(DIM)))
                   - std::exp((double(1) / double(DIM)) * (x.array() * c).cos().sum())
                   + a + std::exp(1));
          };
          auto rand_init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
          {
            static thread_local std::default_random_engine rand_gen(rand_seed);
            static constexpr const double max = 32.768;
            std::uniform_real_distribution<double> probability_dist(-max, max);
            return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
          };
          // Define stopping condition
          auto plan_start_time = std::chrono::high_resolution_clock::now();
          const auto stopping_condition_func =
            [&] (const size_t generation_count, const double best_cost_found) -> bool
            {
              return (best_cost_found <= (min_obj_val + min_obj_val_eps)) or (max_time_sec < (double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9)));
            };
          // Define log function
          const auto logging_func = [] (const Eigen::Matrix<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>,Eigen::Dynamic,1>&) -> void {};

          // Run correct version
          std::pair<bool,double> obj_val(false, std::numeric_limits<double>::quiet_NaN());
          switch(req->use_firefly_operator)
          {
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::NO_USE_FIREFLY_OPERATOR:
              switch(req->use_selection)
              {
                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
                  if(req->force_no_duplicates)
                  {
                    static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
                                                                                   fa::FaFlags::USE_FITNESS_SELECTION bitor
                                                                                   fa::FaFlags::FORCE_NO_DUPLICATES);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const Eigen::Matrix<double,DIM,1> plan =
                      fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                           rand_init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           42+run_ind);
                    obj_val = objective_func(plan);
                  }
                  else
                  {
                    static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
                                                                                   fa::FaFlags::USE_FITNESS_SELECTION);
                    plan_start_time = std::chrono::high_resolution_clock::now();
                    const Eigen::Matrix<double,DIM,1> plan =
                      fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                           rand_init_func,
                                                           objective_func,
                                                           stopping_condition_func,
                                                           logging_func,
                                                           42+run_ind);
                    obj_val = objective_func(plan);
                  }
                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
//                case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                  if(req->use_elitism_selection)
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  else // Don't use elitism selection
//                  {
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_K_MEANS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                  }
//                  break;
                default:
                  std::cout << "Not a valid case" << std::endl;
                  break;
              }
              break;
//            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::NORMAL_FIREFLY_OPERATOR:
//              if(req->use_original_luminosity_multiplier)
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              else // Use other luminosity multiplier
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              break;
//            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::BIDIRECTIONAL_FIREFLY_OPERATOR:
//              if(req->use_original_luminosity_multiplier)
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              else // Use other luminosity multiplier
//              {
//                switch(req->use_selection)
//                {
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_SELECTION:
//                    if(req->force_no_duplicates)
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    else
//                    {
//                      static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                     fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                     fa::FaFlags::USE_FITNESS_SELECTION);
//                      plan_start_time = std::chrono::high_resolution_clock::now();
//                      const Eigen::Matrix<double,DIM,1> plan =
//                        fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                             rand_init_func,
//                                                             objective_func,
//                                                             stopping_condition_func,
//                                                             logging_func,
//                                                             42+run_ind);
//                      obj_val = objective_func(plan);
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::SUS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_STOCHASTIC_UNIVERSAL_SAMPLING_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::FITNESS_PROPORTIONAL_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::K_MEANS_SELECTION:
//                    if(req->use_elitism_selection)
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_ELITISM_SELECTION bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    else // Don't use elitism selection
//                    {
//                      if(req->force_no_duplicates)
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::FORCE_NO_DUPLICATES bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                      else
//                      {
//                        static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor
//                                                                                       fa::FaFlags::USE_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
//                                                                                       fa::FaFlags::USE_K_MEANS_SELECTION);
//                        plan_start_time = std::chrono::high_resolution_clock::now();
//                        const Eigen::Matrix<double,DIM,1> plan =
//                          fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
//                                                               rand_init_func,
//                                                               objective_func,
//                                                               stopping_condition_func,
//                                                               logging_func,
//                                                               42+run_ind);
//                        obj_val = objective_func(plan);
//                      }
//                    }
//                    break;
//                  default:
//                    std::cout << "Not a valid case" << std::endl;
//                    break;
//                }
//              }
//              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::VEC_FIREFLY_OPERATOR:
              if(req->use_original_luminosity_multiplier)
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              else // other lum
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::VEC_BIDIRECTIONAL_FIREFLY_OPERATOR:
              if(req->use_original_luminosity_multiplier)
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              else // other lum
              {
                if(req->use_elitism_selection)
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_ELITISM_SELECTION);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
                else // Don't use elitism selection
                {
                  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor
                                                                             fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR);
                  plan_start_time = std::chrono::high_resolution_clock::now();
                  const Eigen::Matrix<double,DIM,1> plan =
                    fa::generateFireflyPlan<DIM,FA_CONFIG>(make_params<DIM>(req),
                                                         rand_init_func,
                                                         objective_func,
                                                         stopping_condition_func,
                                                         logging_func,
                                                         42+run_ind);
                  obj_val = objective_func(plan);
                }
              }
              break;
            case graph_surveillance_planning_msgs::srv::TryFireflyAlgorithmConfig::Request::PSO:
              {
                pso::ParticleSwarmOptimization<double,Objective<DIM>,pso::ConstantWeight<double>,Callback<DIM>> optimizer(42+run_ind);
                optimizer.setThreads(64);
                optimizer.setMaxIterations(0);
                optimizer.setMinParticleChange(0);
                optimizer.setMinFunctionChange(0);
                optimizer.setPhiParticles(req->pso_phi_particles);
                optimizer.setPhiGlobal(req->pso_phi_global);
                optimizer.setMaxVelocity(0);
                optimizer.setVerbosity(0);
                optimizer.setObjective(Objective<DIM>(objective_func));
                optimizer.setCallback(Callback<DIM>(stopping_condition_func));

                Eigen::Matrix<double,2,DIM> bounds;
                bounds.row(0).array() = -max_val;
                bounds.row(1).array() =  max_val;

                plan_start_time = std::chrono::high_resolution_clock::now();
                obj_val = std::pair<bool,double>(true, optimizer.minimize(bounds, req->population_size).fval);
              }
              break;
            default:
              std::cout << "Not a valid case" << std::endl;
              break;
          }
          assert(obj_val.first);
          res->costs.emplace_back(obj_val.second);
          res->times.emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9));
        }
        #undef DIM
        break;
      default:
        std::cout << "Not a valid case" << std::endl;
        break;
    }
    malloc_trim(0);
  });

  rclcpp::spin(node);
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}

/* firefly_tuner_server.cpp */
