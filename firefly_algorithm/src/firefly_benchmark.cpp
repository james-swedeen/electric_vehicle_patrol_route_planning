/**
 * @File: firefly_benchmark.cpp
 * @Date: August 2024
 * @Author: James Swedeen
 *
 * @brief
 * Simple tuning node for picking the hyper-parameters in the firefly algorithm.
 **/

/* C++ Headers */
#include<malloc.h>
#include<atomic>
#include<vector>
#include<filesystem>
#include<fstream>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Plotting Headers */
#include<matplotlibcpp/matplotlibcpp.hpp>
//#include<libInterpolate/Interpolate.hpp> // From https://github.com/CD3/libInterpolate

/* Local Headers */
#include<firefly_algorithm/firefly_algorithm.hpp>
#include<firefly_algorithm/psocpp.h>


/// Global Parameters
inline static constexpr const size_t NUM_RUNS = 25;
//inline static constexpr const double max_time_sec  = 300;
//inline static constexpr const double max_obj_evals = 3e6;
inline static constexpr const double plot_dt  = 0.001;

template<Eigen::Index DIM>
class Callback
{
public:
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrix;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;
    typedef long int Index;

    std::function<bool(const size_t,const double)> m_stop_func;
    std::function<void(const Vector&)> m_log_func;

    Callback() = default;
    Callback(const std::function<bool(const size_t,const double)>& stop_func,
        const std::function<void(const Vector&)> log_func)
     : m_stop_func(stop_func),
       m_log_func(log_func)
    {}

    bool operator()(const Index num_iterations, const Matrix& population, const Vector& population_costs, const Index best_ind) const
    {
      this->m_log_func(population_costs);
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


// Helper functions
template<Eigen::Index DIM>
fa::FaParamsPtr<DIM> make_params()
{
  fa::FaParamsPtr<DIM> fa_params = std::make_shared<fa::FaParams<DIM>>();
  fa_params->population_size           = 29;
  fa_params->dist_pow                  = 2.208949045286645;
  fa_params->dist_mult                 = 0.05310183122845463;
  fa_params->base_attractiveness_mult  = 91.93174952240038;
  fa_params->min_attractiveness_mult   = 0.007992945377484274;
  fa_params->mutation_magnitude        = Eigen::Matrix<double,DIM,1>::Constant(5.481424232334534);
  fa_params->number_clusters           = 23;
  fa_params->number_cluster_iterations = 1;

  return fa_params;
}
template<Eigen::Index DIM, fa::FaFlags PLAN_CONFIG, fa::FaFlags SELECT_CONFIG = fa::FaFlags::USE_ELITISM_SELECTION, fa::FaFlags DEFAULT_CONFIG = fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER>
void make_prob_func(
    std::vector<std::string>&                                                                          problem_names,
    std::vector<std::function<void(const size_t, const size_t, const size_t)>>&                        problem_funcs,
    std::vector<Eigen::Matrix<std::vector<double>,                                NUM_RUNS,1>>&        time_sec,
    //std::vector<Eigen::Matrix<std::vector<uint64_t>,                              NUM_RUNS,1>>&        obj_eval_counts,
    std::vector<Eigen::Matrix<std::vector<double>,NUM_RUNS,1>>&        population_costs,
    std::vector<double>&                                                                               max_times,
    const std::string&                                                                                 base_name,
    const std::function<Eigen::Matrix<double,DIM,1>(const unsigned)>&                                  init_func,
    const std::function<std::pair<bool,double>(const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>&)>& cost_func,
    const fa::FaParamsPtr<DIM>                                                                         fa_params,
    const double                                                                                       target_cost,
    const double                                                                                       max_time_sec,
    //const uint64_t                                                                                       max_obj_eval,
    const double                                                                                       pso_phi_particles = 0,
    const double                                                                                       pso_phi_global = 0,
    const double                                                                                       max_val = 0)
{
  std::string name = base_name;
  name.append("_d" + std::to_string(DIM));
  switch(PLAN_CONFIG)
  {
    case fa::FaFlags::NULL_FLAG:
      name.append("_pso");
      break;
    case fa::FaFlags::USE_FITNESS_SELECTION bitor fa::FaFlags::USE_MUTATIONS:
      name.append("_es");
      break;
    case fa::FaFlags::USE_VEC_FIREFLY_OPERATOR:
      name.append("_fa");
      break;
    case fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR:
      name.append("_fab");
      break;
  };
  switch(SELECT_CONFIG)
  {
    case fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION:
      name.append("_fps");
      break;
    case fa::FaFlags::USE_K_MEANS_SELECTION:
      name.append("_kmeans");
      break;
  };
  problem_names.emplace_back(name);
  max_times.emplace_back(max_time_sec);
  problem_funcs.emplace_back([/*max_obj_eval,&obj_eval_counts,*/max_val,pso_phi_particles,pso_phi_global,max_time_sec,&time_sec,&population_costs,target_cost,init_func,cost_func,fa_params] (const size_t seed, const size_t problem_ind, const size_t run_ind) -> void
  {
//    std::atomic_uint64_t obj_eval_count  = 0;
    auto                 plan_start_time = std::chrono::high_resolution_clock::now();

//    const std::function<std::pair<bool,double>(const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>&)> full_cost_func =
//    [&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
//    {
//      ++obj_eval_count;
//      return cost_func(x);
//    };
    std::function<bool(const size_t, const double)> stop_func;
//    if(0 == run_ind)
//    {
//      stop_func =
//        [&] (const size_t generation_count, const double best_cost_found) -> bool
//        {
//          return (max_time_sec < time_sec[problem_ind][run_ind].back());// and
//          //return (max_time_sec < (double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9)));// and
////                 (max_obj_evals < obj_eval_count);
//        };
//    }
//    else // Not the first run
//    {
      stop_func =
        [&] (const size_t generation_count, const double best_cost_found) -> bool
        {
          return (best_cost_found <= target_cost) or
                 (max_time_sec < time_sec[problem_ind][run_ind].back());// or
//                 ((max_time_sec < (double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - plan_start_time).count()) * double(1.0e-9))));// and
//                  (max_obj_eval < obj_eval_count);
        };
//    }
    time_sec[        problem_ind][run_ind].reserve(100000000);
    population_costs[problem_ind][run_ind].reserve(100000000);
//    obj_eval_counts[ problem_ind][run_ind].reserve(100000000);
    auto                 last_start_time = std::chrono::high_resolution_clock::now();
    plan_start_time = std::chrono::high_resolution_clock::now();
    if constexpr(fa::FaFlags::NULL_FLAG == PLAN_CONFIG)
    {
      pso::ParticleSwarmOptimization<double,Objective<DIM>,pso::ConstantWeight<double>,Callback<DIM>> optimizer(seed);
      optimizer.setThreads(64);
      optimizer.setMaxIterations(0);
      optimizer.setMinParticleChange(0);
      optimizer.setMinFunctionChange(0);
      optimizer.setPhiParticles(pso_phi_particles);
      optimizer.setPhiGlobal(pso_phi_global);
      optimizer.setMaxVelocity(0);
      optimizer.setVerbosity(0);
      optimizer.setObjective(Objective<DIM>(cost_func));
      optimizer.setCallback(Callback<DIM>(stop_func,
        [&] (const Eigen::Matrix<double,Eigen::Dynamic,1>& population_objs) -> void
        {
          auto log_start_time = std::chrono::high_resolution_clock::now();

          if(time_sec[problem_ind][run_ind].empty())
          {
            time_sec[problem_ind][run_ind].emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(log_start_time - last_start_time).count()) * double(1.0e-9));
          }
          else
          {
            time_sec[problem_ind][run_ind].emplace_back((double(std::chrono::duration_cast<std::chrono::nanoseconds>(log_start_time - last_start_time).count()) * double(1.0e-9)) + time_sec[problem_ind][run_ind].back());
          }
          population_costs[problem_ind][run_ind].emplace_back(population_objs.minCoeff());
//          obj_eval_counts[ problem_ind][run_ind].emplace_back(obj_eval_count);

          last_start_time = std::chrono::high_resolution_clock::now();
        }));

        Eigen::Matrix<double,2,DIM> bounds;
        bounds.row(0).array() = -max_val;
        bounds.row(1).array() =  max_val;

        optimizer.minimize(bounds, fa_params->population_size);
    }
    else // Not PSO
    {
      fa::generateFireflyPlan<DIM,fa::FaFlags(DEFAULT_CONFIG bitor PLAN_CONFIG bitor SELECT_CONFIG)>(
        fa_params,
        init_func,
        cost_func,
        stop_func,
        [&] (const Eigen::Matrix<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>,Eigen::Dynamic,1>& cur_population) -> void
        {
          auto log_start_time = std::chrono::high_resolution_clock::now();

          if(time_sec[problem_ind][run_ind].empty())
          {
            time_sec[problem_ind][run_ind].emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(log_start_time - last_start_time).count()) * double(1.0e-9));
          }
          else
          {
            time_sec[problem_ind][run_ind].emplace_back((double(std::chrono::duration_cast<std::chrono::nanoseconds>(log_start_time - last_start_time).count()) * double(1.0e-9)) + time_sec[problem_ind][run_ind].back());
          }
          population_costs[problem_ind][run_ind].emplace_back(cur_population.unaryExpr([] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& i) -> double { return i->second; }).minCoeff());
//          obj_eval_counts[ problem_ind][run_ind].emplace_back(obj_eval_count);

          last_start_time = std::chrono::high_resolution_clock::now();
        },
        seed);
    }
    time_sec[        problem_ind][run_ind].emplace_back(time_sec[        problem_ind][run_ind].back() + 1.0e-3);
    population_costs[problem_ind][run_ind].emplace_back(population_costs[problem_ind][run_ind].back());
//    obj_eval_counts[ problem_ind][run_ind].emplace_back(obj_eval_counts[ problem_ind][run_ind].back() + 1);
    time_sec[        problem_ind][run_ind].emplace_back(time_sec[        problem_ind][run_ind].back() + 1.0e-3);
    population_costs[problem_ind][run_ind].emplace_back(population_costs[problem_ind][run_ind].back());
//    obj_eval_counts[ problem_ind][run_ind].emplace_back(obj_eval_counts[ problem_ind][run_ind].back() + 1);
//    if(time_sec[problem_ind][run_ind].back() < max_time_sec)
//    {
//      time_sec[        problem_ind][run_ind].emplace_back(max_time_sec);
//      population_costs[problem_ind][run_ind].emplace_back(population_costs[problem_ind][run_ind].back());
//    }
    time_sec[        problem_ind][run_ind].shrink_to_fit();
    population_costs[problem_ind][run_ind].shrink_to_fit();
//    obj_eval_counts[ problem_ind][run_ind].shrink_to_fit();
  });
}
// From https://stackoverflow.com/questions/9394867/c-implementation-of-matlab-interp1-function-linear-interpolation
template<typename Real1, typename Real2>
int nearestNeighbourIndex(std::vector<Real1> &x, Real2 &value)
{
    double dist = std::numeric_limits<double>::max();
    double newDist = dist;
    size_t idx = 0;

    for (size_t i = 0; i < x.size(); ++i) {
        newDist = std::abs(value - x[i]);
        if (newDist <= dist) {
            dist = newDist;
            idx = i;
        }
    }

    return idx;
}
template<typename Real1, typename Real2, typename Real3>
std::vector<double> interp1(std::vector<Real1> &x, std::vector<Real2> &y, std::vector<Real3> &x_new)
{
    std::vector<double> y_new;
    double dx, dy, m, b;
    size_t x_max_idx = x.size() - 1;
    size_t x_new_size = x_new.size();

    const boost::integer_range<size_t> inds(0, x_new_size);

    y_new.resize(x_new_size);

    std::for_each(std::execution::par_unseq, inds.begin(), inds.end(),
    [&] (const size_t i) -> void
    {
        size_t idx = nearestNeighbourIndex(x, x_new[i]);

        if (x[idx] > x_new[i])
        {
            dx = idx > 0 ? (x[idx] - x[idx - 1]) : (x[idx + 1] - x[idx]);
            dy = idx > 0 ? (y[idx] - y[idx - 1]) : (y[idx + 1] - y[idx]);
        }
        else
        {
            dx = idx < x_max_idx ? (x[idx + 1] - x[idx]) : (x[idx] - x[idx - 1]);
            dy = idx < x_max_idx ? (y[idx + 1] - y[idx]) : (y[idx] - y[idx - 1]);
        }

        m = dy / dx;
        b = y[idx] - x[idx] * m;

        y_new[i] = x_new[i] * m + b;
    });

    return y_new;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("firefly_benchmark_node");

  node->declare_parameter("problem_ind",  rclcpp::PARAMETER_INTEGER);
  const long problem_ind = node->get_parameter("problem_ind").as_int();

  // ind 1: problem, ind 2: run, ind 3: generation
  std::vector<Eigen::Matrix<std::vector<double>,                                NUM_RUNS,1>> time_sec;        // To reach that generation
//  std::vector<Eigen::Matrix<std::vector<uint64_t>,                              NUM_RUNS,1>> obj_eval_counts; // To reach that generation
  std::vector<Eigen::Matrix<std::vector<double>,NUM_RUNS,1>> population_costs;

  std::vector<double>                                                        max_times;
//  std::vector<uint64_t>                                                        max_obj;
  std::vector<std::string>                                                   problem_names;
  std::vector<std::function<void(const size_t, const size_t, const size_t)>> problem_funcs; // seed, problem index, run index

  /// Make problems
//  {
//    #define TDIM 16
//    fa::FaParamsPtr<TDIM> orig_params = std::make_shared<fa::FaParams<TDIM>>();
//    orig_params->population_size           = 40;
//    orig_params->dist_pow                  = 2;
//    orig_params->dist_mult                 = 1;
//    orig_params->base_attractiveness_mult  = 1;
//    orig_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.2);
//    orig_params->number_clusters           = 19;
//    orig_params->number_cluster_iterations = 2;
//    make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR bitor fa::FaFlags::USE_FIREFLY_OPERATOR),fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION,fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor fa::FaFlags::USE_ELITISM_SELECTION bitor fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER)>(
//      problem_names, problem_funcs, time_sec, obj_eval_counts, population_costs,
//      "rosenbrock_orig_settings",
//      [&] (const unsigned rand_seed) -> Eigen::Matrix<double,TDIM,1>
//      {
//        static thread_local std::default_random_engine rand_gen(rand_seed);
//        static constexpr const double max = 5;
//        std::uniform_real_distribution<double> probability_dist(-max, max);
//        return Eigen::Matrix<double,TDIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//      },
//      [&] (const Eigen::Ref<const Eigen::Matrix<double,TDIM,1>>& x) -> std::pair<bool,double>
//      {
//        static constexpr const double max = 5;
//        if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
//        double output = 0;
//        for(Eigen::Index dim_ind = 0; dim_ind < (TDIM - 1); ++dim_ind)
//        {
//          output += (std::pow(x[dim_ind] - double(1), double(2)) + (double(100) * std::pow(x[dim_ind + 1] - std::pow(x[dim_ind], double(2)), double(2))));
//        }
//        return std::pair<bool,double>(true, output);
//      },
//      orig_params);
//    #undef TDIM
//  }
//  {
//    #define TDIM 16
//    const std::string name = "rosenbrock";
//    const auto init_func =
//    [&] (const unsigned rand_seed) -> Eigen::Matrix<double,TDIM,1>
//    {
//      static thread_local std::default_random_engine rand_gen(rand_seed);
//      static constexpr const double max = 5;
//      std::uniform_real_distribution<double> probability_dist(-max, max);
//      return Eigen::Matrix<double,TDIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//    };
//    const auto cost_func =
//    [&] (const Eigen::Ref<const Eigen::Matrix<double,TDIM,1>>& x) -> std::pair<bool,double>
//    {
//      static constexpr const double max = 5;
//      if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
//      double output = 0;
//      for(Eigen::Index dim_ind = 0; dim_ind < (TDIM - 1); ++dim_ind)
//      {
//        output += (std::pow(x[dim_ind] - double(1), double(2)) + (double(100) * std::pow(x[dim_ind + 1] - std::pow(x[dim_ind], double(2)), double(2))));
//      }
//      return std::pair<bool,double>(true, output);
//    };
//    const fa::FaParamsPtr<TDIM> fa_params = make_params<TDIM>();
//    make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_FIREFLY_OPERATOR bitor fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR),fa::FaFlags::USE_K_MEANS_SELECTION>(
//      problem_names, problem_funcs, time_sec, obj_eval_counts, population_costs, name, init_func, cost_func, fa_params);
//    make_prob_func<TDIM,fa::FaFlags::USE_FIREFLY_OPERATOR,fa::FaFlags::USE_K_MEANS_SELECTION>(
//      problem_names, problem_funcs, time_sec, obj_eval_counts, population_costs, name, init_func, cost_func, fa_params);
//    make_prob_func<TDIM,fa::FaFlags::NULL_FLAG,fa::FaFlags::USE_K_MEANS_SELECTION>(
//      problem_names, problem_funcs, time_sec, obj_eval_counts, population_costs, name, init_func, cost_func, fa_params);
//    make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_FIREFLY_OPERATOR bitor fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR),fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION>(
//      problem_names, problem_funcs, time_sec, obj_eval_counts, population_costs, name, init_func, cost_func, fa_params);
//    make_prob_func<TDIM,fa::FaFlags::USE_FIREFLY_OPERATOR,fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION>(
//      problem_names, problem_funcs, time_sec, obj_eval_counts, population_costs, name, init_func, cost_func, fa_params);
//    make_prob_func<TDIM,fa::FaFlags::NULL_FLAG,fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION>(
//      problem_names, problem_funcs, time_sec, obj_eval_counts, population_costs, name, init_func, cost_func, fa_params);
//    #undef TDIM
//  }
//  {
//    #define TDIM 4
//    const std::string name = "rosenbrock";
//    const auto init_func =
//    [&] (const unsigned rand_seed) -> Eigen::Matrix<double,TDIM,1>
//    {
//      static thread_local std::default_random_engine rand_gen(rand_seed);
//      static constexpr const double max = 5;
//      std::uniform_real_distribution<double> probability_dist(-max, max);
//      return Eigen::Matrix<double,TDIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//    };
//    const auto cost_func =
//    [&] (const Eigen::Ref<const Eigen::Matrix<double,TDIM,1>>& x) -> std::pair<bool,double>
//    {
//      static constexpr const double max = 5;
//      if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
//      double output = 0;
//      for(Eigen::Index dim_ind = 0; dim_ind < (TDIM - 1); ++dim_ind)
//      {
//        output += (std::pow(x[dim_ind] - double(1), double(2)) + (double(100) * std::pow(x[dim_ind + 1] - std::pow(x[dim_ind], double(2)), double(2))));
//      }
//      return std::pair<bool,double>(true, output);
//    };
//    const fa::FaParamsPtr<TDIM> fa_params = make_params<TDIM>();
//    make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_FIREFLY_OPERATOR bitor fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR),fa::FaFlags::USE_K_MEANS_SELECTION>(
//      problem_names, problem_funcs, time_sec, obj_eval_counts, population_costs, name, init_func, cost_func, fa_params);
//    make_prob_func<TDIM,fa::FaFlags::USE_FIREFLY_OPERATOR,fa::FaFlags::USE_K_MEANS_SELECTION>(
//      problem_names, problem_funcs, time_sec, obj_eval_counts, population_costs, name, init_func, cost_func, fa_params);
//    make_prob_func<TDIM,fa::FaFlags::NULL_FLAG,fa::FaFlags::USE_K_MEANS_SELECTION>(
//      problem_names, problem_funcs, time_sec, obj_eval_counts, population_costs, name, init_func, cost_func, fa_params);
//    make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_FIREFLY_OPERATOR bitor fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR),fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION>(
//      problem_names, problem_funcs, time_sec, obj_eval_counts, population_costs, name, init_func, cost_func, fa_params);
//    make_prob_func<TDIM,fa::FaFlags::USE_FIREFLY_OPERATOR,fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION>(
//      problem_names, problem_funcs, time_sec, obj_eval_counts, population_costs, name, init_func, cost_func, fa_params);
//    make_prob_func<TDIM,fa::FaFlags::NULL_FLAG,fa::FaFlags::USE_FITNESS_PROPORTIONAL_SELECTION>(
//      problem_names, problem_funcs, time_sec, obj_eval_counts, population_costs, name, init_func, cost_func, fa_params);
//    #undef TDIM
//  }



  //// FOR PAPER
  switch(problem_ind)
  {
    case 0:
      {
        #define TDIM 2
        const std::string name = "easom";
        fa::FaParamsPtr<TDIM> fab_params = std::make_shared<fa::FaParams<TDIM>>();
//        fab_params->population_size           = 34;
//        fab_params->dist_pow                  = 2;
//        fab_params->dist_mult                 = 115.02307681865544;
//        fab_params->base_attractiveness_mult  = 8.861773913573899;
//        fab_params->min_attractiveness_mult   = 2.684663374513947;
//        fab_params->cost_scaling_mult         = 94.69864464821383;
//        fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.01734331949064676);
        fab_params->base_attractiveness_mult  = 52.824898942574684;
        fab_params->cost_scaling_mult         = 342.37926026724654;
        fab_params->dist_mult                 = 97.75550256430029;
        fab_params->dist_pow                  = 2;
        fab_params->from_cost_scaling_mult    = 0.7525989311589769;
        fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.008019329683962762);
        fab_params->population_size           = 58;
        fa::FaParamsPtr<TDIM> fa_params = std::make_shared<fa::FaParams<TDIM>>();
        fa_params->population_size           = 31;
        fa_params->dist_pow                  = 2;
        fa_params->dist_mult                 = 51.87250578143583;
        fa_params->base_attractiveness_mult  = 5.179411627801229;
        fa_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.00789742067903547);
        fa::FaParamsPtr<TDIM> se_params = std::make_shared<fa::FaParams<TDIM>>();
        se_params->population_size           = 104;
        se_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.047956082484395327);
        fa::FaParamsPtr<TDIM> pso_params = std::make_shared<fa::FaParams<TDIM>>();
        pso_params->population_size    = 112;
        const double pso_phi_particles = 1.2314527161908053;
        const double pso_phi_global    = 0.009205807114139118;
        static constexpr const double max = double(2) * double(M_PI);
        const auto init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,TDIM,1>
        {
          static thread_local std::default_random_engine rand_gen(rand_seed);
          std::uniform_real_distribution<double> probability_dist(-max, max);
          return Eigen::Matrix<double,TDIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
        };
        const auto cost_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,TDIM,1>>& x) -> std::pair<bool,double>
        {
          assert(not x.array().isNaN().any());
          if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
          return std::pair<bool,double>(true, (
                 double(-std::pow(-1, TDIM))
                 * x.array().cos().square().prod()
                 * std::exp(-(x.array() - double(M_PI)).square().sum())));
        };
        const double target_cost = -1.0 + 1.0e-8;
        const double max_time_sec = 5.0;
        make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR)>(
          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, fab_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags::USE_VEC_FIREFLY_OPERATOR>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, fa_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_FITNESS_SELECTION bitor fa::FaFlags::USE_MUTATIONS)>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, se_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags::NULL_FLAG>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, pso_params, target_cost, max_time_sec, pso_phi_particles, pso_phi_global, max);
        #undef TDIM
      }
      break;
    case 1:
      {
        #define TDIM 4
        const std::string name = "rastrigin";
        fa::FaParamsPtr<TDIM> fab_params = std::make_shared<fa::FaParams<TDIM>>();
//        fab_params->population_size           = 123;
//        fab_params->dist_pow                  = 2;
//        fab_params->dist_mult                 = 0.20184710499694955;
//        fab_params->base_attractiveness_mult  = 4.67484619372063;
//        fab_params->min_attractiveness_mult   = 3.76680484212092;
//        fab_params->cost_scaling_mult         = 78.47766385305826;
//        fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.0006834393322304455);
        fab_params->base_attractiveness_mult  = 83.99716572451158;
        fab_params->cost_scaling_mult         = 705.6336339052813;
        fab_params->dist_mult                 = 59.390386989206576;
        fab_params->dist_pow                  = 2;
        fab_params->from_cost_scaling_mult    = 0.5211263634646235;
        fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.44969329648943707);
        fab_params->population_size           = 83;
        fa::FaParamsPtr<TDIM> fa_params = std::make_shared<fa::FaParams<TDIM>>();
        fa_params->population_size           = 140;
        fa_params->dist_pow                  = 2;
        fa_params->dist_mult                 = 43.55804196064627;
        fa_params->base_attractiveness_mult  = 9.544827255108395;
        fa_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.40672112655434367);
        fa::FaParamsPtr<TDIM> se_params = std::make_shared<fa::FaParams<TDIM>>();
        se_params->population_size           = 78;
        se_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.7196021338096965);
        fa::FaParamsPtr<TDIM> pso_params = std::make_shared<fa::FaParams<TDIM>>();
        pso_params->population_size    = 120;
        const double pso_phi_particles = 0.07415732740988068;
        const double pso_phi_global    = 1.1956907694153793;
        static constexpr const double max = 5.12;
        const auto init_func =
        [&] (const unsigned rand_seed) -> Eigen::Matrix<double,TDIM,1>
        {
          static thread_local std::default_random_engine rand_gen(rand_seed);
          std::uniform_real_distribution<double> probability_dist(-max, max);
          return Eigen::Matrix<double,TDIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
        };
        const auto cost_func =
        [&] (const Eigen::Ref<const Eigen::Matrix<double,TDIM,1>>& x) -> std::pair<bool,double>
        {
          if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
          static constexpr const double A = 10;
          return std::pair<bool,double>(true, (A * TDIM) + (x.array().square() - (A * (x.array() * double(2) * double(M_PI)).cos())).sum());
        };
        const double target_cost = 1.0e-6;
        const double max_time_sec = 90;
        make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR)>(
          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, fab_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags::USE_VEC_FIREFLY_OPERATOR>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, fa_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_FITNESS_SELECTION bitor fa::FaFlags::USE_MUTATIONS)>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, se_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags::NULL_FLAG>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, pso_params, target_cost, max_time_sec, pso_phi_particles, pso_phi_global, max);
        #undef TDIM
      }
      break;
    case 2:
      {
        #define TDIM 8
        const std::string name = "griewank";
        fa::FaParamsPtr<TDIM> fab_params = std::make_shared<fa::FaParams<TDIM>>();
//        fab_params->population_size           = 24;
//        fab_params->dist_pow                  = 2;
//        fab_params->dist_mult                 = 66.09901909877112;
//        fab_params->base_attractiveness_mult  = 0.3254683246966952;
//        fab_params->min_attractiveness_mult   = 3.912875058242717;
//        fab_params->cost_scaling_mult         = 62.98495878952601;
//        fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(9.127077250257482);
        fab_params->base_attractiveness_mult  = 114.22942514491919;
        fab_params->cost_scaling_mult         = 663.1587727842052;
        fab_params->dist_mult                 = 78.80962656168552;
        fab_params->dist_pow                  = 2;
        fab_params->from_cost_scaling_mult    = 0.5729129428954325;
        fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(10.349113929202082);
        fab_params->population_size           = 3;
        fa::FaParamsPtr<TDIM> fa_params = std::make_shared<fa::FaParams<TDIM>>();
        fa_params->population_size           = 19;
        fa_params->dist_pow                  = 2;
        fa_params->dist_mult                 = 65.46980234425531;
        fa_params->base_attractiveness_mult  = 8.011019779435252;
        fa_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(10.553792878884593);
        fa::FaParamsPtr<TDIM> se_params = std::make_shared<fa::FaParams<TDIM>>();
        se_params->population_size           = 125;
        se_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(2.5159039023143723);
        fa::FaParamsPtr<TDIM> pso_params = std::make_shared<fa::FaParams<TDIM>>();
        pso_params->population_size    = 70;
        const double pso_phi_particles = 1.499453153028152;
        const double pso_phi_global    = 0.0067205212344112475;
        static constexpr const double max = 600;
        const auto init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,TDIM,1>
        {
          static thread_local std::default_random_engine rand_gen(rand_seed);
          std::uniform_real_distribution<double> probability_dist(-max, max);
          return Eigen::Matrix<double,TDIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
        };
        const auto cost_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,TDIM,1>>& x) -> std::pair<bool,double>
        {
          assert(not x.array().isNaN().any());
          if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
          return std::pair<bool,double>(true, double(1)
                 + ((double(1)/double(4000)) * x.array().square().sum())
                 - (x.array() / Eigen::Array<double,TDIM,1>::LinSpaced(TDIM, 1, TDIM).sqrt()).cos().prod());
        };
        const double target_cost = 1.0e-8;
        const double max_time_sec = 60;
        make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR)>(
          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, fab_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags::USE_VEC_FIREFLY_OPERATOR>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, fa_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_FITNESS_SELECTION bitor fa::FaFlags::USE_MUTATIONS)>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, se_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags::NULL_FLAG>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, pso_params, target_cost, max_time_sec, pso_phi_particles, pso_phi_global, max);
        #undef TDIM
      }
      break;
    case 3:
      {
        #define TDIM 32
        const std::string name = "schwefel";
        fa::FaParamsPtr<TDIM> fab_params = std::make_shared<fa::FaParams<TDIM>>();
//        fab_params->population_size           = 73;
//        fab_params->dist_pow                  = 2;
//        fab_params->dist_mult                 = 44.589378689224105;
//        fab_params->base_attractiveness_mult  = 7.916081512541565;
//        fab_params->min_attractiveness_mult   = 2.590983338150115;
//        fab_params->cost_scaling_mult         = 29.752568694541214;
//        fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(10.07736052530565);
        fab_params->base_attractiveness_mult  = 133.98977865057208;
        fab_params->cost_scaling_mult         = 7.135997959564463;
        fab_params->dist_mult                 = 58.49056054860544;
        fab_params->dist_pow                  = 2;
        fab_params->from_cost_scaling_mult    = 0.9019105563553946;
        fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(6.22590152251251);
        fab_params->population_size           = 89;
        fa::FaParamsPtr<TDIM> fa_params = std::make_shared<fa::FaParams<TDIM>>();
        fa_params->population_size           = 96;
        fa_params->dist_pow                  = 2;
        fa_params->dist_mult                 = 91.51912152133104;
        fa_params->base_attractiveness_mult  = 5.5251785716257;
        fa_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(7.400346000664275);
        fa::FaParamsPtr<TDIM> se_params = std::make_shared<fa::FaParams<TDIM>>();
        se_params->population_size           = 84;
        se_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.2415713600697234);
        fa::FaParamsPtr<TDIM> pso_params = std::make_shared<fa::FaParams<TDIM>>();
        pso_params->population_size    = 146;
        const double pso_phi_particles = 2.067472680978466;
        const double pso_phi_global    = 0.6393631964391484;
        static constexpr const double max = 500;
        const auto init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,TDIM,1>
        {
          static thread_local std::default_random_engine rand_gen(rand_seed);
          std::uniform_real_distribution<double> probability_dist(-max, max);
          return Eigen::Matrix<double,TDIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
        };
        const auto cost_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,TDIM,1>>& x) -> std::pair<bool,double>
        {
          assert(not x.array().isNaN().any());
          if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
          return std::pair<bool,double>(true, -(x.array() * x.array().abs().sqrt().sin()).sum());
        };
        const double target_cost = -(double(418.9829) * double(TDIM)) + 10.0;
        const double max_time_sec = 30;
        make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR)>(
          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, fab_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags::USE_VEC_FIREFLY_OPERATOR>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, fa_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_FITNESS_SELECTION bitor fa::FaFlags::USE_MUTATIONS)>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, se_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags::NULL_FLAG>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, pso_params, target_cost, max_time_sec, pso_phi_particles, pso_phi_global, max);
        #undef TDIM
      }
      break;
    case 4:
      {
        #define TDIM 64
        const std::string name = "ackley";
        fa::FaParamsPtr<TDIM> fab_params = std::make_shared<fa::FaParams<TDIM>>();
//        fab_params->population_size           = 36;
//        fab_params->dist_pow                  = 2;
//        fab_params->dist_mult                 = 30.422296494435674;
//        fab_params->base_attractiveness_mult  = 2.8773560843776904;
//        fab_params->min_attractiveness_mult   = 0.00304057752161363;
//        fab_params->cost_scaling_mult         = 81.86985355188835;
//        fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(2.5121453527836475);
        fab_params->base_attractiveness_mult  = 32.88369407100235;
        fab_params->cost_scaling_mult         = 521.0453939673549;
        fab_params->dist_mult                 = 50.49924167576104;
        fab_params->dist_pow                  = 2;
        fab_params->from_cost_scaling_mult    = 0.5411865531921342;
        fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(2.153311935634584);
        fab_params->population_size           = 8;
        fa::FaParamsPtr<TDIM> fa_params = std::make_shared<fa::FaParams<TDIM>>();
        fa_params->population_size           = 6;
        fa_params->dist_pow                  = 2;
        fa_params->dist_mult                 = 58.626411356763626;
        fa_params->base_attractiveness_mult  = 1.2974163536146144;
        fa_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(3.2178897541425022);
        fa::FaParamsPtr<TDIM> se_params = std::make_shared<fa::FaParams<TDIM>>();
        se_params->population_size           = 122;
        se_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.5464011311513276);
        fa::FaParamsPtr<TDIM> pso_params = std::make_shared<fa::FaParams<TDIM>>();
        pso_params->population_size    = 105;
        const double pso_phi_particles = 7.299400269262508;
        const double pso_phi_global    = 0.027890401869921533;
        static constexpr const double max = 32.768;
        const auto init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,TDIM,1>
        {
          static thread_local std::default_random_engine rand_gen(rand_seed);
          std::uniform_real_distribution<double> probability_dist(-max, max);
          return Eigen::Matrix<double,TDIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
        };
        const auto cost_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,TDIM,1>>& x) -> std::pair<bool,double>
        {
          assert(not x.array().isNaN().any());
          if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
          static constexpr const double a = 20;
          static constexpr const double b = 0.2;
          static constexpr const double c = double(2) * double(M_PI);
          return std::pair<bool,double>(true, - a * std::exp(-b * std::sqrt(x.array().square().sum() / double(TDIM)))
                 - std::exp((double(1) / double(TDIM)) * (x.array() * c).cos().sum())
                 + a + std::exp(1));
        };
        const double target_cost = 1.0e-3;
        const double max_time_sec = 90;
        make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR)>(
          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, fab_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags::USE_VEC_FIREFLY_OPERATOR>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, fa_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_FITNESS_SELECTION bitor fa::FaFlags::USE_MUTATIONS)>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, se_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags::NULL_FLAG>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, pso_params, target_cost, max_time_sec, pso_phi_particles, pso_phi_global, max);
        #undef TDIM
      }
      break;
    case 5:
      {
        #define TDIM 128
        const std::string name = "rosenbrock";
        fa::FaParamsPtr<TDIM> fab_params = std::make_shared<fa::FaParams<TDIM>>();
//        fab_params->population_size           = 37;
//        fab_params->dist_pow                  = 2;
//        fab_params->dist_mult                 = 147.9530219731089; // Might want to rerun with bigger bound
//        fab_params->base_attractiveness_mult  = 5.314809620762323;
//        fab_params->min_attractiveness_mult   = 1.8411443845319153;
//        fab_params->cost_scaling_mult         = 69.13968197688;
//        fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.006472552484410865);
        fab_params->base_attractiveness_mult  = 2.4805789116371386;
        fab_params->cost_scaling_mult         = 84.01573377171286;
        fab_params->dist_mult                 = 84.89615242558632;
        fab_params->dist_pow                  = 2;
        fab_params->from_cost_scaling_mult    = 0.7751269093375668;
        fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.0076993679131410996);
        fab_params->population_size           = 75;
        fa::FaParamsPtr<TDIM> fa_params = std::make_shared<fa::FaParams<TDIM>>();
        fa_params->population_size           = 64;
        fa_params->dist_pow                  = 2;
        fa_params->dist_mult                 = 17.30247631801645;
        fa_params->base_attractiveness_mult  = 7.951223807664231;
        fa_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.025461923807252672);
        fa::FaParamsPtr<TDIM> se_params = std::make_shared<fa::FaParams<TDIM>>();
        se_params->population_size           = 101;
        se_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.004977062514946907);
        fa::FaParamsPtr<TDIM> pso_params = std::make_shared<fa::FaParams<TDIM>>();
        pso_params->population_size    = 37;
        const double pso_phi_particles = 9.053399595245216;
        const double pso_phi_global    = 0.00027736015991144947;
        static constexpr const double max = 5;
        const auto init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,TDIM,1>
        {
          static thread_local std::default_random_engine rand_gen(rand_seed);
          std::uniform_real_distribution<double> probability_dist(-max, max);
          return Eigen::Matrix<double,TDIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
        };
        const auto cost_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,TDIM,1>>& x) -> std::pair<bool,double>
        {
          assert(not x.array().isNaN().any());
          if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
          double output = 0;
          for(Eigen::Index dim_ind = 0; dim_ind < (TDIM - 1); ++dim_ind)
          {
            output += (std::pow(x[dim_ind] - double(1), double(2)) + (double(100) * std::pow(x[dim_ind + 1] - std::pow(x[dim_ind], double(2)), double(2))));
          }
          return std::pair<bool,double>(true, output);
        };
        const double target_cost = 1.0;
        const double max_time_sec = 15;
        make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR)>(
          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, fab_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags::USE_VEC_FIREFLY_OPERATOR>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, fa_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_FITNESS_SELECTION bitor fa::FaFlags::USE_MUTATIONS)>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, se_params, target_cost, max_time_sec);
//        make_prob_func<TDIM,fa::FaFlags::NULL_FLAG>(
//          problem_names, problem_funcs, time_sec, population_costs, max_times, name, init_func, cost_func, pso_params, target_cost, max_time_sec, pso_phi_particles, pso_phi_global, max);
        #undef TDIM
      }
      break;
    default:
      std::cout << "hit default problem case" << std::endl;
      break;
  };






//  {
//    #define TDIM 4
//    const std::string name = "rastrigin";
//    fa::FaParamsPtr<TDIM> fab_params = std::make_shared<fa::FaParams<TDIM>>();
//    fab_params->population_size           = 136;
//    fab_params->dist_pow                  = 2;
//    fab_params->dist_mult                 = 0.1554115342495651;
//    fab_params->base_attractiveness_mult  = 0.6658981126756569;
//    fab_params->min_attractiveness_mult   = 0.3412968268124972;
//    fab_params->cost_scaling_mult         = 62.04015293344942;
//    fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.019769129048262613);
////    fab_params->population_size           = 46;
////    fab_params->dist_pow                  = 2;
////    fab_params->dist_mult                 = 0.09966962356359943;
////    fab_params->base_attractiveness_mult  = 1.7088350619700483;
////    fab_params->min_attractiveness_mult   = 0.4228229865552783;
////    fab_params->cost_scaling_mult         = 89.25599281725479;
////    fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.05195820697252948);
////    fab_params->number_clusters           = 0;
////    fab_params->number_cluster_iterations = 0;
//    fa::FaParamsPtr<TDIM> fa_params = std::make_shared<fa::FaParams<TDIM>>();
//    fa_params->population_size           = 76;
//    fa_params->dist_pow                  = 2;
//    fa_params->dist_mult                 = 0.12757529148257057;
//    fa_params->base_attractiveness_mult  = 1.6641313715877675;
//    fa_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.004113061088724538);
////    fa_params->population_size           = 117;
////    fa_params->dist_pow                  = 2;
////    fa_params->dist_mult                 = 4.226961692719356;
////    fa_params->base_attractiveness_mult  = 1.018063558349347;
////    fa_params->min_attractiveness_mult   = 0.0;
////    fa_params->cost_scaling_mult         = 0.0;
////    fa_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.001797075063891207);
////    fa_params->number_clusters           = 0;
////    fa_params->number_cluster_iterations = 0;
//    fa::FaParamsPtr<TDIM> se_params = std::make_shared<fa::FaParams<TDIM>>();
//    se_params->population_size           = 64;
//    se_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.7337448720567362);
////    se_params->population_size           = 141;
////    se_params->dist_pow                  = 0.0;
////    se_params->dist_mult                 = 0.0;
////    se_params->base_attractiveness_mult  = 0.0;
////    se_params->min_attractiveness_mult   = 0.0;
////    se_params->cost_scaling_mult         = 0.0;
////    se_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.1521182097869496);
////    se_params->number_clusters           = 0;
////    se_params->number_cluster_iterations = 0;
//    fa::FaParamsPtr<TDIM> pso_params = std::make_shared<fa::FaParams<TDIM>>();
//    pso_params->population_size           = 54;
//    const double pso_phi_particles = 0.19396198641494855;
//    const double pso_phi_global    = 0.02772061025766509;
//    static constexpr const double max = 5.12;
//    const auto init_func =
//    [&] (const unsigned rand_seed) -> Eigen::Matrix<double,TDIM,1>
//    {
//      static thread_local std::default_random_engine rand_gen(rand_seed);
//      std::uniform_real_distribution<double> probability_dist(-max, max);
//      return Eigen::Matrix<double,TDIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//    };
//    const auto cost_func =
//    [&] (const Eigen::Ref<const Eigen::Matrix<double,TDIM,1>>& x) -> std::pair<bool,double>
//    {
//      if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
//      static constexpr const double A = 10;
//      return std::pair<bool,double>(true, (A * TDIM) + (x.array().square() - (A * (x.array() * double(2) * double(M_PI)).cos())).sum());
//    };
//    const double target_cost = 1.0e-8;
//    const double max_time_sec = 10;
//    make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR)>(
//      problem_names, problem_funcs, time_sec, population_costs, name, init_func, cost_func, fab_params, target_cost, max_time_sec);
//    make_prob_func<TDIM,fa::FaFlags::USE_VEC_FIREFLY_OPERATOR>(
//      problem_names, problem_funcs, time_sec, population_costs, name, init_func, cost_func, fa_params, target_cost, max_time_sec);
//    make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_FITNESS_SELECTION bitor fa::FaFlags::USE_MUTATIONS)>(
//      problem_names, problem_funcs, time_sec, population_costs, name, init_func, cost_func, se_params, target_cost, max_time_sec);
//    make_prob_func<TDIM,fa::FaFlags::NULL_FLAG>(
//      problem_names, problem_funcs, time_sec, population_costs, name, init_func, cost_func, pso_params, target_cost, max_time_sec, pso_phi_particles, pso_phi_global, max);
//    #undef TDIM
//  }
//  {
//    #define TDIM 32
//    const std::string name = "schwefel";
//    fa::FaParamsPtr<TDIM> fab_params = std::make_shared<fa::FaParams<TDIM>>();
//    fab_params->population_size           = 124;
//    fab_params->dist_pow                  = 2;
//    fab_params->dist_mult                 = 93.49474913346933;
//    fab_params->base_attractiveness_mult  = 0.38392415248004763;
//    fab_params->min_attractiveness_mult   = 1.6520068832316515;
//    fab_params->cost_scaling_mult         = 76.65430610369981;
//    fab_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.17645187736276202);
//    fab_params->number_clusters           = 0;
//    fab_params->number_cluster_iterations = 0;
////    fa::FaParamsPtr<TDIM> fa_params = fab_params;
//    fa::FaParamsPtr<TDIM> fa_params = std::make_shared<fa::FaParams<TDIM>>();
//    fa_params->population_size           = 136;
//    fa_params->dist_pow                  = 2;
//    fa_params->dist_mult                 = 110.02778025543692;
//    fa_params->base_attractiveness_mult  = 7.456249741699668;
//    fa_params->min_attractiveness_mult   = 0.0;
//    fa_params->cost_scaling_mult         = 0.0;
//    fa_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.9776305401349249);
//    fa_params->number_clusters           = 0;
//    fa_params->number_cluster_iterations = 0;
////    fa::FaParamsPtr<TDIM> ea_params = fab_params;
//    fa::FaParamsPtr<TDIM> se_params = std::make_shared<fa::FaParams<TDIM>>();
//    se_params->population_size           = 149;
//    se_params->dist_pow                  = 0.0;
//    se_params->dist_mult                 = 0.0;
//    se_params->base_attractiveness_mult  = 0.0;
//    se_params->min_attractiveness_mult   = 0.0;
//    se_params->cost_scaling_mult         = 0.0;
//    se_params->mutation_magnitude        = Eigen::Matrix<double,TDIM,1>::Constant(0.39197702810170704);
//    se_params->number_clusters           = 0;
//    se_params->number_cluster_iterations = 0;
//    const auto init_func =
//    [&] (const unsigned rand_seed) -> Eigen::Matrix<double,TDIM,1>
//    {
//      static thread_local std::default_random_engine rand_gen(rand_seed);
//      static constexpr const double max = 500;
//      std::uniform_real_distribution<double> probability_dist(-max, max);
//      return Eigen::Matrix<double,TDIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//    };
//    const auto cost_func =
//    [&] (const Eigen::Ref<const Eigen::Matrix<double,TDIM,1>>& x) -> std::pair<bool,double>
//    {
//      static constexpr const double max = 500;
//      if((x.array().abs() > max).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
//      return std::pair<bool,double>(true,  - (x.array() * x.array().abs().sqrt().sin()).sum());
//    };
//    const double target_cost = 1.0e-8 - (double(418.9829) * double(TDIM));
//    const double max_time_sec = 15;
//    make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR)>(
//      problem_names, problem_funcs, time_sec, population_costs, name, init_func, cost_func, fab_params, target_cost, max_time_sec);
//    make_prob_func<TDIM,fa::FaFlags::USE_VEC_FIREFLY_OPERATOR>(
//      problem_names, problem_funcs, time_sec, population_costs, name, init_func, cost_func, fa_params, target_cost, max_time_sec);
//    make_prob_func<TDIM,fa::FaFlags(fa::FaFlags::USE_FITNESS_SELECTION bitor fa::FaFlags::USE_MUTATIONS)>(
//      problem_names, problem_funcs, time_sec, population_costs, name, init_func, cost_func, se_params, target_cost, max_time_sec);
//    #undef TDIM
//  }

  assert(problem_names.size() == problem_funcs.size());

  const boost::integer_range<size_t> problem_inds(0, problem_names.size());
  const boost::integer_range<size_t> run_inds(    0, NUM_RUNS);

  time_sec.        resize(problem_names.size());
//  obj_eval_counts. resize(problem_names.size());
  population_costs.resize(problem_names.size());

  /// Run problems
  const size_t total_runs = problem_names.size() * NUM_RUNS;
  const auto   start_time = std::chrono::high_resolution_clock::now();
  size_t       run_count  = 0;
    for(size_t problem_ind = 0; problem_ind < problem_names.size(); ++problem_ind)
  {
  for(size_t run_ind = 0; run_ind < NUM_RUNS; ++run_ind)
    {
      //std::cout << problem_names[problem_ind] << std::endl;
      problem_funcs[problem_ind](42 + run_ind, problem_ind, run_ind);
      ++run_count;
      malloc_trim(0);
//      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      const double percent_finished     = double(run_count) / double(total_runs);
      const double time_taken_sec       = double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start_time).count()) * double(1.0e-9);
      const double time_per_percent_sec = time_taken_sec / percent_finished;
      std::cout << "ETA: " << time_per_percent_sec * (double(1) - percent_finished) / double(60) << " minutes" << std::endl;
    }
  }

  /// Interpolate
  // ind 1: problem, ind 2: run, ind 3: time
  std::vector<std::vector<double>>                           interp_time_sec( problem_names.size());
//  std::vector<std::vector<double>>                           interp_obj_count(problem_names.size());
  std::vector<Eigen::Matrix<std::vector<double>,NUM_RUNS,1>> interp_min_costs(problem_names.size());
//  std::vector<Eigen::Matrix<std::vector<double>,NUM_RUNS,1>> interp_min_costs_obj(problem_names.size());
  std::for_each(std::execution::par_unseq, problem_inds.begin(), problem_inds.end(),
  [&] (const size_t problem_ind) -> void
  {
    const size_t num_time_points = std::ceil(max_times[problem_ind] / plot_dt) + 2;
    const double step            = max_times[problem_ind] / double(num_time_points);
    interp_time_sec[problem_ind].resize(num_time_points);
    interp_time_sec[problem_ind].front() = 0;
    for(size_t time_ind = 1; time_ind < num_time_points; ++time_ind)
    {
      interp_time_sec[problem_ind][time_ind] = interp_time_sec[problem_ind][time_ind - 1] + step;
    }
//    interp_obj_count[problem_ind].resize(10000);
//    interp_obj_count[problem_ind].front() = 0;
//    for(size_t time_ind = 1; time_ind < 10000; ++time_ind)
//    {
//      interp_obj_count[problem_ind][time_ind] = interp_obj_count[problem_ind][time_ind - 1] + std::ceil(max_obj[problem_ind] / double(10000));
//    }
    std::for_each(std::execution::par_unseq, run_inds.begin(), run_inds.end(),
    [&] (const size_t run_ind) -> void
    {
      interp_min_costs[problem_ind][run_ind].resize(num_time_points);
//      interp_min_costs_obj[problem_ind][run_ind].resize(10000);
//      _1D::CubicSplineInterpolator<double> interp;
//      interp.setData(time_sec[problem_ind][run_ind], min_costs);
//      for(size_t time_ind = 0; time_ind < num_time_points; ++time_ind)
//      {
//        interp_min_costs[problem_ind][run_ind][time_ind] = interp(interp_time_sec[problem_ind][time_ind]);
//      }
      interp_min_costs[problem_ind][run_ind] = interp1<double>(time_sec[problem_ind][run_ind], population_costs[problem_ind][run_ind], interp_time_sec[problem_ind]);
//      interp_min_costs_obj[problem_ind][run_ind] = interp1(obj_eval_counts[problem_ind][run_ind], population_costs[problem_ind][run_ind], interp_obj_count[problem_ind]);
      assert(interp_time_sec[problem_ind].size() == interp_min_costs[problem_ind][run_ind].size());
    });
  });

  /// Plot
  // ind 1: problem, ind 2: time
//  std::vector<std::vector<double>> mean_costs(   problem_names.size());
//  std::vector<std::vector<double>> min_costs(    problem_names.size());
//  std::vector<std::vector<double>> max_costs(    problem_names.size());
//  std::vector<std::vector<double>> mean_time_sec(problem_names.size());
  std::vector<std::vector<double>> mean_costs_interp(   problem_names.size());
  std::vector<std::vector<double>> min_costs_interp(    problem_names.size());
  std::vector<std::vector<double>> max_costs_interp(    problem_names.size());
//  std::vector<std::vector<double>> mean_costs_interp_obj(   problem_names.size());
//  std::vector<std::vector<double>> min_costs_interp_obj(    problem_names.size());
//  std::vector<std::vector<double>> max_costs_interp_obj(    problem_names.size());

  std::for_each(std::execution::par_unseq, problem_inds.begin(), problem_inds.end(),
  [&] (const size_t problem_ind) -> void
  {
    const boost::integer_range<size_t> time_inds(0, interp_time_sec[problem_ind].size());
//    const boost::integer_range<size_t> obj_inds(0, interp_obj_count[problem_ind].size());

    mean_costs_interp[problem_ind].resize(interp_time_sec[problem_ind].size());
    min_costs_interp[ problem_ind].resize(interp_time_sec[problem_ind].size());
    max_costs_interp[ problem_ind].resize(interp_time_sec[problem_ind].size());
//    mean_costs_interp_obj[problem_ind].resize(interp_obj_count[problem_ind].size());
//    min_costs_interp_obj[ problem_ind].resize(interp_obj_count[problem_ind].size());
//    max_costs_interp_obj[ problem_ind].resize(interp_obj_count[problem_ind].size());
    std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
    [&] (const size_t time_ind) -> void
    {
      mean_costs_interp[problem_ind][time_ind] = 0;
      min_costs_interp[ problem_ind][time_ind] = std::numeric_limits<double>::infinity();
      max_costs_interp[ problem_ind][time_ind] = -std::numeric_limits<double>::infinity();

      std::for_each(run_inds.begin(), run_inds.end(),
      [&] (const size_t run_ind) -> void
      {
        mean_costs_interp[problem_ind][time_ind] += interp_min_costs[problem_ind][run_ind][time_ind];
        min_costs_interp[ problem_ind][time_ind] = std::min<double>(min_costs_interp[problem_ind][time_ind], interp_min_costs[problem_ind][run_ind][time_ind]);
        max_costs_interp[ problem_ind][time_ind] = std::max<double>(max_costs_interp[problem_ind][time_ind], interp_min_costs[problem_ind][run_ind][time_ind]);
      });
      mean_costs_interp[problem_ind][time_ind] /= double(NUM_RUNS);
    });
//    std::for_each(std::execution::par_unseq, obj_inds.begin(), obj_inds.end(),
//    [&] (const size_t obj_ind) -> void
//    {
//      mean_costs_interp_obj[problem_ind][obj_ind] = 0;
//      min_costs_interp_obj[ problem_ind][obj_ind] = std::numeric_limits<double>::infinity();
//      max_costs_interp_obj[ problem_ind][obj_ind] = -std::numeric_limits<double>::infinity();
//
//      std::for_each(run_inds.begin(), run_inds.end(),
//      [&] (const size_t run_ind) -> void
//      {
//        mean_costs_interp_obj[problem_ind][obj_ind] += interp_min_costs_obj[problem_ind][run_ind][obj_ind];
//        min_costs_interp_obj[ problem_ind][obj_ind] = std::min<double>(min_costs_interp_obj[problem_ind][obj_ind], interp_min_costs_obj[problem_ind][run_ind][obj_ind]);
//        max_costs_interp_obj[ problem_ind][obj_ind] = std::max<double>(max_costs_interp_obj[problem_ind][obj_ind], interp_min_costs_obj[problem_ind][run_ind][obj_ind]);
//      });
//      mean_costs_interp_obj[problem_ind][obj_ind] /= double(NUM_RUNS);
//    });

//
//    const size_t num_gen = time_sec[problem_ind].unaryExpr([] (const std::vector<double>& gen_info) -> size_t { return gen_info.size(); }).maxCoeff();
//
//    const boost::integer_range<size_t> gen_inds(0, num_gen);
//
//    std::for_each(std::execution::par_unseq, run_inds.begin(), run_inds.end(),
//    [&] (const size_t run_ind) -> void
//    {
//      while(num_gen > time_sec[        problem_ind][run_ind].size()) { time_sec[        problem_ind][run_ind].emplace_back(time_sec[        problem_ind][run_ind].back()); }
//      while(num_gen > population_costs[problem_ind][run_ind].size()) { population_costs[problem_ind][run_ind].emplace_back(population_costs[problem_ind][run_ind].back()); }
//    });
//
//    mean_time_sec[problem_ind].resize(num_gen);
//    mean_costs[   problem_ind].resize(num_gen);
//    min_costs[    problem_ind].resize(num_gen);
//    max_costs[    problem_ind].resize(num_gen);
//    std::for_each(std::execution::par_unseq, gen_inds.begin(), gen_inds.end(),
//    [&] (const size_t gen_ind) -> void
//    {
//      mean_time_sec[problem_ind][gen_ind] = 0;
//      mean_costs[   problem_ind][gen_ind] = 0;
//      min_costs[    problem_ind][gen_ind] = std::numeric_limits<double>::infinity();
//      max_costs[    problem_ind][gen_ind] = -std::numeric_limits<double>::infinity();
//
//      std::for_each(run_inds.begin(), run_inds.end(),
//      [&] (const size_t run_ind) -> void
//      {
//        mean_time_sec[problem_ind][gen_ind] += time_sec[problem_ind][run_ind][gen_ind];
//        mean_costs[   problem_ind][gen_ind] += population_costs[problem_ind][run_ind][gen_ind];
//        min_costs[    problem_ind][gen_ind] = std::min<double>(min_costs[problem_ind][gen_ind], population_costs[problem_ind][run_ind][gen_ind]);
//        max_costs[    problem_ind][gen_ind] = std::max<double>(max_costs[problem_ind][gen_ind], population_costs[problem_ind][run_ind][gen_ind]);
//      });
//      mean_time_sec[problem_ind][gen_ind] /= double(NUM_RUNS);
//      mean_costs[   problem_ind][gen_ind] /= double(NUM_RUNS);
//    });
  });

  // Export to CSV
  const std::string base_dir = "~/firefly_benchmark_results";
  std::filesystem::create_directories(base_dir);
  const std::string base_dir_interp = base_dir + "/interpolated";
  std::filesystem::create_directories(base_dir_interp);
  const std::string base_dir_norm = base_dir + "/generation_averages";
  std::filesystem::create_directories(base_dir_norm);
  for(size_t problem_ind = 0; problem_ind < problem_names.size(); ++problem_ind)
  {
//    {
//      std::ofstream file(base_dir_norm + "/" + problem_names[problem_ind] + ".csv");
//      file << "mean_time,mean_cost,max_cost,min_cost";
//      size_t last_ind_saved     = 0;
//      for(Eigen::Index gen_it = 0; gen_it < mean_time_sec[problem_ind].size(); ++gen_it)
//      {
//        if((gen_it != 0) and
//           (gen_it != (mean_time_sec[problem_ind].size() - 1)))
//        {
//           if(std::fabs(mean_time_sec[problem_ind][last_ind_saved] - mean_time_sec[problem_ind][gen_it]) < plot_dt) { continue; }
//           if((std::fabs(mean_costs[problem_ind][last_ind_saved] - mean_costs[problem_ind][gen_it + 1]) < 1.0e-12) and
//              (std::fabs(mean_costs[problem_ind][last_ind_saved] - mean_costs[problem_ind][gen_it])     < 1.0e-12) and
//              (std::fabs(max_costs[ problem_ind][last_ind_saved] - max_costs[ problem_ind][gen_it + 1]) < 1.0e-12) and
//              (std::fabs(max_costs[ problem_ind][last_ind_saved] - max_costs[ problem_ind][gen_it])     < 1.0e-12) and
//              (std::fabs(min_costs[ problem_ind][last_ind_saved] - min_costs[ problem_ind][gen_it + 1]) < 1.0e-12) and
//              (std::fabs(min_costs[ problem_ind][last_ind_saved] - min_costs[ problem_ind][gen_it])     < 1.0e-12)) { continue; }
//        }
//        last_ind_saved = gen_it;
//
//        file << "\n"
//             << mean_time_sec[problem_ind][gen_it] << ","
//             << mean_costs[   problem_ind][gen_it] << ","
//             << max_costs[    problem_ind][gen_it] << ","
//             << min_costs[    problem_ind][gen_it];
//      }
//      file.flush();
//      file.close();
//    }
    {
      std::ofstream file(base_dir_interp + "/" + problem_names[problem_ind] + ".csv");
      file << "mean_time,mean_cost,max_cost,min_cost";
      double last_cost_saved     = std::numeric_limits<double>::infinity();
      double last_max_cost_saved = std::numeric_limits<double>::infinity();
      double last_min_cost_saved = std::numeric_limits<double>::infinity();
      for(Eigen::Index gen_it = 0; gen_it < interp_time_sec[problem_ind].size(); ++gen_it)
      {
        if((gen_it != 0) and
           (gen_it != (interp_time_sec[problem_ind].size() - 1)) and
           (std::fabs(last_cost_saved -     mean_costs_interp[problem_ind][gen_it + 1]) < 1.0e-12) and
           (std::fabs(last_cost_saved -     mean_costs_interp[problem_ind][gen_it])     < 1.0e-12) and
           (std::fabs(last_max_cost_saved - max_costs_interp[ problem_ind][gen_it + 1]) < 1.0e-12) and
           (std::fabs(last_max_cost_saved - max_costs_interp[ problem_ind][gen_it])     < 1.0e-12) and
           (std::fabs(last_min_cost_saved - min_costs_interp[ problem_ind][gen_it + 1]) < 1.0e-12) and
           (std::fabs(last_min_cost_saved - min_costs_interp[ problem_ind][gen_it])     < 1.0e-12)) { continue; }
        last_cost_saved     = mean_costs_interp[problem_ind][gen_it];
        last_min_cost_saved = min_costs_interp[ problem_ind][gen_it];
        last_max_cost_saved = max_costs_interp[ problem_ind][gen_it];

        file << "\n"
             << interp_time_sec[problem_ind][gen_it] << ","
             << mean_costs_interp[   problem_ind][gen_it] << ","
             << max_costs_interp[    problem_ind][gen_it] << ","
             << min_costs_interp[    problem_ind][gen_it];
//             << interp_obj_count[problem_ind][gen_it] << ","
//             << mean_costs_interp_obj[   problem_ind][gen_it] << ","
//             << max_costs_interp_obj[    problem_ind][gen_it] << ","
//             << min_costs_interp_obj[    problem_ind][gen_it];
      }
      file.flush();
      file.close();
    }
  }

  exit(EXIT_SUCCESS);

//  for(size_t problem_ind = 0; problem_ind < problem_names.size(); ++problem_ind)
//  {
//    matplotlibcpp::figure();
//    matplotlibcpp::named_plot<double,double>("Min",  interp_time_sec[problem_ind], min_costs_interp[problem_ind],  "m");
//    matplotlibcpp::named_plot<double,double>("Max",  interp_time_sec[problem_ind], max_costs_interp[problem_ind],  "g");
//    matplotlibcpp::named_plot<double,double>("Mean", interp_time_sec[problem_ind], mean_costs_interp[problem_ind], "b");
//
//    matplotlibcpp::named_plot<double,double>("Min",  mean_time_sec[problem_ind], min_costs[problem_ind],  "m--");
//    matplotlibcpp::named_plot<double,double>("Max",  mean_time_sec[problem_ind], max_costs[problem_ind],  "g--");
//    matplotlibcpp::named_plot<double,double>("Mean", mean_time_sec[problem_ind], mean_costs[problem_ind], "b--");
//    for(Eigen::Index run_it = 0; run_it < NUM_RUNS; ++run_it)
//    {
//      matplotlibcpp::plot(interp_time_sec[problem_ind], interp_min_costs[problem_ind][run_it], "y");
//      matplotlibcpp::plot(time_sec[problem_ind][run_it], population_costs[problem_ind][run_it], "r.");
//    }
//    matplotlibcpp::xlabel("Time (sec)");
//    matplotlibcpp::ylabel("Objective Function Value");
//    matplotlibcpp::title(problem_names[problem_ind]);
//    matplotlibcpp::legend();
//  }
//
//  matplotlibcpp::show();
//  exit(EXIT_SUCCESS);
}

/* firefly_benchmark.cpp */
