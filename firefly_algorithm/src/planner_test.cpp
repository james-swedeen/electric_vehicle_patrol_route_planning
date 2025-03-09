/**
 * @File: planner_test.cpp
 * @Date: August 2024
 * @Author: James Swedeen
 *
 * @brief
 * Simple demo for testing planners with plotting.
 **/

/* C++ Headers */
#include<random>
#include<deque>
#include<atomic>

/* Plotting Headers */
#include<matplotlibcpp/matplotlibcpp.hpp>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<firefly_algorithm/firefly_algorithm.hpp>


std::vector<double> toVec(const Eigen::Ref<const Eigen::Matrix<double,Eigen::Dynamic,1>>& input)
{
  std::vector<double> output(input.rows());

  for(Eigen::Index col_it = 0; col_it < input.rows(); ++col_it)
  {
    output[col_it] = input[col_it];
  }

  return output;
}


int main(int argc, char** argv)
{
//  rclcpp::init(argc, argv);
//  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("planner_test");

  const size_t num_rand_runs = 100;

  std::vector<double>                                fa_planner_time_sec;
  std::deque<Eigen::Matrix<double,Eigen::Dynamic,1>> fa_planner_cost;
  fa_planner_time_sec.reserve(999999);
  std::cout << "Planning with Firefly" << std::endl;

  //static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_MUTATIONS bitor fa::FaFlags::USE_K_MEANS_SELECTION bitor fa::FaFlags::USE_ELITISM_SELECTION);
  //static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor fa::FaFlags::USE_ELITISM_SELECTION);
  static constexpr const fa::FaFlags FA_CONFIG = fa::FaFlags(fa::FaFlags::USE_VEC_FIREFLY_OPERATOR bitor fa::FaFlags::USE_DEFAULT_LUMINOSITY_MULTIPLAYER bitor fa::FaFlags::USE_BIDIRECTIONAL_FIREFLY_OPERATOR);
  static constexpr const Eigen::Index DIM = 2;

  fa::FaParamsPtr<DIM> fa_params = std::make_shared<fa::FaParams<DIM>>();
  fa_params->population_size           = 10;
  fa_params->dist_pow                  = 2;
  fa_params->dist_mult                 = 1;
  fa_params->base_attractiveness_mult  = 1;
  fa_params->min_attractiveness_mult   = 0.1;
  fa_params->cost_scaling_mult         = 1;
  fa_params->mutation_magnitude        = Eigen::Matrix<double,DIM,1>::Constant(0.2);
  fa_params->number_clusters           = 100;
  fa_params->number_cluster_iterations = 2;
  fa_params->from_cost_scaling_mult    = 0.1;
  fa_params->soft_max_mult             = 2;

  std::atomic_uint64_t obj_eval_count = 0;
  const auto objective_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
    {
      if((x.array().abs() > M_PI).any()) { return std::pair<bool,double>(false, std::numeric_limits<double>::infinity()); }
      ++obj_eval_count;
      // Michalewicz function
      static constexpr const double m = 10;
      double output = 0;
      for(Eigen::Index dim_ind = 0; dim_ind < DIM; ++dim_ind)
      {
        output += (std::sin(x[dim_ind]) * std::pow(std::sin(((dim_ind+1)*std::pow(x[dim_ind], 2.0)) / double(M_PI)), 2.0 * m));
      }
      return std::pair<bool,double>(true, -output);
    };
  const auto rand_init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
    {
      std::default_random_engine             rand_gen(rand_seed);
      std::uniform_real_distribution<double> probability_dist(0, M_PI);
      return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
    };

//  const auto objective_func = [&] (const Eigen::Ref<const Eigen::Matrix<double,DIM,1>>& x) -> std::pair<bool,double>
//    {
//      ++obj_eval_count;
//      // Rastrigin function
//      static constexpr const double A = 10;
//      return std::pair<bool,double>(true, (A * DIM) + (x.array().square() - (A * (x.array() * double(2) * double(M_PI)).cos())).sum());
//    };
//  const auto rand_init_func = [&] (const unsigned rand_seed) -> Eigen::Matrix<double,DIM,1>
//    {
//      std::default_random_engine             rand_gen(rand_seed);
//      std::uniform_real_distribution<double> probability_dist(-5.12, 5.12);
//      return Eigen::Matrix<double,DIM,1>::NullaryExpr([&] () { return probability_dist(rand_gen); });
//    };

  const auto plan_start_time = std::chrono::high_resolution_clock::now();
  auto       last_start_time = std::chrono::high_resolution_clock::now();
  const Eigen::Matrix<double,DIM,1> plan = fa::generateFireflyPlan<DIM,FA_CONFIG>(fa_params,
                                              rand_init_func,
                                              objective_func,
                                              [&] (const size_t generation_count, const double best_cost_found) -> bool
                                              {
                                                //std::cout << generation_count << std::endl;
                                                return (best_cost_found <= (-1.801 + 1.0e-6)) or (generation_count >= 10000);
                                                //return generation_count >= 1000;
                                              },
                                              [&] (const Eigen::Matrix<std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>,Eigen::Dynamic,1>& cur_population) -> void
                                              {
                                                auto log_start_time = std::chrono::high_resolution_clock::now();

                                                if(fa_planner_time_sec.empty())
                                                {
                                                  fa_planner_time_sec.emplace_back(double(std::chrono::duration_cast<std::chrono::nanoseconds>(log_start_time - last_start_time).count()) * double(1.0e-9));
                                                }
                                                else
                                                {
                                                  fa_planner_time_sec.emplace_back((double(std::chrono::duration_cast<std::chrono::nanoseconds>(log_start_time - last_start_time).count()) * double(1.0e-9)) + fa_planner_time_sec.back());
                                                }
                                                fa_planner_cost.emplace_back(cur_population.unaryExpr([] (const std::unique_ptr<std::pair<Eigen::Matrix<double,DIM,1>,double>>& i) -> double { return i->second; }));

                                                last_start_time = std::chrono::high_resolution_clock::now();
                                              },
                                              42);
  const auto plan_end_time = std::chrono::high_resolution_clock::now();
  std::cout << "Took " << double(std::chrono::duration_cast<std::chrono::nanoseconds>(plan_end_time - plan_start_time).count()) * double(1.0e-9) << " seconds" << std::endl;
  std::cout << "With " << obj_eval_count << " objective function evaluations" << std::endl;

  std::cout << "Running Objective Function" << std::endl;
  const auto obj_start_time = std::chrono::high_resolution_clock::now();
  const std::pair<bool,double> obj_func_result = objective_func(plan);
  const auto obj_end_time = std::chrono::high_resolution_clock::now();
  std::cout << "Objective function value: " << obj_func_result.second << std::endl;
  std::cout << "Took " << std::chrono::duration_cast<std::chrono::milliseconds>(obj_end_time - obj_start_time).count() << " milliseconds" << std::endl;

  Eigen::Matrix<double,Eigen::Dynamic,1> random_planner_times_sec(num_rand_runs, 1);
  Eigen::Matrix<double,Eigen::Dynamic,1> random_planner_costs(    num_rand_runs, 1);
  std::cout << "Random Planning" << std::endl;
  for(size_t rand_it = 0; rand_it < num_rand_runs; ++rand_it)
  {
    const auto plan_start_time = std::chrono::high_resolution_clock::now();
    const Eigen::Matrix<double,DIM,1> plan = rand_init_func(42 + rand_it);
    const auto plan_end_time = std::chrono::high_resolution_clock::now();
    random_planner_times_sec[rand_it] = double(std::chrono::duration_cast<std::chrono::nanoseconds>(plan_end_time - plan_start_time).count()) * double(1.0e-9);
    const std::pair<bool,double> obj_func_result = objective_func(plan);
    random_planner_costs[rand_it] = obj_func_result.second;
  }

  /// Plot results
  matplotlibcpp::figure();
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
  {
    const size_t num_gen         = fa_planner_cost.size();
    const size_t population_size = fa_planner_cost[0].rows();

    std::vector<double> mean_cost(num_gen);
    std::vector<double> min_cost( num_gen);
    std::vector<double> max_cost( num_gen);
    for(size_t gen_ind = 0; gen_ind < num_gen; ++gen_ind)
    {
      mean_cost[gen_ind] = fa_planner_cost[gen_ind].mean();
      min_cost[ gen_ind] = fa_planner_cost[gen_ind].minCoeff();
      max_cost[ gen_ind] = fa_planner_cost[gen_ind].maxCoeff();
    }

    matplotlibcpp::named_plot<double>("Firefly Planner Min", fa_planner_time_sec, min_cost, "m*");
    matplotlibcpp::named_plot<double>("Firefly Planner Max", fa_planner_time_sec, max_cost, "g*");
    matplotlibcpp::named_plot<double>("Firefly Planner Mean", fa_planner_time_sec, mean_cost, "b*");
    matplotlibcpp::named_plot<double>("Optimal", fa_planner_time_sec, std::vector<double>(num_gen, -1.8010), "r");
  }
  matplotlibcpp::xlabel("Solve Time (sec)");
  matplotlibcpp::ylabel("Objective Function Value");
  matplotlibcpp::title("Planner Results");
  matplotlibcpp::legend();
  matplotlibcpp::show();

//  rclcpp::spin(node);
//  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}

/* planner_test.cpp */
