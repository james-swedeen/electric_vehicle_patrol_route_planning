#!/usr/bin/env python3
##
# @File: firefly_tuner.py
#

from plan_plotter_msgs.srv import TryFireflyAlgorithmConfig
import numpy as np
import rclpy
from rclpy.node import Node
from ray import tune, train
from ray.train import RunConfig
from ray.tune.search.hyperopt import HyperOptSearch
from ray.tune.search import Repeater
import os
import multiprocessing


class PlannerClient(Node):
    def __init__(self):
        super().__init__('firefly_planner_tuner')
        self.client = self.create_client(TryFireflyAlgorithmConfig, 'firefly_config_test')
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')


def objective(config):
    #if (config['use_selection'] == 3) and (config['number_clusters'] > config['population_size']):
    #    train.report({"cost_mean": np.inf,
    #                  "cost_std":  np.inf,
    #                  "cost_max":  np.inf,
    #                  "cost_min":  np.inf,
    #                  "time_mean": np.inf,
    #                  "time_std":  np.inf,
    #                  "time_max":  np.inf,
    #                  "time_min":  np.inf,
    #                  "metric":    np.inf})
    #    return

    rclpy.init()
    planner_client = PlannerClient()
    req = TryFireflyAlgorithmConfig.Request()

    req.number_runs                        = 4;
    req.population_size                    = config['population_size']
    req.distance_power                     = config['distance_power']
    req.distance_multiplier                = config['distance_multiplier']
    req.base_attractiveness_multiplier     = config['base_attractiveness_multiplier']
    req.min_attractiveness_multiplier      = config['min_attractiveness_multiplier']
    req.cost_scaling_mult                  = config['cost_scaling_mult']
    req.use_firefly_operator               = config['use_firefly_operator']
    req.use_original_luminosity_multiplier = True # config['use_original_luminosity_multiplier']
    req.force_no_duplicates                = False # config['force_no_duplicates']
    req.use_selection                      = config['use_selection']
    req.use_elitism_selection              = True # config['use_elitism_selection']
    req.mutation_magnitude                 = config['mutation_magnitude']
    req.number_clusters                    = 0 # config['number_clusters']
    req.number_cluster_iterations          = 0 # config['number_cluster_iterations']
    req.pso_phi_particles                  = config['pso_phi_particles']
    req.pso_phi_global                     = config['pso_phi_global']
    req.problem_ind                        = config['problem_ind']
    req.soft_max_mult                      = config['soft_max_mult']
    req.from_cost_scaling_mult             = config['from_cost_scaling_mult']

    #if (config['use_selection'] == 3) and (config['number_clusters'] > config['population_size']):
    #    req.number_clusters = req.population_size

    # Run planner
    future = planner_client.client.call_async(req)
    rclpy.spin_until_future_complete(planner_client, future)
    response = future.result()

    #print(response)

    if all(i <= 1.0e-6 for i in response.costs):
        metric = -float(1) / (np.mean(response.times) + (3.0 * np.std(response.times)))
    else:
        metric = np.mean(response.costs) + (3.0 * np.std(response.costs))

    # Send the score to Tune
    train.report({"cost_mean": np.mean(response.costs),
                  "cost_std":  np.std( response.costs),
                  "cost_max":  np.max( response.costs),
                  "cost_min":  np.min( response.costs),
                  "time_mean": np.mean(response.times),
                  "time_std":  np.std( response.times),
                  "time_max":  np.max( response.times),
                  "time_min":  np.min( response.times),
                  "metric":    metric})
    planner_client.destroy_node()
    rclpy.shutdown()

def temp_func(alg, prob_ind):
    os.nice(19)
    pnt_to_eval=[
    ]
    if alg == 0:
        param_space={
            'base_attractiveness_multiplier':     0.0, #tune.uniform(1.0e-8, 10.0),
            'cost_scaling_mult':                  0.0, #tune.uniform(1.0e-8, 100.0),
            'distance_multiplier':                0.0, #tune.uniform(1.0e-8, 150.0),
            'distance_power':                     2.0, # tune.uniform(1.0e-8, 10.0),
            'min_attractiveness_multiplier':      0.0, #tune.uniform(1.0e-8, 5.0),
            #'force_no_duplicates':                False,
            'mutation_magnitude':                 tune.uniform(1.0e-8, 25.0),
            #'number_cluster_iterations':          0, #tune.randint(0, 6),
            #'number_clusters':                    0, #tune.randint(2, 50),
            'population_size':                    tune.randint(5, 151),
            #'use_elitism_selection':              True,
            'use_firefly_operator':               0,
            #'use_original_luminosity_multiplier': True,
            'use_selection':                      0,
            'pso_phi_particles':                  0.0, #tune.uniform(1.0e-8, 10.0),
            'pso_phi_global':                     0.0, #tune.uniform(1.0e-8, 10.0),
            'problem_ind':                        prob_ind,
        }
        num_run = 100
    if alg == 3:
        param_space={
            'base_attractiveness_multiplier':     tune.uniform(1.0e-8, 10.0),
            'cost_scaling_mult':                  0.0, #tune.uniform(1.0e-8, 100.0),
            'distance_multiplier':                tune.uniform(1.0e-8, 150.0),
            'distance_power':                     2.0, # tune.uniform(1.0e-8, 10.0),
            'min_attractiveness_multiplier':      0.0, #tune.uniform(1.0e-8, 5.0),
            #'force_no_duplicates':                False,
            'mutation_magnitude':                 tune.uniform(1.0e-8, 25.0),
            #'number_cluster_iterations':          0, #tune.randint(0, 6),
            #'number_clusters':                    0, #tune.randint(2, 50),
            'population_size':                    tune.randint(5, 151),
            #'use_elitism_selection':              True,
            'use_firefly_operator':               3,
            #'use_original_luminosity_multiplier': True,
            'use_selection':                      0,
            'pso_phi_particles':                  0.0, #tune.uniform(1.0e-8, 10.0),
            'pso_phi_global':                     0.0, #tune.uniform(1.0e-8, 10.0),
            'problem_ind':                        prob_ind,
        }
        num_run = 200
    if alg == 4:
        param_space={
            'base_attractiveness_multiplier':     tune.uniform(1.0e-8, 150.0),
            'cost_scaling_mult':                  tune.uniform(0.0, 800.0),
            'from_cost_scaling_mult':             tune.uniform(0.0, 1.0),
            'distance_multiplier':                tune.uniform(1.0e-8, 150.0),
            'distance_power':                     2.0, # tune.uniform(1.0e-8, 10.0),
            'min_attractiveness_multiplier':      0.0, # tune.uniform(0.0, 25.0),
            'soft_max_mult':                      0.0, #tune.uniform(1.0e-8, 25.0),
            #'force_no_duplicates':                False,
            'mutation_magnitude':                 tune.uniform(0.0, 25.0),
            #'number_cluster_iterations':          0, #tune.randint(0, 6),
            #'number_clusters':                    0, #tune.randint(2, 50),
            'population_size':                    tune.randint(3, 151),
            #'use_elitism_selection':              True,
            'use_firefly_operator':               4,
            #'use_original_luminosity_multiplier': True,
            'use_selection':                      0,
            'pso_phi_particles':                  0.0, #tune.uniform(1.0e-8, 10.0),
            'pso_phi_global':                     0.0, #tune.uniform(1.0e-8, 10.0),
            'problem_ind':                        prob_ind,
        }
        num_run = 300
    if alg == 5:
        param_space={
            'base_attractiveness_multiplier':     0.0, #tune.uniform(1.0e-8, 10.0),
            'cost_scaling_mult':                  0.0, #tune.uniform(1.0e-8, 100.0),
            'distance_multiplier':                0.0, #tune.uniform(1.0e-8, 150.0),
            'distance_power':                     2.0, # tune.uniform(1.0e-8, 10.0),
            'min_attractiveness_multiplier':      0.0, #tune.uniform(1.0e-8, 5.0),
            #'force_no_duplicates':                False,
            'mutation_magnitude':                 0.0, #tune.uniform(1.0e-8, 1.0),
            #'number_cluster_iterations':          0, #tune.randint(0, 6),
            #'number_clusters':                    0, #tune.randint(2, 50),
            'population_size':                    tune.randint(5, 151),
            #'use_elitism_selection':              True,
            'use_firefly_operator':               5,
            #'use_original_luminosity_multiplier': True,
            'use_selection':                      0,
            'pso_phi_particles':                  tune.uniform(1.0e-8, 10.0),
            'pso_phi_global':                     tune.uniform(1.0e-8, 10.0),
            'problem_ind':                        prob_ind,
        }
        num_run = 150
    # gamma bigger means more focused best samples so far
    search = HyperOptSearch(gamma=0.2, n_initial_points=10, points_to_evaluate=pnt_to_eval)
    tuner = tune.Tuner(
        objective,
        tune_config=tune.TuneConfig(
            search_alg=search,
            metric="metric",
            mode="min",
            num_samples=num_run, # 0, 3, 4, 5
            max_concurrent_trials=1,
            time_budget_s=60*60*1000000,
        ),
        param_space=param_space,
        run_config=RunConfig(storage_path="~/ray_results", name="alg" + str(alg) + "_prob" + str(prob_ind))
    )
    results = tuner.fit()



if __name__ == '__main__':
    os.environ["RAY_memory_monitor_refresh_ms"] = "0"
    for prob_ind in [1, 2, 3, 4, 5, 0]:
        for alg in [4]:
            p = multiprocessing.Process(target=temp_func, args=(alg, prob_ind,))
            p.start()
            p.join()

