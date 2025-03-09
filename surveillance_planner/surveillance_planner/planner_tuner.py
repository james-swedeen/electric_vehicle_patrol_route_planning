#!/usr/bin/env python3
##
# @File: planner_tuner.py
#

import os
os.environ["RAY_worker_niceness"] = str(10)
os.environ["RAY_memory_monitor_refresh_ms"] = "0"

from graph_surveillance_planning_msgs.srv import TryFireflyConfig
import numpy as np
import time
import rclpy
from rclpy.node import Node
import ray
from ray import tune, train
from ray.train import RunConfig
from ray.tune.search.ax import AxSearch
import multiprocessing
import random
import gc

max_runs           = 3
num_samples        = 300
trial_max_time_sec = 1.0 * 60.0

class PlannerClient(Node):
    def __init__(self):
        super().__init__('firefly_planner_tuner')
        self.client = self.create_client(TryFireflyConfig, 'firefly_config_test')
        while not self.client.wait_for_service(timeout_sec=15.0):
            self.get_logger().info('service not available, waiting again...')


def objective(config):
    gc.collect()
    rclpy.init()
    planner_client = PlannerClient()
    req = TryFireflyConfig.Request()

    req.max_solve_time_sec = trial_max_time_sec

    req.use_greedy_in_init                         = True # config['use_greedy_in_init']
    req.use_field_accent_in_init                   = True # config['use_field_accent_in_init']
    req.use_local_search                           = True # config['use_local_search']
    req.local_search_max_time_sec                  = config['local_search_max_time_sec']
    req.field_accent_width                         = config['field_accent_width']
    req.field_accent_search_weight                 = config['field_accent_search_weight']
    req.field_accent_avoid_weight                  = config['field_accent_avoid_weight']
    req.population_size                            = config['population_size']
    req.max_dwell_time_hotspot                     = config['max_dwell_time_hotspot']
    req.max_dwell_time_depot                       = config['max_dwell_time_depot']
    req.distance_power                             = config['distance_power']
    req.distance_multiplier                        = config['distance_multiplier']
    req.base_attractiveness_multiplier             = config['base_attractiveness_multiplier']
    req.tweak_dwell_time_max_change                = config['tweak_dwell_time_max_change']
    req.initialization_max_dwell_time              = config['initialization_max_dwell_time']
    req.local_search_probability                   = config['local_search_probability']
    req.initial_temperature                        = config['initial_temperature']
    req.min_temperature                            = config['min_temperature']
    req.cooling_rate                               = config['cooling_rate']
    req.group_number                               = config['group_number']
    req.number_elite                               = config['number_elite']
    req.sa_max_time_sec                            = config['sa_max_time_sec']
    req.accept_cost_threshold                      = config['accept_cost_threshold']
    req.use_firefly_operator                       = config['use_firefly_operator']
    req.use_original_luminosity_multiplier         = False # config['use_original_luminosity_multiplier']
    req.force_no_duplicates                        = False # config['force_no_duplicates']
    req.use_selection                              = config['use_selection']
    req.use_elitism_selection                      = True # config['use_elitism_selection']
    req.number_clusters                            = config['number_clusters']
    req.number_cluster_iterations                  = config['number_cluster_iterations']
    #req.mop_add_visit                              = config['mop_add_visit']
    req.mop_add_hotspot_visit                      = config['mop_add_hotspot_visit']
    req.mop_add_depot_visit                        = config['mop_add_depot_visit']
    #req.mop_remove_visit                           = config['mop_remove_visit']
    req.mop_remove_hotspot_visit                   = config['mop_remove_hotspot_visit']
    req.mop_remove_depot_visit                     = config['mop_remove_depot_visit']
    req.mop_remove_visits_windowed                 = config['mop_remove_visits_windowed']
    #req.mop_change_dwell_time                      = config['mop_change_dwell_time']
    req.mop_tweak_dwell_time                       = config['mop_tweak_dwell_time']
    req.mop_swap_path                              = config['mop_swap_path']
    req.mop_swap_visits                            = config['mop_swap_visits']
    req.mop_swap_multiple_visits                   = config['mop_swap_multiple_visits']
    #req.mop_move_visit                             = config['mop_move_visit']
    req.mop_sim                                    = config['mop_sim']
    #req.mop_add_visit_min_time_path                = config['mop_add_visit_min_time_path']
    #req.mop_add_hotspot_visit_min_time_path        = config['mop_add_hotspot_visit_min_time_path']
    #req.mop_add_depot_visit_min_time_path          = config['mop_add_depot_visit_min_time_path']
    #req.mop_remove_visit_min_time_path             = config['mop_remove_visit_min_time_path']
    #req.mop_remove_hotspot_visit_min_time_path     = config['mop_remove_hotspot_visit_min_time_path']
    #req.mop_remove_depot_visit_min_time_path       = config['mop_remove_depot_visit_min_time_path']
    #req.mop_remove_visits_windowed_min_time_path   = config['mop_remove_visits_windowed_min_time_path']
    #req.mop_swap_path_min_time_path                = config['mop_swap_path_min_time_path']
    #req.mop_swap_visits_min_time_path              = config['mop_swap_visits_min_time_path']
    #req.mop_swap_multiple_visits_min_time_path     = config['mop_swap_multiple_visits_min_time_path']
    #req.mop_move_visit_min_time_path               = config['mop_move_visit_min_time_path']
    #req.mop_sim_min_time_path                      = config['mop_sim_min_time_path']
    #req.mop_add_visit_min_charge_path              = config['mop_add_visit_min_charge_path']
    #req.mop_add_hotspot_visit_min_charge_path      = config['mop_add_hotspot_visit_min_charge_path']
    #req.mop_add_depot_visit_min_charge_path        = config['mop_add_depot_visit_min_charge_path']
    #req.mop_remove_visit_min_charge_path           = config['mop_remove_visit_min_charge_path']
    #req.mop_remove_hotspot_visit_min_charge_path   = config['mop_remove_hotspot_visit_min_charge_path']
    #req.mop_remove_depot_visit_min_charge_path     = config['mop_remove_depot_visit_min_charge_path']
    #req.mop_remove_visits_windowed_min_charge_path = config['mop_remove_visits_windowed_min_charge_path']
    #req.mop_swap_path_min_charge_path              = config['mop_swap_path_min_charge_path']
    #req.mop_swap_visits_min_charge_path            = config['mop_swap_visits_min_charge_path']
    #req.mop_swap_multiple_visits_min_charge_path   = config['mop_swap_multiple_visits_min_charge_path']
    #req.mop_move_visit_min_charge_path             = config['mop_move_visit_min_charge_path']
    #req.mop_sim_min_charge_path                    = config['mop_sim_min_charge_path']

    # Run planner
    costs     = []
    gen_ran   = []
    end_times = []
    for _ in range(max_runs):
        future = planner_client.client.call_async(req)
        rclpy.spin_until_future_complete(node=planner_client, future=future)#, timeout_sec=60*60*10 + random.uniform(0, 180))
        #while not future.done():
        #    print("Warning: ROS service timed out!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        #    future = planner_client.client.call_async(req)
        #    rclpy.spin_until_future_complete(node=planner_client, future=future, timeout_sec=60*60*12 + random.uniform(0, 180))
        response = future.result()
        costs.append(np.interp(x=[req.max_solve_time_sec], xp=response.time_gen_sec, fp=response.cost_gen))
        gen_ran.append(np.interp(x=[req.max_solve_time_sec], xp=response.time_gen_sec, fp=[i for i in range(len(response.cost_gen))]))
        end_times.append(response.time_gen_sec[-1])
    # Send the score to Tune
    #train.report({"mean":              float(np.mean(costs)),
    #              "std":               float(np.std( costs)),
    #              "max":               float(np.max( costs)),
    #              "min":               float(np.min( costs)),
    #              "number_of_samples": int(  len(    costs)),
    #              "metric":            float(np.mean(costs)),
    #              "mean_gen_ran":      float(np.mean(gen_ran)),
    #              "std_gen_ran":       float(np.std( gen_ran)),
    #              "max_gen_ran":       float(np.max( gen_ran)),
    #              "min_gen_ran":       float(np.min( gen_ran)),
    #              })

    planner_client.destroy_node()
    rclpy.shutdown()
    gc.collect()
    return {"mean":              float(np.mean(costs)),
            "std":               float(np.std( costs)),
            "max":               float(np.max( costs)),
            "min":               float(np.min( costs)),
            "number_of_samples": int(  len(    costs)),
            "metric":            float(np.sum( costs)),
            "mean_gen_ran":      float(np.mean(gen_ran)),
            "std_gen_ran":       float(np.std( gen_ran)),
            "max_gen_ran":       float(np.max( gen_ran)),
            "min_gen_ran":       float(np.min( gen_ran)),
            "std_run_time_sec":  float(np.std( end_times)),
            "max_run_time_sec":  float(np.max( end_times)),
            "min_run_time_sec":  float(np.min( end_times)),
            }

def run_func(alg_ind, selection_ind, only_one_tuner_running):
    ray.init(num_cpus=1)
    pnt_to_eval=[
    ]
    param_space={
        #'use_greedy_in_init':                         tune.choice([True, False]),
        #'use_field_accent_in_init':                   tune.choice([True, False]),
        #'use_local_search':                           tune.choice([True, False]),
        'local_search_max_time_sec':                  tune.uniform(0, trial_max_time_sec),
        'field_accent_width':                         tune.uniform(1.0e-12, 160000),
        'field_accent_search_weight':                 tune.uniform(0, 500),
        'field_accent_avoid_weight':                  tune.uniform(300, 10000),
        'population_size':                            tune.randint(3, 51),
        'max_dwell_time_depot':                       tune.randint(80, 500),
        'max_dwell_time_hotspot':                     tune.randint(5, 400),
        'distance_power':                             tune.uniform(1.0e-6, 15),
        'distance_multiplier':                        tune.uniform(0, 100),
        'base_attractiveness_multiplier':             tune.uniform(0, 100),
        'tweak_dwell_time_max_change':                tune.randint(5, 500),
        'initialization_max_dwell_time':              tune.randint(3, 20),
        'local_search_probability':                   tune.uniform(0, 1),
        'initial_temperature':                        tune.uniform(0, 1500),
        'min_temperature':                            tune.uniform(0, 100),
        'cooling_rate':                               tune.uniform(0, 1),
        'group_number':                               tune.randint(1, 100),
        'sa_max_time_sec':                            tune.uniform(0, trial_max_time_sec),
        'number_elite':                               tune.randint(0, 51),
        'accept_cost_threshold':                      tune.uniform(0.5, 5),
        'use_firefly_operator':                       tune.choice([0, 1, 2, 3, 4, 5, 6, 7]),
        # 'use_original_luminosity_multiplier':         tune.choice([True, False]),
        # 'force_no_duplicates':                        tune.choice([True, False]),
        'use_selection':                              tune.choice([0, 1, 2, 3]),
        #'use_elitism_selection':                      tune.choice([True, False]),
        'number_clusters':                            tune.randint(1, 45),
        'number_cluster_iterations':                  tune.randint(0, 100),
        'mop_add_depot_visit':                        tune.uniform(0, 1),
        'mop_add_hotspot_visit':                      tune.uniform(0, 1),
        # 'mop_add_visit':                              tune.uniform(0, 1),
        # 'mop_change_dwell_time':                      tune.uniform(0, 1),
        # 'mop_move_visit':                             tune.uniform(0, 1),
        'mop_remove_depot_visit':                     tune.uniform(0, 1),
        'mop_remove_hotspot_visit':                   tune.uniform(0, 1),
        # 'mop_remove_visit':                           tune.uniform(0, 1),
        'mop_remove_visits_windowed':                 tune.uniform(0, 1),
        'mop_sim':                                    tune.uniform(0, 1),
        'mop_swap_multiple_visits':                   tune.uniform(0, 1),
        'mop_swap_path':                              tune.uniform(0, 1),
        'mop_swap_visits':                            tune.uniform(0, 1),
        'mop_tweak_dwell_time':                       tune.uniform(0, 1),
        # 'mop_add_depot_visit_min_time_path':          tune.uniform(0, 1),
        # 'mop_add_hotspot_visit_min_time_path':        tune.uniform(0, 1),
        # 'mop_add_visit_min_time_path':                tune.uniform(0, 1),
        # 'mop_move_visit_min_time_path':               tune.uniform(0, 1),
        # 'mop_remove_depot_visit_min_time_path':       tune.uniform(0, 1),
        # 'mop_remove_hotspot_visit_min_time_path':     tune.uniform(0, 1),
        # 'mop_remove_visit_min_time_path':             tune.uniform(0, 1),
        # 'mop_remove_visits_windowed_min_time_path':   tune.uniform(0, 1),
        # 'mop_sim_min_time_path':                      tune.uniform(0, 1),
        # 'mop_swap_multiple_visits_min_time_path':     tune.uniform(0, 1),
        # 'mop_swap_path_min_time_path':                tune.uniform(0, 1),
        # 'mop_swap_visits_min_time_path':              tune.uniform(0, 1),
        # 'mop_add_depot_visit_min_charge_path':        tune.uniform(0, 1),
        # 'mop_add_hotspot_visit_min_charge_path':      tune.uniform(0, 1),
        # 'mop_add_visit_min_charge_path':              tune.uniform(0, 1),
        # 'mop_move_visit_min_charge_path':             tune.uniform(0, 1),
        # 'mop_remove_depot_visit_min_charge_path':     tune.uniform(0, 1),
        # 'mop_remove_hotspot_visit_min_charge_path':   tune.uniform(0, 1),
        # 'mop_remove_visit_min_charge_path':           tune.uniform(0, 1),
        # 'mop_remove_visits_windowed_min_charge_path': tune.uniform(0, 1),
        # 'mop_sim_min_charge_path':                    tune.uniform(0, 1),
        # 'mop_swap_multiple_visits_min_charge_path':   tune.uniform(0, 1),
        # 'mop_swap_path_min_charge_path':              tune.uniform(0, 1),
        # 'mop_swap_visits_min_charge_path':            tune.uniform(0, 1),
    }
    parameter_constraints=[]
    alg_name = ''
    if alg_ind == 0:
        alg_name                                          = 'rands'
        param_space["distance_power"]                     = 0.0
        param_space["distance_multiplier"]                = 0.0
        param_space["base_attractiveness_multiplier"]     = 0.0
        param_space['initial_temperature']                = 0.0
        param_space['min_temperature']                    = 0.0
        param_space['cooling_rate']                       = 0.0
        param_space['group_number']                       = 0
        param_space['number_elite']                       = 0
        param_space['sa_max_time_sec']                    = 0.0
        param_space['accept_cost_threshold']              = 0.0
        param_space["use_firefly_operator"]               = 0
        #param_space["use_original_luminosity_multiplier"] = False
        parameter_constraints=["tweak_dwell_time_max_change   - max_dwell_time_depot   <= -1.0",
                               "tweak_dwell_time_max_change   - max_dwell_time_hotspot <= -1.0",
                               "initialization_max_dwell_time - max_dwell_time_depot   <= 0.0",
                               "initialization_max_dwell_time - max_dwell_time_hotspot <= 0.0"]
    elif alg_ind == 1:
        alg_name                             = 'fa'
        param_space['initial_temperature']   = 0.0
        param_space['min_temperature']       = 0.0
        param_space['cooling_rate']          = 0.0
        param_space['group_number']          = 0
        param_space['number_elite']          = 0
        param_space['sa_max_time_sec']       = 0.0
        param_space['accept_cost_threshold'] = 0.0
        param_space["use_firefly_operator"]  = 1
        parameter_constraints=["tweak_dwell_time_max_change   - max_dwell_time_depot   <= -1.0",
                               "tweak_dwell_time_max_change   - max_dwell_time_hotspot <= -1.0",
                               "initialization_max_dwell_time - max_dwell_time_depot   <= 0.0",
                               "initialization_max_dwell_time - max_dwell_time_hotspot <= 0.0"]
    elif alg_ind == 2:
        alg_name                             = 'fab'
        param_space['initial_temperature']   = 0.0
        param_space['min_temperature']       = 0.0
        param_space['cooling_rate']          = 0.0
        param_space['group_number']          = 0
        param_space['number_elite']          = 0
        param_space['sa_max_time_sec']       = 0.0
        param_space['accept_cost_threshold'] = 0.0
        param_space["use_firefly_operator"]  = 2
        parameter_constraints=["tweak_dwell_time_max_change   - max_dwell_time_depot   <= -1.0",
                               "tweak_dwell_time_max_change   - max_dwell_time_hotspot <= -1.0",
                               "initialization_max_dwell_time - max_dwell_time_depot   <= 0.0",
                               "initialization_max_dwell_time - max_dwell_time_hotspot <= 0.0"]
    elif alg_ind == 3:
        alg_name                                          = 'sa'
        param_space["distance_power"]                     = 0.0
        param_space["distance_multiplier"]                = 0.0
        param_space["base_attractiveness_multiplier"]     = 0.0
        param_space['local_search_probability']           = 0.0
        param_space['use_firefly_operator']               = 3
        #param_space["use_original_luminosity_multiplier"] = False
        parameter_constraints=["tweak_dwell_time_max_change   - max_dwell_time_depot   <= -1.0",
                               "tweak_dwell_time_max_change   - max_dwell_time_hotspot <= -1.0",
                               "initialization_max_dwell_time - max_dwell_time_depot   <= 0.0",
                               "initialization_max_dwell_time - max_dwell_time_hotspot <= 0.0"]
        parameter_constraints.append("number_elite    - population_size     <= -1.0")
        parameter_constraints.append("min_temperature - initial_temperature <= 0.0")
    elif alg_ind == 4:
        alg_name                                          = 'pr'
        param_space["distance_power"]                     = 0.0
        param_space["distance_multiplier"]                = 0.0
        param_space["base_attractiveness_multiplier"]     = 0.0
        param_space['initial_temperature']                = 0.0
        param_space['min_temperature']                    = 0.0
        param_space['cooling_rate']                       = 0.0
        param_space['group_number']                       = 0
        param_space['number_elite']                       = 0
        param_space['sa_max_time_sec']                    = 0.0
        param_space['accept_cost_threshold']              = 0.0
        param_space['use_firefly_operator']               = 4
        parameter_constraints=["tweak_dwell_time_max_change   - max_dwell_time_depot   <= -1.0",
                               "tweak_dwell_time_max_change   - max_dwell_time_hotspot <= -1.0",
                               "initialization_max_dwell_time - max_dwell_time_depot   <= 0.0",
                               "initialization_max_dwell_time - max_dwell_time_hotspot <= 0.0"]
        #param_space["use_original_luminosity_multiplier"] = False
    elif alg_ind == 5:
        alg_name                                      = 'rand'
        param_space['local_search_max_time_sec']      = 0.0
        param_space['field_accent_width']             = 0.0
        param_space['field_accent_search_weight']     = 0.0
        param_space['field_accent_avoid_weight']      = 0.0
        param_space['population_size']                = 0
        param_space['max_dwell_time_depot']           = 0
        param_space['max_dwell_time_hotspot']         = 0
        param_space["distance_power"]                 = 0.0
        param_space["distance_multiplier"]            = 0.0
        param_space["base_attractiveness_multiplier"] = 0.0
        param_space['tweak_dwell_time_max_change']    = 0
        param_space['local_search_probability']       = 0.0
        param_space['initial_temperature']            = 0.0
        param_space['min_temperature']                = 0.0
        param_space['cooling_rate']                   = 0.0
        param_space['group_number']                   = 0
        param_space['sa_max_time_sec']                = 0.0
        param_space['number_elite']                   = 0
        param_space['accept_cost_threshold']          = 0.0
        param_space['use_firefly_operator']           = 5
        param_space['mop_add_depot_visit']            = 0.0
        param_space['mop_add_hotspot_visit']          = 0.0
        param_space['mop_remove_depot_visit']         = 0.0
        param_space['mop_remove_hotspot_visit']       = 0.0
        param_space['mop_remove_visits_windowed']     = 0.0
        param_space['mop_sim']                        = 0.0
        param_space['mop_swap_multiple_visits']       = 0.0
        param_space['mop_swap_path']                  = 0.0
        param_space['mop_swap_visits']                = 0.0
        param_space['mop_tweak_dwell_time']           = 0.0
    elif alg_ind == 6:
        alg_name                                      = 'greedy'
        param_space['local_search_max_time_sec']      = 0.0
        param_space['field_accent_width']             = 0.0
        param_space['field_accent_search_weight']     = 0.0
        param_space['field_accent_avoid_weight']      = 0.0
        param_space['population_size']                = 0
        param_space['max_dwell_time_depot']           = 0
        param_space['max_dwell_time_hotspot']         = 0
        param_space["distance_power"]                 = 0.0
        param_space["distance_multiplier"]            = 0.0
        param_space["base_attractiveness_multiplier"] = 0.0
        param_space['tweak_dwell_time_max_change']    = 0
        param_space['local_search_probability']       = 0.0
        param_space['initial_temperature']            = 0.0
        param_space['min_temperature']                = 0.0
        param_space['cooling_rate']                   = 0.0
        param_space['group_number']                   = 0
        param_space['sa_max_time_sec']                = 0.0
        param_space['number_elite']                   = 0
        param_space['accept_cost_threshold']          = 0.0
        param_space['use_firefly_operator']           = 6
        param_space['mop_add_depot_visit']            = 0.0
        param_space['mop_add_hotspot_visit']          = 0.0
        param_space['mop_remove_depot_visit']         = 0.0
        param_space['mop_remove_hotspot_visit']       = 0.0
        param_space['mop_remove_visits_windowed']     = 0.0
        param_space['mop_sim']                        = 0.0
        param_space['mop_swap_multiple_visits']       = 0.0
        param_space['mop_swap_path']                  = 0.0
        param_space['mop_swap_visits']                = 0.0
        param_space['mop_tweak_dwell_time']           = 0.0
    elif alg_ind == 7:
        alg_name                                      = 'field_accent'
        param_space['local_search_max_time_sec']      = 0.0
        param_space['population_size']                = 0
        param_space['max_dwell_time_depot']           = 0
        param_space['max_dwell_time_hotspot']         = 0
        param_space["distance_power"]                 = 0.0
        param_space["distance_multiplier"]            = 0.0
        param_space["base_attractiveness_multiplier"] = 0.0
        param_space['tweak_dwell_time_max_change']    = 0
        param_space['local_search_probability']       = 0.0
        param_space['initial_temperature']            = 0.0
        param_space['min_temperature']                = 0.0
        param_space['cooling_rate']                   = 0.0
        param_space['group_number']                   = 0
        param_space['sa_max_time_sec']                = 0.0
        param_space['number_elite']                   = 0
        param_space['accept_cost_threshold']          = 0.0
        param_space['use_firefly_operator']           = 7
        param_space['mop_add_depot_visit']            = 0.0
        param_space['mop_add_hotspot_visit']          = 0.0
        param_space['mop_remove_depot_visit']         = 0.0
        param_space['mop_remove_hotspot_visit']       = 0.0
        param_space['mop_remove_visits_windowed']     = 0.0
        param_space['mop_sim']                        = 0.0
        param_space['mop_swap_multiple_visits']       = 0.0
        param_space['mop_swap_path']                  = 0.0
        param_space['mop_swap_visits']                = 0.0
        param_space['mop_tweak_dwell_time']           = 0.0
    else:
        raise "not a valid alg_ind"
    selection_name = ''
    if selection_ind == 0:
        selection_name                           = 'fitness'
        param_space['use_selection']             = 0
        param_space['use_elitism_selection']     = True
        param_space['number_clusters']           = 0
        param_space['number_cluster_iterations'] = 0
    elif selection_ind == 1:
        selection_name                           = 'sus'
        param_space['use_selection']             = 1
        param_space['number_clusters']           = 0
        param_space['number_cluster_iterations'] = 0
    elif selection_ind == 2:
        selection_name                           = 'fitness_proportional'
        param_space['use_selection']             = 2
        param_space['number_clusters']           = 0
        param_space['number_cluster_iterations'] = 0
    elif selection_ind == 3:
        selection_name               = 'k_means'
        param_space['use_selection'] = 3
        parameter_constraints.append("number_clusters - population_size <= -1.0")
        if alg_ind == 3: # SA
            parameter_constraints.append("number_clusters - number_elite <= 0.0")
    else:
        raise "not a valid selection_ind"
    restore_run = os.path.exists(path="~/ray_results/alg_" + alg_name + "_selection_" + selection_name)
    # gamma bigger means more focused best samples so far
    #search = HyperOptSearch(gamma=0.25, n_initial_points=0, points_to_evaluate=pnt_to_eval)#, random_state_seed=42)
    # gamma bigger means more focused best samples so far
    search = AxSearch(points_to_evaluate=pnt_to_eval, parameter_constraints=parameter_constraints)
    tuner = tune.Tuner(
        objective,
        tune_config=tune.TuneConfig(
            search_alg=search,
            metric="metric",
            mode="min",
            num_samples=num_samples,
            max_concurrent_trials=1 + int(only_one_tuner_running),
            time_budget_s=24*60*60*1000,
        ),
        param_space=param_space,
        run_config=RunConfig(storage_path="/home/james/ray_results",
                             name="alg_" + alg_name + "_selection_" + selection_name)
    )
    # if tune.Tuner.can_restore(path):
    if restore_run:
        tuner = tune.Tuner.restore(path="/home/james/ray_results/alg_" + alg_name + "_selection_" + selection_name, trainable=objective, resume_unfinished=True, resume_errored=False, restart_errored=True)
    results = tuner.fit()


if __name__ == '__main__':
    process_queue = []
    # 0: No crossover, 1: firefly, 2: bidirectional firefly, 3: simulated annealing, 4: path relinking, 5: Rand, 6: Greedy, 7: Field accent
    list_of_algs = [1, 3, 4, 5, 6, 7]
    # 0: fitness selection, 1: full random selection, 2: fitness proportional selection, 3: k-means selection
    list_of_selections = [2, 3]
    for alg_ind in list_of_algs:
        if alg_ind >= 5:
            p = multiprocessing.Process(target=run_func, args=(alg_ind, 0, (len(list_of_algs) == 1) and (len(list_of_selections) == 1)))
            p.start()
            process_queue.append(p)
        else:
            for selection_ind in list_of_selections:
                p = multiprocessing.Process(target=run_func, args=(alg_ind, selection_ind, (len(list_of_algs) == 1) and (len(list_of_selections) == 1)))
                p.start()
                process_queue.append(p)
    while len(process_queue) > 0:
        time.sleep(1)
        for process in process_queue:
            process.join(timeout=1.0)
        process_queue = [item for item in process_queue if item.is_alive()]


