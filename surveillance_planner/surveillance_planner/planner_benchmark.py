#!/usr/bin/env python3
##
# @File: planner_benchmark.py
#

import os
from graph_surveillance_planning_msgs.srv import TryFireflyConfig
import numpy as np
import time
import rclpy
from rclpy.node import Node
import random
import sys
import csv
import math


result_base_dir    = "/home/james/planner_benchmark/los_angeles"
num_runs           = 530
max_solve_time_sec = 15.0 * 60.0
avg_max_time_sec   = 16.0 * 60.0
avg_dt_sec         = 1.0


class PlannerClient(Node):
    def __init__(self):
        super().__init__('firefly_planner_tuner')
        self.client = self.create_client(TryFireflyConfig, 'firefly_config_test')
        while not self.client.wait_for_service(timeout_sec=15.0):
            self.get_logger().info('service not available, waiting again...')


def objective(config, run_ind):
    alg_name = ''
    if config['use_firefly_operator'] == 0:
        alg_name = 'rands'
    elif config['use_firefly_operator'] == 1:
        alg_name = 'fa'
    elif config['use_firefly_operator'] == 2:
        alg_name = 'fab'
    elif config['use_firefly_operator'] == 3:
        alg_name = 'sa'
    elif config['use_firefly_operator'] == 4:
        alg_name = 'pr'
    elif config['use_firefly_operator'] == 5:
        alg_name = 'rand'
    elif config['use_firefly_operator'] == 6:
        alg_name = 'greedy'
    elif config['use_firefly_operator'] == 7:
        alg_name = 'field_accent'
    else:
        raise "not a valid alg_ind"
    selection_name = ''
    if config['use_selection'] == 0:
        selection_name = 'fitness'
    elif config['use_selection'] == 1:
        selection_name = 'sus'
    elif config['use_selection'] == 2:
        selection_name = 'fitness_proportional'
    elif config['use_selection'] == 3:
        selection_name = 'k_means'
    else:
        raise "not a valid selection_ind"

    if not os.path.exists(path=result_base_dir + "/" + alg_name + "_selection_" + selection_name):
        os.mkdir(path=result_base_dir + "/" + alg_name + "_selection_" + selection_name)
    # Run planner
    if not os.path.exists(result_base_dir + "/" + alg_name + "_selection_" + selection_name + "/run_" + str(run_ind) + ".csv"): # Don't redo any runs
        req = TryFireflyConfig.Request()

        req.max_solve_time_sec                         = max_solve_time_sec
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

        rclpy.init()
        planner_client = PlannerClient()

        future = planner_client.client.call_async(req)
        rclpy.spin_until_future_complete(node=planner_client, future=future)
        response = future.result()

        # Save result off
        with open(result_base_dir + "/" + alg_name + "_selection_" + selection_name + "/run_" + str(run_ind) + ".csv", 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["best_cost",
                             "time_sec",
                             "ecr_two_norm",
                             "ecr_inf_norm",
                             "response_time_two_norm",
                             "response_time_inf_norm",
                            ])
            for gen_ind in range(len(response.cost_gen)):
                writer.writerow([response.cost_gen[gen_ind],
                                 response.time_gen_sec[gen_ind],
                                 response.ecr_two_norm_gen[gen_ind],
                                 response.ecr_inf_norm_gen[gen_ind],
                                 response.response_time_two_norm_gen[gen_ind],
                                 response.response_time_inf_norm_gen[gen_ind],
                                ])

        planner_client.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    # Generate run files
    if not os.path.exists(path=result_base_dir):
        os.mkdir(path=result_base_dir)
    if False:
        for run_ind in range(num_runs):
            objective({
                "accept_cost_threshold": 0.0,
                "base_attractiveness_multiplier": 0.0,
                "cooling_rate": 0.0,
                "distance_multiplier": 0.0,
                "distance_power": 0.0,
                "field_accent_avoid_weight": 8725.977444928027,
                "field_accent_search_weight": 500.0,
                "field_accent_width": 30590.196900456613,
                "group_number": 0,
                "initial_temperature": 0.0,
                "initialization_max_dwell_time": 12,
                "local_search_max_time_sec": 0.0,
                "local_search_probability": 0.0,
                "max_dwell_time_depot": 0,
                "max_dwell_time_hotspot": 0,
                "min_temperature": 0.0,
                "mop_add_depot_visit": 0.0,
                "mop_add_hotspot_visit": 0.0,
                "mop_remove_depot_visit": 0.0,
                "mop_remove_hotspot_visit": 0.0,
                "mop_remove_visits_windowed": 0.0,
                "mop_sim": 0.0,
                "mop_swap_multiple_visits": 0.0,
                "mop_swap_path": 0.0,
                "mop_swap_visits": 0.0,
                "mop_tweak_dwell_time": 0.0,
                "number_cluster_iterations": 0,
                "number_clusters": 0,
                "number_elite": 0,
                "population_size": 0,
                "sa_max_time_sec": 0.0,
                "tweak_dwell_time_max_change": 0,
                "use_elitism_selection": True,
                "use_firefly_operator": 7,
                "use_selection": 0
                }, run_ind=run_ind)
            objective({
                "accept_cost_threshold": 0.0,
                "base_attractiveness_multiplier": 1.572705328835258e-13,
                "cooling_rate": 0.0,
                "distance_multiplier": 5.07586730218853e-14,
                "distance_power": 0.7719541389212111,
                "field_accent_avoid_weight": 9999.999999999998,
                "field_accent_search_weight": 499.9999995118326,
                "field_accent_width": 10773.63327716815,
                "group_number": 0,
                "initial_temperature": 0.0,
                "initialization_max_dwell_time": 11,
                "local_search_max_time_sec": 95.99502893314212,
                "local_search_probability": 0.0104720861448853,
                "max_dwell_time_depot": 80,
                "max_dwell_time_hotspot": 11,
                "min_temperature": 0.0,
                "mop_add_depot_visit": 0.7156575765304729,
                "mop_add_hotspot_visit": 1.7847191438898968e-16,
                "mop_remove_depot_visit": 0.9999999999999971,
                "mop_remove_hotspot_visit": 5.921007714293505e-16,
                "mop_remove_visits_windowed": 7.85814215383525e-16,
                "mop_sim": 0.7974275776591029,
                "mop_swap_multiple_visits": 0.9999999999999997,
                "mop_swap_path": 0.5880437526956979,
                "mop_swap_visits": 0.6059433245677313,
                "mop_tweak_dwell_time": 0.4717434671682534,
                "number_cluster_iterations": 0,
                "number_clusters": 0,
                "number_elite": 0,
                "population_size": 3,
                "sa_max_time_sec": 0.0,
                "tweak_dwell_time_max_change": 5,
                "use_firefly_operator": 1,
                "use_selection": 2
                }, run_ind=run_ind)
            objective({
                "accept_cost_threshold": 0.0,
                "base_attractiveness_multiplier": 100.0,
                "cooling_rate": 0.0,
                "distance_multiplier": 100.0,
                "distance_power": 1e-06,
                "field_accent_avoid_weight": 300.0,
                "field_accent_search_weight": 2.5758829919007785e-06,
                "field_accent_width": 1e-12,
                "group_number": 0,
                "initial_temperature": 0.0,
                "initialization_max_dwell_time": 11,
                "local_search_max_time_sec": 0.0,
                "local_search_probability": 0.0,
                "max_dwell_time_depot": 499,
                "max_dwell_time_hotspot": 399,
                "min_temperature": 0.0,
                "mop_add_depot_visit": 1.0,
                "mop_add_hotspot_visit": 1.0,
                "mop_remove_depot_visit": 0.9999999911824962,
                "mop_remove_hotspot_visit": 1.360203113839723e-09,
                "mop_remove_visits_windowed": 0.0,
                "mop_sim": 1.0,
                "mop_swap_multiple_visits": 0.5405645939049099,
                "mop_swap_path": 0.0,
                "mop_swap_visits": 1.0,
                "mop_tweak_dwell_time": 0.6239542316305345,
                "number_cluster_iterations": 0,
                "number_clusters": 1,
                "number_elite": 0,
                "population_size": 3,
                "sa_max_time_sec": 0.0,
                "tweak_dwell_time_max_change": 398,
                "use_firefly_operator": 1,
                "use_selection": 3
                }, run_ind=run_ind)
            objective({
                "accept_cost_threshold": 0.0,
                "base_attractiveness_multiplier": 0.0,
                "cooling_rate": 0.0,
                "distance_multiplier": 0.0,
                "distance_power": 0.0,
                "field_accent_avoid_weight": 0.0,
                "field_accent_search_weight": 0.0,
                "field_accent_width": 0.0,
                "group_number": 0,
                "initial_temperature": 0.0,
                "initialization_max_dwell_time": 11,
                "local_search_max_time_sec": 0.0,
                "local_search_probability": 0.0,
                "max_dwell_time_depot": 0,
                "max_dwell_time_hotspot": 0,
                "min_temperature": 0.0,
                "mop_add_depot_visit": 0.0,
                "mop_add_hotspot_visit": 0.0,
                "mop_remove_depot_visit": 0.0,
                "mop_remove_hotspot_visit": 0.0,
                "mop_remove_visits_windowed": 0.0,
                "mop_sim": 0.0,
                "mop_swap_multiple_visits": 0.0,
                "mop_swap_path": 0.0,
                "mop_swap_visits": 0.0,
                "mop_tweak_dwell_time": 0.0,
                "number_cluster_iterations": 0,
                "number_clusters": 0,
                "number_elite": 0,
                "population_size": 0,
                "sa_max_time_sec": 0.0,
                "tweak_dwell_time_max_change": 0,
                "use_elitism_selection": True,
                "use_firefly_operator": 6,
                "use_selection": 0
                }, run_ind=run_ind)
            objective({
                "accept_cost_threshold": 0.0,
                "base_attractiveness_multiplier": 0.0,
                "cooling_rate": 0.0,
                "distance_multiplier": 0.0,
                "distance_power": 0.0,
                "field_accent_avoid_weight": 300.00000011947213,
                "field_accent_search_weight": 1.7768250418958833e-16,
                "field_accent_width": 2.8925946342714e-12,
                "group_number": 0,
                "initial_temperature": 0.0,
                "initialization_max_dwell_time": 8,
                "local_search_max_time_sec": 1.9981002781760185e-14,
                "local_search_probability": 0.44675905401986865,
                "max_dwell_time_depot": 499,
                "max_dwell_time_hotspot": 399,
                "min_temperature": 0.0,
                "mop_add_depot_visit": 1.55299350146828e-16,
                "mop_add_hotspot_visit": 0.9999999999999999,
                "mop_remove_depot_visit": 0.9999999999999996,
                "mop_remove_hotspot_visit": 4.0677770724811616e-16,
                "mop_remove_visits_windowed": 9.17170942606965e-17,
                "mop_sim": 0.82044272101412,
                "mop_swap_multiple_visits": 5.024513041625141e-10,
                "mop_swap_path": 5.63192535568385e-16,
                "mop_swap_visits": 0.944538081524814,
                "mop_tweak_dwell_time": 0.9999999999999999,
                "number_cluster_iterations": 0,
                "number_clusters": 0,
                "number_elite": 0,
                "population_size": 3,
                "sa_max_time_sec": 0.0,
                "tweak_dwell_time_max_change": 398,
                "use_firefly_operator": 4,
                "use_selection": 2
                }, run_ind=run_ind)
            objective({
                "accept_cost_threshold": 0.0,
                "base_attractiveness_multiplier": 0.0,
                "cooling_rate": 0.0,
                "distance_multiplier": 0.0,
                "distance_power": 0.0,
                "field_accent_avoid_weight": 9999.999999999996,
                "field_accent_search_weight": 499.9999999999997,
                "field_accent_width": 46168.29327812841,
                "group_number": 0,
                "initial_temperature": 0.0,
                "initialization_max_dwell_time": 11,
                "local_search_max_time_sec": 59.455358329342296,
                "local_search_probability": 0.12002047492784901,
                "max_dwell_time_depot": 334,
                "max_dwell_time_hotspot": 11,
                "min_temperature": 0.0,
                "mop_add_depot_visit": 0.9386350810164745,
                "mop_add_hotspot_visit": 0.9999999999999993,
                "mop_remove_depot_visit": 7.104980349886509e-16,
                "mop_remove_hotspot_visit": 1.0,
                "mop_remove_visits_windowed": 0.26886277090791544,
                "mop_sim": 3.7089610701782246e-16,
                "mop_swap_multiple_visits": 0.011334924425652123,
                "mop_swap_path": 1.3115941951477971e-15,
                "mop_swap_visits": 3.6454000238890817e-16,
                "mop_tweak_dwell_time": 1.0,
                "number_cluster_iterations": 0,
                "number_clusters": 1,
                "number_elite": 0,
                "population_size": 3,
                "sa_max_time_sec": 0.0,
                "tweak_dwell_time_max_change": 5,
                "use_firefly_operator": 4,
                "use_selection": 3
                }, run_ind=run_ind)
            objective({
                "accept_cost_threshold": 0.0,
                "base_attractiveness_multiplier": 0.0,
                "cooling_rate": 0.0,
                "distance_multiplier": 0.0,
                "distance_power": 0.0,
                "field_accent_avoid_weight": 0.0,
                "field_accent_search_weight": 0.0,
                "field_accent_width": 0.0,
                "group_number": 0,
                "initial_temperature": 0.0,
                "initialization_max_dwell_time": 10,
                "local_search_max_time_sec": 0.0,
                "local_search_probability": 0.0,
                "max_dwell_time_depot": 0,
                "max_dwell_time_hotspot": 0,
                "min_temperature": 0.0,
                "mop_add_depot_visit": 0.0,
                "mop_add_hotspot_visit": 0.0,
                "mop_remove_depot_visit": 0.0,
                "mop_remove_hotspot_visit": 0.0,
                "mop_remove_visits_windowed": 0.0,
                "mop_sim": 0.0,
                "mop_swap_multiple_visits": 0.0,
                "mop_swap_path": 0.0,
                "mop_swap_visits": 0.0,
                "mop_tweak_dwell_time": 0.0,
                "number_cluster_iterations": 0,
                "number_clusters": 0,
                "number_elite": 0,
                "population_size": 0,
                "sa_max_time_sec": 0.0,
                "tweak_dwell_time_max_change": 0,
                "use_elitism_selection": True,
                "use_firefly_operator": 5,
                "use_selection": 0
                }, run_ind=run_ind)
            objective({
                "accept_cost_threshold": 5.0,
                "base_attractiveness_multiplier": 0.0,
                "cooling_rate": 1.0,
                "distance_multiplier": 0.0,
                "distance_power": 0.0,
                "field_accent_avoid_weight": 10000.0,
                "field_accent_search_weight": 500.0,
                "field_accent_width": 6.892008148842907e-12,
                "group_number": 99,
                "initial_temperature": 186.23035199579746,
                "initialization_max_dwell_time": 8,
                "local_search_max_time_sec": 0.0,
                "local_search_probability": 0.0,
                "max_dwell_time_depot": 499,
                "max_dwell_time_hotspot": 8,
                "min_temperature": 0.0,
                "mop_add_depot_visit": 4.380689156201173e-17,
                "mop_add_hotspot_visit": 0.13024162457965902,
                "mop_remove_depot_visit": 0.9999176955107811,
                "mop_remove_hotspot_visit": 0.0,
                "mop_remove_visits_windowed": 0.0,
                "mop_sim": 2.233915627395141e-16,
                "mop_swap_multiple_visits": 0.0001615596835815943,
                "mop_swap_path": 7.046226006311721e-17,
                "mop_swap_visits": 1.0,
                "mop_tweak_dwell_time": 1.0,
                "number_cluster_iterations": 0,
                "number_clusters": 0,
                "number_elite": 44,
                "population_size": 45,
                "sa_max_time_sec": 224.99999999999997,
                "tweak_dwell_time_max_change": 5,
                "use_firefly_operator": 3,
                "use_selection": 2
                }, run_ind=run_ind)
            objective({
                "accept_cost_threshold": 4.492767687969428,
                "base_attractiveness_multiplier": 0.0,
                "cooling_rate": 0.5208223062620193,
                "distance_multiplier": 0.0,
                "distance_power": 0.0,
                "field_accent_avoid_weight": 4206.805902608776,
                "field_accent_search_weight": 499.99999999864946,
                "field_accent_width": 160000.0,
                "group_number": 99,
                "initial_temperature": 1499.9999994596185,
                "initialization_max_dwell_time": 11,
                "local_search_max_time_sec": 4.252599963357624e-08,
                "local_search_probability": 0.0,
                "max_dwell_time_depot": 399,
                "max_dwell_time_hotspot": 399,
                "min_temperature": 39.82447654286113,
                "mop_add_depot_visit": 0.999999999958731,
                "mop_add_hotspot_visit": 0.9999999990701445,
                "mop_remove_depot_visit": 1.0,
                "mop_remove_hotspot_visit": 6.40416609100157e-11,
                "mop_remove_visits_windowed": 1.5485579920406174e-09,
                "mop_sim": 0.9999999995631943,
                "mop_swap_multiple_visits": 1.0,
                "mop_swap_path": 2.8694021456207137e-11,
                "mop_swap_visits": 0.9999999999812859,
                "mop_tweak_dwell_time": 0.9999999998540277,
                "number_cluster_iterations": 0,
                "number_clusters": 1,
                "number_elite": 22,
                "population_size": 23,
                "sa_max_time_sec": 192.9669244178827,
                "tweak_dwell_time_max_change": 398,
                "use_firefly_operator": 3,
                "use_selection": 3
                }, run_ind=run_ind)
            print("Run: " + str(run_ind) + " finished")

    # Make summery files
    if True:
        # Make interpolated results
        new_times = np.linspace(start=0.0, stop=avg_max_time_sec, num=int(math.ceil(avg_max_time_sec/avg_dt_sec)), dtype=float)
        for alg_name in os.listdir(result_base_dir):
            alg_results_path = os.path.join(result_base_dir, alg_name)
            if os.path.isdir(alg_results_path):
                # Interpolate results
                interp_best_costs              = np.full(shape=(len(new_times), len(os.listdir(alg_results_path))), fill_value=np.nan, dtype=float)
                interp_gen_ran                 = np.full(shape=(len(new_times), len(os.listdir(alg_results_path))), fill_value=np.nan, dtype=float)
                interp_ecr_two_norms           = np.full(shape=(len(new_times), len(os.listdir(alg_results_path))), fill_value=np.nan, dtype=float)
                interp_ecr_inf_norms           = np.full(shape=(len(new_times), len(os.listdir(alg_results_path))), fill_value=np.nan, dtype=float)
                interp_response_time_two_norms = np.full(shape=(len(new_times), len(os.listdir(alg_results_path))), fill_value=np.nan, dtype=float)
                interp_response_time_inf_norms = np.full(shape=(len(new_times), len(os.listdir(alg_results_path))), fill_value=np.nan, dtype=float)
                for run_ind, run_name in enumerate(os.listdir(alg_results_path)):
                    run_path = os.path.join(alg_results_path, run_name)
                    best_costs_gen             = np.genfromtxt(fname=run_path, dtype=float, delimiter=',', usecols=(0), skip_header=1)
                    times_gen                  = np.genfromtxt(fname=run_path, dtype=float, delimiter=',', usecols=(1), skip_header=1)
                    ecr_two_norm_gen           = np.genfromtxt(fname=run_path, dtype=float, delimiter=',', usecols=(2), skip_header=1)
                    ecr_inf_norm_gen           = np.genfromtxt(fname=run_path, dtype=float, delimiter=',', usecols=(3), skip_header=1)
                    response_time_two_norm_gen = np.genfromtxt(fname=run_path, dtype=float, delimiter=',', usecols=(4), skip_header=1)
                    response_time_inf_norm_gen = np.genfromtxt(fname=run_path, dtype=float, delimiter=',', usecols=(5), skip_header=1)

                    if best_costs_gen.size == 1:
                        interp_best_costs[             :, run_ind] = best_costs_gen
                        interp_gen_ran[                :, run_ind] = 0
                        interp_ecr_two_norms[          :, run_ind] = ecr_two_norm_gen
                        interp_ecr_inf_norms[          :, run_ind] = ecr_inf_norm_gen
                        interp_response_time_two_norms[:, run_ind] = response_time_two_norm_gen
                        interp_response_time_inf_norms[:, run_ind] = response_time_inf_norm_gen
                    else:
                        interp_best_costs[             :, run_ind] = np.interp(x=new_times, xp=times_gen, fp=best_costs_gen)
                        interp_gen_ran[                :, run_ind] = np.interp(x=new_times, xp=times_gen, fp=np.arange(start=0, stop=len(times_gen), step=1, dtype=float))
                        interp_ecr_two_norms[          :, run_ind] = np.interp(x=new_times, xp=times_gen, fp=ecr_two_norm_gen)
                        interp_ecr_inf_norms[          :, run_ind] = np.interp(x=new_times, xp=times_gen, fp=ecr_inf_norm_gen)
                        interp_response_time_two_norms[:, run_ind] = np.interp(x=new_times, xp=times_gen, fp=response_time_two_norm_gen)
                        interp_response_time_inf_norms[:, run_ind] = np.interp(x=new_times, xp=times_gen, fp=response_time_inf_norm_gen)
                # Save summaries off
                with open(result_base_dir + "/" + alg_name + "_interp.csv", 'w') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(["time_sec",
                                     "mean_cost",
                                     "max_cost",
                                     "min_cost",
                                     "std_cost",
                                     "avg_plus_three_std_cost",
                                     "avg_minus_three_std_cost",
                                     "mean_gen_ran",
                                     "max_gen_ran",
                                     "min_gen_ran",
                                     "std_gen_ran",
                                     "avg_plus_three_std_gen_ran",
                                     "avg_minus_three_std_gen_ran",
                                     "mean_ecr_two_norm",
                                     "max_ecr_two_norm",
                                     "min_ecr_two_norm",
                                     "std_ecr_two_norm",
                                     "avg_plus_three_std_ecr_two_norm",
                                     "avg_minus_three_std_ecr_two_norm",
                                     "mean_ecr_inf_norm",
                                     "max_ecr_inf_norm",
                                     "min_ecr_inf_norm",
                                     "std_ecr_inf_norm",
                                     "avg_plus_three_std_ecr_inf_norm",
                                     "avg_minus_three_std_ecr_inf_norm",
                                     "mean_rt_two_norm",
                                     "max_rt_two_norm",
                                     "min_rt_two_norm",
                                     "std_rt_two_norm",
                                     "avg_plus_three_std_rt_two_norm",
                                     "avg_minus_three_std_rt_two_norm",
                                     "mean_rt_inf_norm",
                                     "max_rt_inf_norm",
                                     "min_rt_inf_norm",
                                     "std_rt_inf_norm",
                                     "avg_plus_three_std_rt_inf_norm",
                                     "avg_minus_three_std_rt_inf_norm",
                                    ])
                    for time_ind in range(len(new_times)):
                        writer.writerow([new_times[time_ind],
                                         np.average(interp_best_costs[time_ind, :]),
                                         np.max(interp_best_costs[time_ind, :]),
                                         np.min(interp_best_costs[time_ind, :]),
                                         np.std(interp_best_costs[time_ind, :]),
                                         np.average(interp_best_costs[time_ind, :]) + (3.0 * np.std(interp_best_costs[time_ind, :])),
                                         np.average(interp_best_costs[time_ind, :]) - (3.0 * np.std(interp_best_costs[time_ind, :])),
                                         np.average(interp_gen_ran[time_ind, :]),
                                         np.max(interp_gen_ran[time_ind, :]),
                                         np.min(interp_gen_ran[time_ind, :]),
                                         np.std(interp_gen_ran[time_ind, :]),
                                         np.average(interp_gen_ran[time_ind, :]) + (3.0 * np.std(interp_gen_ran[time_ind, :])),
                                         np.average(interp_gen_ran[time_ind, :]) - (3.0 * np.std(interp_gen_ran[time_ind, :])),
                                         np.average(interp_ecr_two_norms[time_ind, :]),
                                         np.max(interp_ecr_two_norms[time_ind, :]),
                                         np.min(interp_ecr_two_norms[time_ind, :]),
                                         np.std(interp_ecr_two_norms[time_ind, :]),
                                         np.average(interp_ecr_two_norms[time_ind, :]) + (3.0 * np.std(interp_ecr_two_norms[time_ind, :])),
                                         np.average(interp_ecr_two_norms[time_ind, :]) - (3.0 * np.std(interp_ecr_two_norms[time_ind, :])),
                                         np.average(interp_ecr_inf_norms[time_ind, :]),
                                         np.max(interp_ecr_inf_norms[time_ind, :]),
                                         np.min(interp_ecr_inf_norms[time_ind, :]),
                                         np.std(interp_ecr_inf_norms[time_ind, :]),
                                         np.average(interp_ecr_inf_norms[time_ind, :]) + (3.0 * np.std(interp_ecr_inf_norms[time_ind, :])),
                                         np.average(interp_ecr_inf_norms[time_ind, :]) - (3.0 * np.std(interp_ecr_inf_norms[time_ind, :])),
                                         np.average(interp_response_time_two_norms[time_ind, :]),
                                         np.max(interp_response_time_two_norms[time_ind, :]),
                                         np.min(interp_response_time_two_norms[time_ind, :]),
                                         np.std(interp_response_time_two_norms[time_ind, :]),
                                         np.average(interp_response_time_two_norms[time_ind, :]) + (3.0 * np.std(interp_response_time_two_norms[time_ind, :])),
                                         np.average(interp_response_time_two_norms[time_ind, :]) - (3.0 * np.std(interp_response_time_two_norms[time_ind, :])),
                                         np.average(interp_response_time_inf_norms[time_ind, :]),
                                         np.max(interp_response_time_inf_norms[time_ind, :]),
                                         np.min(interp_response_time_inf_norms[time_ind, :]),
                                         np.std(interp_response_time_inf_norms[time_ind, :]),
                                         np.average(interp_response_time_inf_norms[time_ind, :]) + (3.0 * np.std(interp_response_time_inf_norms[time_ind, :])),
                                         np.average(interp_response_time_inf_norms[time_ind, :]) - (3.0 * np.std(interp_response_time_inf_norms[time_ind, :])),
                                        ])
        # Make per gen results
        for alg_name in os.listdir(result_base_dir):
            alg_results_path = os.path.join(result_base_dir, alg_name)
            if os.path.isdir(alg_results_path):
                # Find the minimum number of generations ran across all runs
                min_gen_ran = sys.maxsize
                for run_ind, run_name in enumerate(os.listdir(alg_results_path)):
                    run_path = os.path.join(alg_results_path, run_name)
                    min_gen_ran = min(min_gen_ran, np.genfromtxt(fname=run_path, dtype=float, delimiter=',', usecols=(0), skip_header=1).size)
                # Gather by generation results
                best_costs              = np.full(shape=(min_gen_ran, len(os.listdir(alg_results_path))), fill_value=np.nan, dtype=float)
                time_sec                = np.full(shape=(min_gen_ran, len(os.listdir(alg_results_path))), fill_value=np.nan, dtype=float)
                ecr_two_norms           = np.full(shape=(min_gen_ran, len(os.listdir(alg_results_path))), fill_value=np.nan, dtype=float)
                ecr_inf_norms           = np.full(shape=(min_gen_ran, len(os.listdir(alg_results_path))), fill_value=np.nan, dtype=float)
                response_time_two_norms = np.full(shape=(min_gen_ran, len(os.listdir(alg_results_path))), fill_value=np.nan, dtype=float)
                response_time_inf_norms = np.full(shape=(min_gen_ran, len(os.listdir(alg_results_path))), fill_value=np.nan, dtype=float)
                for run_ind, run_name in enumerate(os.listdir(alg_results_path)):
                    run_path = os.path.join(alg_results_path, run_name)
                    best_costs_gen             = np.genfromtxt(fname=run_path, dtype=float, delimiter=',', usecols=(0), skip_header=1)
                    times_gen                  = np.genfromtxt(fname=run_path, dtype=float, delimiter=',', usecols=(1), skip_header=1)
                    ecr_two_norm_gen           = np.genfromtxt(fname=run_path, dtype=float, delimiter=',', usecols=(2), skip_header=1)
                    ecr_inf_norm_gen           = np.genfromtxt(fname=run_path, dtype=float, delimiter=',', usecols=(3), skip_header=1)
                    response_time_two_norm_gen = np.genfromtxt(fname=run_path, dtype=float, delimiter=',', usecols=(4), skip_header=1)
                    response_time_inf_norm_gen = np.genfromtxt(fname=run_path, dtype=float, delimiter=',', usecols=(5), skip_header=1)

                    if best_costs_gen.size == 1:
                        best_costs[             :, run_ind] = best_costs_gen
                        time_sec[               :, run_ind] = times_gen
                        ecr_two_norms[          :, run_ind] = ecr_two_norm_gen
                        ecr_inf_norms[          :, run_ind] = ecr_inf_norm_gen
                        response_time_two_norms[:, run_ind] = response_time_two_norm_gen
                        response_time_inf_norms[:, run_ind] = response_time_inf_norm_gen
                    else:
                        best_costs[             :, run_ind] = best_costs_gen[            :min_gen_ran]
                        time_sec[               :, run_ind] = times_gen[                 :min_gen_ran]
                        ecr_two_norms[          :, run_ind] = ecr_two_norm_gen[          :min_gen_ran]
                        ecr_inf_norms[          :, run_ind] = ecr_inf_norm_gen[          :min_gen_ran]
                        response_time_two_norms[:, run_ind] = response_time_two_norm_gen[:min_gen_ran]
                        response_time_inf_norms[:, run_ind] = response_time_inf_norm_gen[:min_gen_ran]
                # Save summaries off
                with open(result_base_dir + "/" + alg_name + "_by_gen.csv", 'w') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(["gen",
                                     "mean_time",
                                     "max_time",
                                     "min_time",
                                     "std_time",
                                     "avg_plus_three_std_time",
                                     "avg_minus_three_std_time",
                                     "mean_cost",
                                     "max_cost",
                                     "min_cost",
                                     "std_cost",
                                     "avg_plus_three_std_cost",
                                     "avg_minus_three_std_cost",
                                     "mean_ecr_two_norm",
                                     "max_ecr_two_norm",
                                     "min_ecr_two_norm",
                                     "std_ecr_two_norm",
                                     "avg_plus_three_std_ecr_two_norm",
                                     "avg_minus_three_std_ecr_two_norm",
                                     "mean_ecr_inf_norm",
                                     "max_ecr_inf_norm",
                                     "min_ecr_inf_norm",
                                     "std_ecr_inf_norm",
                                     "avg_plus_three_std_ecr_inf_norm",
                                     "avg_minus_three_std_ecr_inf_norm",
                                     "mean_rt_two_norm",
                                     "max_rt_two_norm",
                                     "min_rt_two_norm",
                                     "std_rt_two_norm",
                                     "avg_plus_three_std_rt_two_norm",
                                     "avg_minus_three_std_rt_two_norm",
                                     "mean_rt_inf_norm",
                                     "max_rt_inf_norm",
                                     "min_rt_inf_norm",
                                     "std_rt_inf_norm",
                                     "avg_plus_three_std_rt_inf_norm",
                                     "avg_minus_three_std_rt_inf_norm",
                                    ])
                    for gen_ind in range(min_gen_ran):
                        writer.writerow([gen_ind,
                                         np.average(time_sec[gen_ind, :]),
                                         np.max(time_sec[gen_ind, :]),
                                         np.min(time_sec[gen_ind, :]),
                                         np.std(time_sec[gen_ind, :]),
                                         np.average(time_sec[gen_ind, :]) + (3.0 * np.std(time_sec[gen_ind, :])),
                                         np.average(time_sec[gen_ind, :]) - (3.0 * np.std(time_sec[gen_ind, :])),
                                         np.average(best_costs[gen_ind, :]),
                                         np.max(best_costs[gen_ind, :]),
                                         np.min(best_costs[gen_ind, :]),
                                         np.std(best_costs[gen_ind, :]),
                                         np.average(best_costs[gen_ind, :]) + (3.0 * np.std(best_costs[gen_ind, :])),
                                         np.average(best_costs[gen_ind, :]) - (3.0 * np.std(best_costs[gen_ind, :])),
                                         np.average(ecr_two_norms[gen_ind, :]),
                                         np.max(ecr_two_norms[gen_ind, :]),
                                         np.min(ecr_two_norms[gen_ind, :]),
                                         np.std(ecr_two_norms[gen_ind, :]),
                                         np.average(ecr_two_norms[gen_ind, :]) + (3.0 * np.std(ecr_two_norms[gen_ind, :])),
                                         np.average(ecr_two_norms[gen_ind, :]) - (3.0 * np.std(ecr_two_norms[gen_ind, :])),
                                         np.average(ecr_inf_norms[gen_ind, :]),
                                         np.max(ecr_inf_norms[gen_ind, :]),
                                         np.min(ecr_inf_norms[gen_ind, :]),
                                         np.std(ecr_inf_norms[gen_ind, :]),
                                         np.average(ecr_inf_norms[gen_ind, :]) + (3.0 * np.std(ecr_inf_norms[gen_ind, :])),
                                         np.average(ecr_inf_norms[gen_ind, :]) - (3.0 * np.std(ecr_inf_norms[gen_ind, :])),
                                         np.average(response_time_two_norms[gen_ind, :]),
                                         np.max(response_time_two_norms[gen_ind, :]),
                                         np.min(response_time_two_norms[gen_ind, :]),
                                         np.std(response_time_two_norms[gen_ind, :]),
                                         np.average(response_time_two_norms[gen_ind, :]) + (3.0 * np.std(response_time_two_norms[gen_ind, :])),
                                         np.average(response_time_two_norms[gen_ind, :]) - (3.0 * np.std(response_time_two_norms[gen_ind, :])),
                                         np.average(response_time_inf_norms[gen_ind, :]),
                                         np.max(response_time_inf_norms[gen_ind, :]),
                                         np.min(response_time_inf_norms[gen_ind, :]),
                                         np.std(response_time_inf_norms[gen_ind, :]),
                                         np.average(response_time_inf_norms[gen_ind, :]) + (3.0 * np.std(response_time_inf_norms[gen_ind, :])),
                                         np.average(response_time_inf_norms[gen_ind, :]) - (3.0 * np.std(response_time_inf_norms[gen_ind, :])),
                                        ])




