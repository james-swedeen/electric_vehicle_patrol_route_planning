##
# @File: graph_csv_gen.py
#

import osmnx as ox
import osmnx.truncate
import requests
import urllib
import matplotlib.pyplot as plt
import networkx as nx
import pandas as pd
import numpy as np
import math
import random
import time
import sys


ox.settings.cache_folder = "/tmp/osmnx_cache"
elevation_url = r'https://epqs.nationalmap.gov/v1/json?'


class physics_energy_model: # From: Power-based electric vehicle energy consumption model: Model development and validation By Chiara Fior
    def __init__(self):
        if False: # 2016 Nissan Leaf
            self.vehicle_mass         = 1521.0 # kg
            self.vehicle_front_area   = 2.3316 # m**2
            self.aero_drag_coef       = 0.28
            self.driveline_efficiency = 0.92
            self.motor_efficiency     = 0.91
            self.battery_efficiency   = 0.9
            self.auxiliary_power      = 700.0 # W
        else: # 2025 Chevrolet Blazer EV PPV
            self.vehicle_mass         = 2663.0 + (2.0 * 62.0) # kg
            self.vehicle_front_area   = 3.2524 # m**2
            self.aero_drag_coef       = 0.3
            self.driveline_efficiency = 1.0
            self.motor_efficiency     = 0.8
            self.battery_efficiency   = 0.95
            self.auxiliary_power      = 3000.0 # W
        self.grav_accel         = 9.8066 # m/s**2
        self.road_roll_res      = 1.75
        self.road_cond_roll_res = 0.0328
        self.tire_roll_res      = 4.575
        self.air_density        = 1.22563 # kg/m**3

    def get_consumed_kwh(self, avg_speed_mps: float, grade: float, distance_m: float) -> float:
        end_time = distance_m / avg_speed_mps

        roll_res_force = self.vehicle_mass * \
                         self.grav_accel * \
                         math.cos(grade) * \
                         self.road_roll_res * \
                         ((self.road_cond_roll_res * avg_speed_mps) + self.tire_roll_res) \
                         / 1000.0
        air_res_force = self.air_density * self.vehicle_front_area * self.aero_drag_coef * (avg_speed_mps**2.0) / 2.0
        grav_force = self.vehicle_mass * self.grav_accel * math.sin(grade)

        wheel_force = roll_res_force + air_res_force + grav_force
        if wheel_force >= 0.0:
            battery_power = wheel_force * avg_speed_mps / self.driveline_efficiency / self.motor_efficiency
        else: # Regenerative breaking
            accel = wheel_force / self.vehicle_mass
            break_eff = 1.0/(np.exp(0.0411/math.fabs(accel)))
            battery_power = break_eff * wheel_force * avg_speed_mps * self.driveline_efficiency * self.motor_efficiency

        at_batter_power = battery_power + self.auxiliary_power
        if at_batter_power >= 0.0:
            return (battery_power + self.auxiliary_power) * end_time / (60.0*60.0*1000.0*self.battery_efficiency)
        else:
            return self.battery_efficiency * (battery_power + self.auxiliary_power) * end_time / (60.0*60.0*1000.0)



def gen_street_csvs(center_point: tuple[float, float],
                    graph_name: str,
                    bound_box_dist: int,
                    max_node_dist: float = 1000000000.0,
                    plot: bool = False,
                    export_csvs: bool = True) -> None:
    """
    @Inputs
        center_point: latitude and longitude of home base
        TODO
    """

    # Find a graph
    G = ox.graph_from_point(center_point=center_point,
                            dist=bound_box_dist,
                            dist_type="bbox",
                            network_type="drive",
                            simplify=False,
                            retain_all=False,
                            truncate_by_edge=False)
    G = osmnx.truncate.largest_component(G=G, strongly=True)
    G = ox.distance.add_edge_lengths(G)
    G = ox.add_edge_speeds(G=G)
    if plot:
        ec = ox.plot.get_edge_colors_by_attr(G=G, attr="length", cmap="Reds")
        ox.plot_graph(G=G, show=False, edge_color=ec)
        plt.plot(center_point[0], center_point[1], "*b")
        plt.title("Original")

    # Make edges shorter
    while False:
        made_change = False
        for start_node,end_node,edge_info in list(G.edges(data=True)):
            if ('length' in edge_info) and (edge_info['length'] > max_node_dist):
                made_change = True
                speed_kph = edge_info['speed_kph']
                new_node = random.randint(0, sys.maxsize)
                while G.has_node(new_node):
                    new_node = random.randint(0, sys.maxint)
                new_node_x = (G.nodes[start_node]['x'] + G.nodes[end_node]['x']) / 2.0
                new_node_y = (G.nodes[start_node]['y'] + G.nodes[end_node]['y']) / 2.0
                G.remove_edge(start_node, end_node)
                G.add_node(new_node, x=new_node_x, y=new_node_y)
                G.add_edges_from([(start_node, new_node), (new_node, end_node)], speed_kph=speed_kph)
                if G.has_edge(end_node, start_node):
                    G.remove_edge(end_node, start_node)
                    G.add_edges_from([(end_node, new_node), (new_node, start_node)], speed_kph=speed_kph)
                break
        if made_change:
            G = ox.distance.add_edge_lengths(G)
        else:
            break

    # add root flag
    near_center_node_ind = ox.distance.nearest_nodes(G=G, Y=center_point[0], X=center_point[1])
    G.nodes[near_center_node_ind]["center_node"] = True
    if plot:
        ec = ox.plot.get_edge_colors_by_attr(G=G, attr="length", cmap="Reds")
        ox.plot_graph(G=G, show=False, edge_color=ec)
        plt.plot(center_point[0], center_point[1], "*b")
        plt.plot(float(G.nodes[near_center_node_ind]['x']), float(G.nodes[near_center_node_ind]['y']), "*g")
        plt.title("Shorter Edges")

    # Add elevation values
    for ind, node in enumerate(G.nodes()):
        if (ind % int(len(G.nodes)/100)) == 0:
            print(f"Getting Elevation Percent Complete: {(ind/len(G.nodes))*100.0:.2f}%")
        # define rest query params
        params = {
            'output': 'json',
            'y': G.nodes[node]['y'],
            'x': G.nodes[node]['x'],
            'units': 'Meters'
        }
        while True:
            try:
                # format query string and return query value
                result = requests.get((elevation_url + urllib.parse.urlencode(params)))
                nx.set_node_attributes(G, {node: {"elevation": float(result.json()['value'])}})
                break
            except Exception as e:
                print("Exception while getting elevation values: " + str(e))
                print("Retrying request: " + str(params))
    if plot:
        nc = ox.plot.get_node_colors_by_attr(G, "elevation", cmap="Blues")
        ox.plot_graph(G=G, show=False, edge_color=ec, node_color=nc)
        plt.title("Node Elevation")
    G = ox.elevation.add_edge_grades(G=G, add_absolute=False)

    # Project to a North East coordinate frame
    G = ox.projection.project_graph(G)
    if plot:
        ox.plot_graph(G=G, show=False, edge_color=ec, node_color=nc)
        plt.plot(float(G.nodes[near_center_node_ind]['x']), float(G.nodes[near_center_node_ind]['y']), "*g")
        plt.title("Projected")

    # Update distances with elevation into
    for start_node,end_node,edge_info in list(G.edges(data=True)):
        from_node = np.matrix([[G.nodes[start_node]['x']], [G.nodes[start_node]['y']], [G.nodes[start_node]['elevation']]], dtype=float)
        to_node = np.matrix([[G.nodes[end_node]['x']], [G.nodes[end_node]['y']], [G.nodes[end_node]['elevation']]], dtype=float)
        edge_info['length'] = np.linalg.norm(from_node - to_node)

    # Move the center node to be at the origin
    new_x = {}
    new_y = {}
    for node_ind in G.nodes():
        new_x[node_ind] = G.nodes[node_ind]['x'] - G.nodes[near_center_node_ind]['x']
        new_y[node_ind] = G.nodes[node_ind]['y'] - G.nodes[near_center_node_ind]['y']
    nx.set_node_attributes(G, new_x, 'x')
    nx.set_node_attributes(G, new_y, 'y')
    if plot:
        ox.plot_graph(G=G, show=False, edge_color=ec, node_color=nc)
        plt.plot(float(G.nodes[near_center_node_ind]['x']), float(G.nodes[near_center_node_ind]['y']), "*g")
        plt.title("Centered")

    # To CSV format
    old_node_ind_to_new = {}
    cur_ind = 1
    for node_ind in G.nodes():
        if near_center_node_ind == node_ind:
            old_node_ind_to_new[near_center_node_ind] = 0
        else:
            old_node_ind_to_new[node_ind] = cur_ind
            cur_ind += 1

    nodes_csv = pd.DataFrame(columns=["pos_x_m","pos_y_m","elevation"],                         dtype=float)
    edges_csv = pd.DataFrame(columns=["from_node","to_node","speed_lim_mps","power_usage_kwh"], dtype=float)

    for new_node_ind in range(len(old_node_ind_to_new)):
        old_node_ind = list(old_node_ind_to_new.keys())[list(old_node_ind_to_new.values()).index(new_node_ind)]

        pos_x_m   = G.nodes[old_node_ind]['x']
        pos_y_m   = G.nodes[old_node_ind]['y']
        elevation = G.nodes[old_node_ind]['elevation']

        nodes_csv = pd.concat([nodes_csv, pd.DataFrame([[pos_x_m,pos_y_m,elevation]], columns=nodes_csv.columns)], ignore_index=True)

    power_consumption_model = physics_energy_model()
    for from_node_ind_old, to_node_ind_old in G.edges():
        from_node       = old_node_ind_to_new[from_node_ind_old]
        to_node         = old_node_ind_to_new[to_node_ind_old]
        length_m        = G[from_node_ind_old][to_node_ind_old][0]['length']
        speed_lim_mps   = G[from_node_ind_old][to_node_ind_old][0]['speed_kph'] * (1000.0/(60.0*60.0))
        grade           = G[from_node_ind_old][to_node_ind_old][0]['grade']
        power_usage_kwh = power_consumption_model.get_consumed_kwh(avg_speed_mps=speed_lim_mps, grade=grade, distance_m=length_m)

        edges_csv = pd.concat([edges_csv, pd.DataFrame([[from_node,to_node,speed_lim_mps,power_usage_kwh]], columns=edges_csv.columns)], ignore_index=True)

    assert edges_csv.shape[0] == edges_csv.drop_duplicates(subset=["from_node","to_node"]).shape[0]

    # Export to csv
    if export_csvs:
        nodes_csv.to_csv(path_or_buf=graph_name+"_nodes.csv", index=False)
        edges_csv.to_csv(path_or_buf=graph_name+"_edges.csv", index=False)

if __name__=="__main__":
    base_dir = "~/ros_ws/src/graph_surveillance_planning/street_graph/config/"

    #gen_street_csvs(center_point=(41.736840,-111.836470), graph_name=base_dir+"logan_500",        bound_box_dist=500,  plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(41.736840,-111.836470), graph_name=base_dir+"logan_1000",       bound_box_dist=1000, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(41.736840,-111.836470), graph_name=base_dir+"logan_1500",       bound_box_dist=1500, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(34.052189,-118.243927), graph_name=base_dir+"los_angeles_500",  bound_box_dist=500,  plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(34.052189,-118.243927), graph_name=base_dir+"los_angeles_1000", bound_box_dist=1000, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(34.052189,-118.243927), graph_name=base_dir+"los_angeles_2500", bound_box_dist=2500, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(34.052189,-118.243927), graph_name=base_dir+"los_angeles_3000", bound_box_dist=3000, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(34.052189,-118.243927), graph_name=base_dir+"los_angeles_3500", bound_box_dist=3500, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(34.052189,-118.243927), graph_name=base_dir+"los_angeles_4000", bound_box_dist=4000, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(34.052189,-118.243927), graph_name=base_dir+"los_angeles_4500", bound_box_dist=4500, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(34.052189,-118.243927), graph_name=base_dir+"los_angeles_5000", bound_box_dist=5000, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(40.835468,-72.609749),  graph_name=base_dir+"new_york_2500",    bound_box_dist=2500, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(40.835468,-72.609749),  graph_name=base_dir+"new_york_3000",    bound_box_dist=3000, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(40.835468,-72.609749),  graph_name=base_dir+"new_york_4000",    bound_box_dist=4000, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(40.835468,-72.609749),  graph_name=base_dir+"new_york_5000",    bound_box_dist=5000, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(40.835468,-72.609749),  graph_name=base_dir+"new_york_6000",    bound_box_dist=6000, plot=False, export_csvs=True)
    gen_street_csvs(center_point=(40.835468,-72.609749),  graph_name=base_dir+"new_york_15000",    bound_box_dist=15000, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(47.604207,-122.329187), graph_name=base_dir+"seattle_500",      bound_box_dist=500,  plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(47.604207,-122.329187), graph_name=base_dir+"seattle_2500",     bound_box_dist=2500, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(47.604207,-122.329187), graph_name=base_dir+"seattle_3500",     bound_box_dist=3500, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(47.604207,-122.329187), graph_name=base_dir+"seattle_4500",     bound_box_dist=4500, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(47.604207,-122.329187), graph_name=base_dir+"seattle_5500",     bound_box_dist=5500, plot=False, export_csvs=True)
    #gen_street_csvs(center_point=(47.604207,-122.329187), graph_name=base_dir+"seattle_6500",     bound_box_dist=6500, plot=False, export_csvs=True)

    plt.show()
