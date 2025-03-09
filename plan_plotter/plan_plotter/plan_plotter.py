"""plan_plotter.py: Send plans here to have them plotted."""

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from typing import Optional
from graph_surveillance_planning_msgs.msg import PlotPlan
import numpy as np
import math

import matplotlib.style as mplstyle
mplstyle.use('fast')

class PlanPlotter(Node):
    def __init__(self):
        super().__init__("plan_plotter")
        # Fixed parameters
        self.agent_colors = ['b', 'g', 'r', 'c', 'm', 'y']

        # Plotting variables
        self.figure     = plt.figure(figsize=plt.figaspect(0.5))
        self.network_ax = self.figure.add_subplot(2, 2, 1)
        self.charge_ax  = self.figure.add_subplot(2, 2, 2)
        self.graph_ax   = self.figure.add_subplot(2, 2, 3, projection='3d')
        self.metric_fig = self.figure.add_subfigure(self.figure.add_subplot(2,2,4).get_subplotspec().get_gridspec()[1,1])

        self.network_ax.set_title("Streets")
        self.network_ax.set_xlabel("x (km)")
        self.network_ax.set_ylabel("y (km)")
        self.graph_ax.set_title("Streets")
        self.graph_ax.set_xlabel("x (km)")
        self.graph_ax.set_ylabel("y (km)")
        self.graph_ax.set_zlabel("Elevation (km)")
        self.charge_ax.set_title("State Of Charge")
        self.charge_ax.set_ylabel("kWh")
        self.charge_ax.grid(True)

        self.static_plots        = []
        self.network_agents_plot = []
        self.network_poi_plot    = []
        self.agents_plot         = []
        self.charge_plot         = self.charge_ax.bar([],[])
        self.metric_sub_ax       = []

        # Parameters set by subscription
        self.cur_plan: Optional[PlotPlan] = None
        self.cur_plotting_index: int = -1 # Current time in plotting

        # Interfaces
        self.plan_sub = self.create_subscription(
            PlotPlan,
            '/plan',
            self.plan_to_plot_cb,
            10)

        self.plot_data_loop = self.create_timer(0.01, self.update_plot_data_cb)

    def plan_to_plot_cb(self, msg: PlotPlan):
        if self.cur_plan is not None:
            # Clear old plan
            for plot in self.static_plots:
                plot.remove()
            self.static_plots = []
            for agent_plot in self.network_agents_plot:
                agent_plot.remove()
            self.network_agents_plot = []
            for poi_plot in self.network_poi_plot:
                poi_plot.remove()
            self.network_poi_plot = []
            for agent_plot in self.agents_plot:
                agent_plot.remove()
            self.agents_plot = []
            self.charge_plot.remove()
            for metric_ind in range(len(self.cur_plan.metrics)):
                self.metric_sub_ax[metric_ind].remove()
            self.metric_sub_ax.clear()
        while len(msg.agents_state) > len(self.agent_colors):
            self.agent_colors = self.agent_colors + self.agent_colors

        # Plot first step
        self.cur_plan = msg
        self.cur_plotting_index = 0

        # Draw graph
        self.network_ax.set_xlim(left  = min(msg.node_position_x), right = max(msg.node_position_x))
        self.network_ax.set_ylim(bottom= min(msg.node_position_y), top   = max(msg.node_position_y))
        #self.network_ax.set_aspect(1)
        edge_width = 1.0
        if True:
            self.graph_ax.set_xlim3d(left  = min(msg.node_position_x), right = max(msg.node_position_x))
            self.graph_ax.set_ylim3d(bottom= min(msg.node_position_y), top   = max(msg.node_position_y))
            self.graph_ax.set_zlim3d(bottom= min(msg.node_position_z), top   = max(msg.node_position_z))
            self.graph_ax.set_box_aspect((np.ptp(msg.node_position_x), np.ptp(msg.node_position_y), np.ptp(msg.node_position_z)))
            for edge_it in range(len(msg.edge_paths)):
                self.static_plots.append(
                    self.graph_ax.plot(msg.edge_paths[edge_it].position_x,
                                       msg.edge_paths[edge_it].position_y,
                                       msg.edge_paths[edge_it].position_z,
                                       'k')[0])
        for edge_it in range(len(msg.edge_paths)):
            self.static_plots.append(
                self.network_ax.plot(msg.edge_paths[edge_it].position_x,
                                     msg.edge_paths[edge_it].position_y,
                                     'k', linewidth=edge_width, zorder=5)[0])
            end_unit_vec = np.matrix([msg.edge_paths[edge_it].position_x[-1] - msg.edge_paths[edge_it].position_x[-2],
                                      msg.edge_paths[edge_it].position_y[-1] - msg.edge_paths[edge_it].position_y[-2]]).transpose()
            end_unit_vec = end_unit_vec / np.linalg.norm(end_unit_vec)
            end_angle = np.arctan2(end_unit_vec[1], end_unit_vec[0])
            arrow_angle = end_angle+(7.0*np.pi/8.0)
            self.static_plots.append(self.network_ax.plot([msg.edge_paths[edge_it].position_x[-1], msg.edge_paths[edge_it].position_x[-1] + (0.01 * np.cos(arrow_angle)).item()],
                                                          [msg.edge_paths[edge_it].position_y[-1], msg.edge_paths[edge_it].position_y[-1] + (0.01 * np.sin(arrow_angle)).item()],
                                                          'k',
                                                          linewidth=edge_width,
                                                          zorder=5)[0])
            arrow_angle = end_angle-(7.0*np.pi/8.0)
            self.static_plots.append(self.network_ax.plot([msg.edge_paths[edge_it].position_x[-1], msg.edge_paths[edge_it].position_x[-1] + (0.01 * np.cos(arrow_angle)).item()],
                                                          [msg.edge_paths[edge_it].position_y[-1], msg.edge_paths[edge_it].position_y[-1] + (0.01 * np.sin(arrow_angle)).item()],
                                                          'k',
                                                          linewidth=edge_width,
                                                          zorder=5)[0])
        self.static_plots.append(self.network_ax.plot(msg.rtp_position_x, msg.rtp_position_y, "y.", zorder=0)[0])
        # Draw charge state
        charge_vec = [self.cur_plan.agents_state[i].state_of_charge[self.cur_plotting_index] for i in range(len(self.cur_plan.agents_state))]
        self.charge_plot = self.charge_ax.bar(list(map(chr, range(65, 65+len(self.cur_plan.agents_state)))),
                                              charge_vec,
                                              color=self.agent_colors[:len(self.cur_plan.agents_state)])
        # Draw agents
        for agent_ind in range(len(msg.agents_state)):
            self.agents_plot.append(self.graph_ax.plot([msg.agents_state[agent_ind].position_x[self.cur_plotting_index]],
                                                       [msg.agents_state[agent_ind].position_y[self.cur_plotting_index]],
                                                       [msg.agents_state[agent_ind].position_z[self.cur_plotting_index]],
                                                       "*", markersize=10, zorder=10, color=self.agent_colors[agent_ind])[0])
            self.network_agents_plot.append(self.network_ax.plot([msg.agents_state[agent_ind].position_x[self.cur_plotting_index]],
                                                                 [msg.agents_state[agent_ind].position_y[self.cur_plotting_index]],
                                                                 "*", markersize=10, zorder=10, color=self.agent_colors[agent_ind])[0])
        # Draw places of interest
        for poi_ind in range(len(msg.poi_radius)):
            angles = np.linspace(start=0, stop=np.pi, num=100)
            half_x = msg.poi_radius[poi_ind] * np.cos(angles)
            half_y = msg.poi_radius[poi_ind] * np.sin(angles)
            self.static_plots.append(self.network_ax.plot(np.append(half_x, half_x[::-1]) + msg.poi_position_x[poi_ind],
                                                          np.append(half_y, -half_y)      + msg.poi_position_y[poi_ind],
                                                          "r",
                                                          alpha=1.0,
                                                          zorder=1,
                                                          linewidth=edge_width)[0])
            self.network_poi_plot.append(self.network_ax.fill_between(x=  half_x + msg.poi_position_x[poi_ind],
                                                                      y1= half_y + msg.poi_position_y[poi_ind],
                                                                      y2=-half_y + msg.poi_position_y[poi_ind],
                                                                      alpha=msg.expected_event_rates[poi_ind].expected_event_rate[0] / msg.poi_max_expected_event_rate[poi_ind],
                                                                      color="r",
                                                                      zorder=1))

        # Draw metrics
        num_metric = len(msg.metrics)
        self.metric_sub_ax = []
        for metric_ind in range(num_metric):
            self.metric_sub_ax.append(self.metric_fig.add_subplot(num_metric, 1, metric_ind+1))
            self.metric_sub_ax[-1].set_xlim(msg.time[0], msg.time[-1])
            self.metric_sub_ax[-1].set_ylabel(msg.metrics[metric_ind].name)
            self.metric_sub_ax[-1].plot(msg.time[0], msg.metrics[metric_ind].values[0], "k")
            self.metric_sub_ax[-1].set_autoscaley_on(True)
        self.metric_sub_ax[0].set_title("Metrics")
        self.metric_sub_ax[-1].set_xlabel("Time (hr)")

    def update_plot_data_cb(self):
        if self.cur_plan is not None:
            self.cur_plotting_index += 1
            if self.cur_plotting_index == len(self.cur_plan.time):
                self.cur_plotting_index = 1
                for metric_ind in range(len(self.cur_plan.metrics)):
                    self.metric_sub_ax[metric_ind].clear()
                    self.metric_sub_ax[metric_ind].set_xlim(self.cur_plan.time[0], self.cur_plan.time[-1])
                    self.metric_sub_ax[metric_ind].set_ylabel(self.cur_plan.metrics[metric_ind].name)
                    self.metric_sub_ax[metric_ind].plot(self.cur_plan.time[0], self.cur_plan.metrics[metric_ind].values[0], "k")
                    self.metric_sub_ax[metric_ind].set_autoscaley_on(True)
            # Draw charge
            charge_vec = [self.cur_plan.agents_state[i].state_of_charge[self.cur_plotting_index] for i in range(len(self.cur_plan.agents_state))]
            for rect, height in zip(self.charge_plot, charge_vec):
                rect.set_height(height)
            # Draw metrics
            for metric_ind in range(len(self.cur_plan.metrics)):
                self.metric_sub_ax[metric_ind].plot([self.cur_plan.time[self.cur_plotting_index-1], self.cur_plan.time[self.cur_plotting_index]],
                                                    [self.cur_plan.metrics[metric_ind].values[self.cur_plotting_index-1], self.cur_plan.metrics[metric_ind].values[self.cur_plotting_index]],
                                                    "k")
            # Draw agents
            for agent_ind in range(len(self.cur_plan.agents_state)):
                self.agents_plot[agent_ind].set_data_3d([self.cur_plan.agents_state[agent_ind].position_x[self.cur_plotting_index]],
                                                        [self.cur_plan.agents_state[agent_ind].position_y[self.cur_plotting_index]],
                                                        [self.cur_plan.agents_state[agent_ind].position_z[self.cur_plotting_index]])
                self.network_agents_plot[agent_ind].set_data([self.cur_plan.agents_state[agent_ind].position_x[self.cur_plotting_index]],
                                                             [self.cur_plan.agents_state[agent_ind].position_y[self.cur_plotting_index]])
            # Draw places of interest
            for poi_ind in range(len(self.cur_plan.poi_radius)):
                self.network_poi_plot[poi_ind].set_alpha(self.cur_plan.expected_event_rates[poi_ind].expected_event_rate[self.cur_plotting_index] / self.cur_plan.poi_max_expected_event_rate[poi_ind])


def main(args=None):
    rclpy.init(args=args)

    plan_plotter = PlanPlotter()
    while rclpy.ok():
        rclpy.spin_once(plan_plotter)
        #plan_plotter.figure.canvas.draw_idle()
        #plan_plotter.figure.canvas.flush_events()
        plt.pause(0.01)

    plan_plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
