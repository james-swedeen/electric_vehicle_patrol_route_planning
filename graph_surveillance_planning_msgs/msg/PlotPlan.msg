##
# @File: PlotPlan.msg
# @Date: March 2024
# @Author: James Swedeen
#
# @brief
# Used to send graph and plan information to the plotter.
#

## Graph
# Position of every node in the graph
float64[] node_position_x
float64[] node_position_y
float64[] node_position_z

# The node indexes that each directed edge connects
uint64[]                                from_node_ind
uint64[]                                to_node_ind
graph_surveillance_planning_msgs/Edge[] edge_paths

## Places of Interest
float64[] poi_position_x
float64[] poi_position_y
float64[] poi_radius
float64[] poi_max_expected_event_rate

## Response Time Points
float64[] rtp_position_x
float64[] rtp_position_y

## Plan
# Each of the following share the same set of indexes
float64[] time
# First index is agent, second is time
graph_surveillance_planning_msgs/AgentState[] agents_state
# First index is place of interest, second is time
graph_surveillance_planning_msgs/ExpectedEventRates[] expected_event_rates

## Metrics
graph_surveillance_planning_msgs/Metric[] metrics

