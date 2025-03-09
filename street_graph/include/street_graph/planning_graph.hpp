/**
 * @File: planning_graph.hpp
 * @Date: May 2024
 * @Author: James Swedeen
 *
 * @brief
 * This will be a graph where the zero vertex is the depot and all other vertices are hotspots. Edges are defined as
 * paths between each vertex.
 **/

#ifndef STREET_GRAPH_PLANNING_GRAPH_HPP
#define STREET_GRAPH_PLANNING_GRAPH_HPP

/* C++ Headers */
#include<utility>
#include<random>
#include<memory>
#include<vector>
#include<list>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<street_graph/graph.hpp>

namespace graph
{
/**
 * @Vertex Test Functions
 *
 * @parameters
 * graph_index: The index of this vertex in the graph this vertex is part of
 **/
inline bool     isDepot(        const uint32_t vertex_index) noexcept;
inline bool     isHotspot(      const uint32_t vertex_index) noexcept;
inline uint32_t getHotspotIndex(const uint32_t vertex_index) noexcept;
/**
 * @Path
 *
 * @brief
 * A path connects two vertices.
 **/
class Path
{
public:
  /**
   * @Default Constructor
   **/
  Path() noexcept = default;
  /**
   * @Copy Constructor
   **/
  Path(const Path&) noexcept = default;
  /**
   * @Move Constructor
   **/
  Path(Path&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor initializes the class for use.
   *
   * @parameters
   * from_vertex: The index of the vertex this path starts at
   * to_vertex: The index of the vertex this path ends at
   * edges: The edges this path is made up of
   * traversal_time: The amount of time it takes to traverse this path
   * traversal_charge: The amount of charge used when this path is traversed in kWh
   * traversal_charge_needed: The amount of charge necessary to traverse this path in kWh
   * end_traversal_charge: The amount of charge used after the minimum charge usage point in the path in kWh
   * path_length: The length of the path
   **/
  inline Path(const uint32_t                  from_vertex,
              const uint32_t                  to_vertex,
              const std::vector<const Edge*>& edges,
              const float                     traversal_time,
              const float                     traversal_charge,
              const float                     traversal_charge_needed,
              const float                     end_traversal_charge,
              const float                     path_length) noexcept;
  /**
   * @Deconstructor
   **/
  ~Path() noexcept = default;
  /**
   * @Assignment Operators
   **/
  Path& operator=(const Path&)  noexcept = default;
  Path& operator=(      Path&&) noexcept = default;
  /**
   * @stateAtTime
   *
   * @brief
   * Used to retrieve the location of a vehicle after a set about of time has passed since entering this path.
   *
   * @parameters
   * time_spent: Time spent since entering this path
   *
   * @return
   * The state at that time.
   **/
  inline Eigen::Matrix<float,3,1> stateAtTime(const float time_spent) const noexcept;
  /**
   * @chargeUsedBetweenTimes
   *
   * @brief
   * Used to retrieve the amount of charge used by the vehicle between two time points along the path.
   *
   * @parameters
   * start_time: Time spent since entering this path to start counting charge
   * end_time: Time spent since entering this path to stop counting charge
   *
   * @return
   * The charge used in that time.
   **/
  inline float chargeUsedBetweenTimes(const float start_time, const float end_time) const noexcept;
  /**
   * @distanceTraversedBetweenTimes
   *
   * @brief
   * Used to retrieve the distance traversed by the vehicle between two time points along the path.
   *
   * @parameters
   * start_time: Time spent since entering this path to start counting charge
   * end_time: Time spent since entering this path to stop counting charge
   *
   * @return
   * The distance traversed in that time.
   **/
  inline float distanceTraversedBetweenTimes(const float start_time, const float end_time) const noexcept;
  /**
   * @generateStraightSegments
   *
   * @brief
   * Used to generate a set of straight line segments that span this path.
   *
   * @parameters
   * points: Is filled with the points that start and end each straight segment
   * time_points: The time since entering the path that each point is reached
   **/
  inline void generateStraightSegments(Eigen::Matrix<float,3,Eigen::Dynamic>& points,
                                       Eigen::Matrix<float,Eigen::Dynamic,1>& time_points) const noexcept;
  /**
   * @edgeInUseAtTime
   *
   * @brief
   * Finds the original graph edge that is in use at a set time after entering this path.
   *
   * @parameters
   * time_spent: Time spent since entering this path
   *
   * @return
   * The original graph edge that is in use at this time and the start time of the edge relative to the start time
   * of this path.
   **/
  inline std::pair<const Edge*,float> edgeInUseAtTime(const float time_spent) const noexcept;
  /**
   * @edgeInUseAtTimePlus
   *
   * @brief
   * Finds the original graph edge that is in use at a set time after entering this path.
   *
   * @parameters
   * time_spent: Time spent since entering this path
   *
   * @return
   * 0: The original graph edge that is in use at this time
   * 1: The start time of the edge relative to the start time of this path
   * 2: The charge used to get to the start of this edge
   * 3: The distance traversed to get to the start of this edge
   **/
  inline std::tuple<const Edge*,float,float,float> edgeInUseAtTimePlus(const float time_spent) const noexcept;
  /**
   * @Getters
   **/
  inline const std::vector<const Edge*>& cgetSubEdges() const noexcept;
private:
  std::vector<const Edge*> m_edges;
  // The time sense entering the path that each sub-edge starts
  Eigen::Matrix<double,Eigen::Dynamic,1> m_edge_start_times;
  // The charge used sense entering the path that each sub-edge starts
  Eigen::Matrix<double,Eigen::Dynamic,1> m_edge_start_charges;
  // The distance traversed sense entering the path that each sub-edge starts
  Eigen::Matrix<double,Eigen::Dynamic,1> m_edge_start_distances;
public:
  /**
   * @Class Parameters
   **/
  uint32_t from_vertex;
  uint32_t to_vertex;
  float    traversal_time;
  float    traversal_charge;
  float    traversal_charge_needed;
  float    end_traversal_charge;
  float    length;
};
/**
 * @PlanningGraph
 *
 * @brief
 * Contains information about the vertices and paths that are being planned over.
 **/
class PlanningGraph
{
public:
  /**
   * @Default Constructor
   **/
  PlanningGraph() = delete;
  /**
   * @Copy Constructor
   **/
  PlanningGraph(const PlanningGraph&) = delete;
  /**
   * @Move Constructor
   **/
  PlanningGraph(PlanningGraph&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * This constructor initializes the class for use.
   *
   * @parameters
   * street_graph: The street graph for this problem
   * hotspot_locations: Locations of the hotspots in terms of the index of the original node
   * agent_max_soc: The max charge level an agent can have
   * max_time: The max length of time that planning is possible for
   * max_trip_length: The max length of path that planning is possible for
   * using_distance_constraints: True to use distance=based constraints instead of SOC-based ones
   **/
  PlanningGraph(const Graph&                 street_graph,
                const std::vector<uint32_t>& hotspot_locations,
                const float                  agent_max_soc,
                const float                  max_time,
                const float                  max_trip_length,
                const bool                   using_soc_constraints,
                const bool                   using_distance_constraints) noexcept;
  /**
   * @Deconstructor
   **/
  ~PlanningGraph() noexcept = default;
  /**
   * @Assignment Operators
   **/
  PlanningGraph& operator=(const PlanningGraph&)  = delete;
  PlanningGraph& operator=(      PlanningGraph&&) = delete;
  /**
   * @Getters
   **/
  inline const Eigen::Matrix<std::vector<Path>,Eigen::Dynamic,Eigen::Dynamic>& cgetPointToPointPaths() const noexcept;
  /**
   * @Query Functions
   **/
  inline uint32_t numberVertices() const noexcept;
  inline uint32_t numberHotspots() const noexcept;
  inline uint32_t numberPaths()    const noexcept;
  /**
   * @minTravelTimePath
   *
   * @brief
   * Finds the minimal travel time path between to nodes in this graph.
   *
   * @parameters
   * from_node_ind: The index of the node that the travel starts at in this graph
   * to_node_ind: The index of the node that the travel ends at in this graph
   *
   * @return
   * The minimal path.
   **/
  inline const Path& minTravelTimePath(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept;
  /**
   * @minChargePath
   *
   * @brief
   * Finds the minimal charge used path between to nodes in this graph.
   *
   * @parameters
   * from_node_ind: The index of the node that the travel starts at in this graph
   * to_node_ind: The index of the node that the travel ends at in this graph
   *
   * @return
   * The minimal path.
   **/
  inline const Path& minChargePath(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept;
  /**
   * @minChargeNecessaryPath
   *
   * @brief
   * Finds the minimal charge needed path between to nodes in this graph.
   *
   * @parameters
   * from_node_ind: The index of the node that the travel starts at in this graph
   * to_node_ind: The index of the node that the travel ends at in this graph
   *
   * @return
   * The minimal path.
   **/
  inline const Path& minChargeNecessaryPath(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept;
  /**
   * @minEndChargePath
   *
   * @brief
   * Finds the minimal end charge used path between to nodes in this graph.
   *
   * @parameters
   * from_node_ind: The index of the node that the travel starts at in this graph
   * to_node_ind: The index of the node that the travel ends at in this graph
   *
   * @return
   * The minimal path.
   **/
  inline const Path& minEndChargePath(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept;
  /**
   * @minLengthPath
   *
   * @brief
   * Finds the minimal length path between to nodes in this graph.
   *
   * @parameters
   * from_node_ind: The index of the node that the travel starts at in this graph
   * to_node_ind: The index of the node that the travel ends at in this graph
   *
   * @return
   * The minimal path.
   **/
  inline const Path& minLengthPath(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept;
private:
  // Rows indicate the start vertex and columns are the target vertex
  Eigen::Matrix<std::vector<Path>,Eigen::Dynamic,Eigen::Dynamic> m_ptp_paths;
  Eigen::Matrix<Path,             Eigen::Dynamic,Eigen::Dynamic> m_min_time_ptp_paths;
  Eigen::Matrix<Path,             Eigen::Dynamic,Eigen::Dynamic> m_min_charge_ptp_paths;
  Eigen::Matrix<Path,             Eigen::Dynamic,Eigen::Dynamic> m_min_charge_needed_ptp_paths;
  Eigen::Matrix<Path,             Eigen::Dynamic,Eigen::Dynamic> m_min_end_charge_ptp_paths;
  Eigen::Matrix<Path,             Eigen::Dynamic,Eigen::Dynamic> m_min_length_ptp_paths;
  uint32_t                                                       m_number_vertices;
  uint32_t                                                       m_number_paths;
};
} // graph


inline bool graph::isDepot(const uint32_t vertex_index) noexcept
{
  return 0 == vertex_index;
}

inline bool graph::isHotspot(const uint32_t vertex_index) noexcept
{
  return 1 != vertex_index;
}

inline uint32_t graph::getHotspotIndex(const uint32_t vertex_index) noexcept
{
  return vertex_index - 1;
}

inline graph::Path::Path(const uint32_t                  from_vertex,
                         const uint32_t                  to_vertex,
                         const std::vector<const Edge*>& edges,
                         const float                     traversal_time,
                         const float                     traversal_charge,
                         const float                     traversal_charge_needed,
                         const float                     end_traversal_charge,
                         const float                     path_length) noexcept
 : m_edges(edges),
   from_vertex(from_vertex),
   to_vertex(to_vertex),
   traversal_time(traversal_time),
   traversal_charge(traversal_charge),
   traversal_charge_needed(traversal_charge_needed),
   end_traversal_charge(end_traversal_charge),
   length(path_length)
{
  // Test for path correctness
  assert(traversal_time >= -1e-2);
  assert(traversal_charge_needed >= -1e-2);
  assert(traversal_time < std::numeric_limits<double>::infinity());
  assert(traversal_charge_needed < std::numeric_limits<double>::infinity());
  assert(traversal_charge_needed >= traversal_charge);
  assert(path_length >= -1e-2);
  #ifndef NDEBUG
  const auto edges_end = edges.cend();
  for(auto edge_it = std::next(edges.cbegin()); edge_it != edges_end; ++edge_it)
  {
    assert((*std::prev(edge_it))->cgetToNode() == (*edge_it)->cgetFromNode());
  }
  #endif

  this->m_edge_start_times.    resize(this->m_edges.size());
  this->m_edge_start_charges.  resize(this->m_edges.size());
  this->m_edge_start_distances.resize(this->m_edges.size());
  this->m_edge_start_times[    0] = 0;
  this->m_edge_start_charges[  0] = 0;
  this->m_edge_start_distances[0] = 0;
  for(uint32_t edge_ind = 1; edge_ind < this->m_edges.size(); ++edge_ind)
  {
    this->m_edge_start_times[    edge_ind] = this->m_edge_start_times[edge_ind-1] +
                                             this->cgetSubEdges()[    edge_ind-1]->cgetMinTraversalTime();
    this->m_edge_start_charges[  edge_ind] = this->m_edge_start_charges[edge_ind-1] +
                                             this->cgetSubEdges()[    edge_ind-1]->cgetTraversalCharge();
    this->m_edge_start_distances[edge_ind] = this->m_edge_start_distances[edge_ind-1] +
                                             this->cgetSubEdges()[    edge_ind-1]->cgetLength();
  }
}

inline Eigen::Matrix<float,3,1> graph::Path::stateAtTime(const float time_spent) const noexcept
{
  assert(time_spent >= -1e-2);
  assert((time_spent - this->traversal_time) < 1e-2);

  const uint32_t active_edge_ind = (this->m_edge_start_times.array() <= std::max<float>(0, time_spent)).count() - 1;

  return this->cgetSubEdges()[active_edge_ind]->stateAtTime(time_spent - this->m_edge_start_times[active_edge_ind]);
}

inline float graph::Path::chargeUsedBetweenTimes(const float start_time, const float end_time) const noexcept
{
  assert(start_time >= -1e-2);
  assert((end_time - this->traversal_time) < 1e-2);
  assert(start_time <= end_time);

  const uint32_t start_active_edge_ind = (this->m_edge_start_times.array() <= std::max<float>(0, start_time)).count() - 1;
  const uint32_t end_active_edge_ind   = (this->m_edge_start_times.array() <= end_time).                      count() - 1;

  float output = this->cgetSubEdges()[start_active_edge_ind]->chargeUsedBetweenTimes(
                   start_time - this->m_edge_start_times[start_active_edge_ind],
                   std::min<float>(end_time - this->m_edge_start_times[start_active_edge_ind], this->cgetSubEdges()[start_active_edge_ind]->cgetMinTraversalTime()));
  if(start_active_edge_ind != end_active_edge_ind)
  {
    for(uint32_t edge_ind = start_active_edge_ind+1; edge_ind < end_active_edge_ind; ++edge_ind)
    {
      output += this->cgetSubEdges()[edge_ind]->cgetTraversalCharge();
    }
    output += this->cgetSubEdges()[end_active_edge_ind]->chargeUsedAtTime(end_time - this->m_edge_start_times[end_active_edge_ind]);
  }

  return output;
}

inline float graph::Path::distanceTraversedBetweenTimes(const float start_time, const float end_time) const noexcept
{
  assert(start_time >= -1e-2);
  assert((end_time - this->traversal_time) < 1e-2);
  assert(start_time <= end_time);

  const uint32_t start_active_edge_ind = (this->m_edge_start_times.array() <= std::max<float>(0, start_time)).count() - 1;
  const uint32_t end_active_edge_ind   = (this->m_edge_start_times.array() <= end_time).                      count() - 1;

  float output = this->cgetSubEdges()[start_active_edge_ind]->distanceTraversedBetweenTimes(
                    start_time - this->m_edge_start_times[start_active_edge_ind],
                    std::min<float>(end_time - this->m_edge_start_times[start_active_edge_ind], this->cgetSubEdges()[start_active_edge_ind]->cgetMinTraversalTime()));
  if(start_active_edge_ind != end_active_edge_ind)
  {
    for(uint32_t edge_ind = start_active_edge_ind+1; edge_ind < end_active_edge_ind; ++edge_ind)
    {
      output += this->cgetSubEdges()[edge_ind]->cgetLength();
    }
    output += this->cgetSubEdges()[end_active_edge_ind]->distanceTraversedBetweenTimes(0, end_time - this->m_edge_start_times[end_active_edge_ind]);
  }

  return output;
}

inline void graph::Path::generateStraightSegments(Eigen::Matrix<float,3,Eigen::Dynamic>& points,
                                                  Eigen::Matrix<float,Eigen::Dynamic,1>& time_points) const noexcept
{
  points.     resize(3, this->m_edges.size() + 1);
  time_points.resize(this->m_edges.size() + 1, 1);

  points.leftCols<1>() = this->cgetSubEdges().front()->cgetFromNode()->cgetPosition();
  time_points[0]       = 0;
  for(uint32_t point_ind = 0; point_ind < this->m_edges.size(); ++point_ind)
  {
    points.col( point_ind + 1) = this->cgetSubEdges()[point_ind]->cgetToNode()->cgetPosition();
    time_points[point_ind + 1] = time_points[point_ind] + this->cgetSubEdges()[point_ind]->cgetMinTraversalTime();
  }
}

inline std::pair<const graph::Edge*,float> graph::Path::edgeInUseAtTime(const float time_spent) const noexcept
{
  assert(time_spent >= -1e-2);
  assert((time_spent - this->traversal_time) < 1e-2);

  const uint32_t active_edge_ind = (this->m_edge_start_times.array() <= std::max<float>(0, time_spent)).count() - 1;

  return std::pair<const Edge*,float>(this->cgetSubEdges()[active_edge_ind], this->m_edge_start_times[active_edge_ind]);
}

inline std::tuple<const graph::Edge*,float,float,float> graph::Path::edgeInUseAtTimePlus(const float time_spent) const noexcept
{
  assert(time_spent >= -1e-2);
  assert((time_spent - this->traversal_time) < 1e-2);

  const uint32_t active_edge_ind = (this->m_edge_start_times.array() <= std::max<float>(0, time_spent)).count() - 1;

  return std::tuple<const Edge*,float,float,float>(this->cgetSubEdges()[active_edge_ind],
                                                   this->m_edge_start_times[active_edge_ind],
                                                   this->m_edge_start_charges[active_edge_ind],
                                                   this->m_edge_start_distances[active_edge_ind]);
}

inline const std::vector<const graph::Edge*>& graph::Path::cgetSubEdges() const noexcept
{
  return this->m_edges;
}

inline const Eigen::Matrix<std::vector<graph::Path>,Eigen::Dynamic,Eigen::Dynamic>& graph::PlanningGraph::cgetPointToPointPaths() const noexcept
{
  return this->m_ptp_paths;
}

inline uint32_t graph::PlanningGraph::numberVertices() const noexcept
{
  return this->m_number_vertices;
}

inline uint32_t graph::PlanningGraph::numberHotspots() const noexcept
{
  return this->numberVertices() - 1;
}

inline uint32_t graph::PlanningGraph::numberPaths() const noexcept
{
  return this->m_number_paths;
}

inline const graph::Path& graph::PlanningGraph::minTravelTimePath(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept
{
  return this->m_min_time_ptp_paths(from_node_ind, to_node_ind);
}

inline const graph::Path& graph::PlanningGraph::minChargePath(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept
{
  return this->m_min_charge_ptp_paths(from_node_ind, to_node_ind);
}

inline const graph::Path& graph::PlanningGraph::minChargeNecessaryPath(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept
{
  return this->m_min_charge_needed_ptp_paths(from_node_ind, to_node_ind);
}

inline const graph::Path& graph::PlanningGraph::minEndChargePath(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept
{
  return this->m_min_end_charge_ptp_paths(from_node_ind, to_node_ind);
}

inline const graph::Path& graph::PlanningGraph::minLengthPath(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept
{
  return this->m_min_length_ptp_paths(from_node_ind, to_node_ind);
}

#endif
/* planning_graph.hpp */
