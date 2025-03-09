/**
 * @File: graph.hpp
 * @Date: March 2024
 * @Author: James Swedeen
 *
 * @brief
 * Defines a directed street graph.
 **/

#ifndef STREET_GRAPH_GRAPH_HPP
#define STREET_GRAPH_GRAPH_HPP

/* C++ Headers */
#include<string>
#include<forward_list>
#include<list>
#include<memory>
#include<utility>
#include<algorithm>
#include<execution>

/* Local Headers */
#include<street_graph/nodes_edges.hpp>

namespace graph
{
class Graph
{
public:
  /**
   * @Default Constructor
   **/
  Graph() = delete;
  /**
   * @Copy Constructor
   **/
  Graph(const Graph&) = delete;
  /**
   * @Move Constructor
   **/
  Graph(Graph&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * nodes_file_path: An absolute path to the nodes CSV file
   * edges_file_path: An absolute path to the edges CSV file
   **/
  Graph(const std::string& nodes_file_path, const std::string& edges_file_path, const double max_area_for_straight = 10, const bool need_min_paths = true);
  /**
   * @Deconstructor
   **/
  ~Graph() noexcept = default;
  /**
   * @Assignment Operators
   **/
  Graph& operator=(const Graph&)  = delete;
  Graph& operator=(      Graph&&) = delete;
  /**
   * @Get Functions
   **/
  inline const std::vector<std::unique_ptr<Node>>&     cgetOriginalNodes()      const noexcept;
  inline const std::vector<std::unique_ptr<Edge>>&     cgetOriginalEdges()      const noexcept;
  inline const std::vector<std::unique_ptr<Node>>&     cgetSimplifiedNodes()    const noexcept;
  inline const std::vector<std::unique_ptr<EdgeBase>>& cgetSimplifiedEdges()    const noexcept;
  inline const Node&                                   cgetRootNode()           const noexcept;
  inline const Node&                                   cgetSimplifiedRootNode() const noexcept;
  /**
   * @Query Functions
   **/
  inline uint32_t numberOriginalNodes()   const noexcept;
  inline uint32_t numberOriginalEdges()   const noexcept;
  inline uint32_t numberSimplifiedNodes() const noexcept;
  inline uint32_t numberSimplifiedEdges() const noexcept;
  /**
   * @findEdgesBetweenSimpleNodes
   *
   * @brief
   * Finds the set of simplified edges that start at from_node and end at to_node.
   *
   * @parameters
   * from_node_ind: The index of the node that the edges starts at in the simplified graph
   * to_node_ind: The index of the node that the edges ends at in the simplified graph
   *
   * @return
   * A set of simplified edges that start at from_node and end at to_node.
   **/
  std::vector<const EdgeBase*> findEdgesBetweenSimpleNodes(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept;
  /**
   * @boundingBoxOriginal
   *
   * @brief
   * Finds a bounding box that contains all of this graph's original nodes and edges.
   *
   * @return
   * The min and max on the bounding box in each dimension.
   **/
  Eigen::Matrix<float,3,2> boundingBoxOriginal() const noexcept;
  /**
   * @boundingBoxSimplified
   *
   * @brief
   * Finds a bounding box that contains all of this graph's simple nodes and edges.
   *
   * @return
   * The min and max on the bounding box in each dimension.
   **/
  Eigen::Matrix<float,3,2> boundingBoxSimplified() const noexcept;
  /**
   * @totalEdgeLength
   *
   * @brief
   * Calculates the summed total length of all edges.
   *
   * @return
   * The summed total length of all edges.
   **/
  double totalEdgeLength() const noexcept;
  /**
   * @totalStreetLength
   *
   * @brief
   * Calculates the summed total length of all streets.
   *
   * @return
   * The summed total length of all streets.
   **/
  double totalStreetLength() const noexcept;
  /**
   * @minTravelTime
   *
   * @brief
   * Finds the minimal travel time between to nodes in the original graph.
   *
   * @parameters
   * from_node_ind: The index of the node that the travel starts at in the original graph
   * to_node_ind: The index of the node that the travel ends at in the original graph
   *
   * @return
   * The minimal travel time (first),
   * the charge used along the minimal travel time path (second),
   * the charge needed to follow that path (third),
   * the length of the path (fourth)
   **/
  inline std::tuple<float,float,float,float> minTravelTimeFull(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept;
  inline float                               minTravelTime(    const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept;
  /**
   * @minCharge
   *
   * @brief
   * Finds the minimal charge used between to nodes in the original graph.
   *
   * @parameters
   * from_node_ind: The index of the node that the travel starts at in the original graph
   * to_node_ind: The index of the node that the travel ends at in the original graph
   *
   * @return
   * The traversal time of the minimal charge path (first),
   * the minimal charge use (second),
   * the charge needed to follow that path (third),
   * the length of the path (fourth)
   **/
  inline std::tuple<float,float,float,float> minChargeFull(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept;
  inline float                               minCharge(    const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept;
  /**
   * @minChargeNecessary
   *
   * @brief
   * Finds the minimal charge necessary to get between to nodes in the original graph.
   *
   * @parameters
   * from_node_ind: The index of the node that the travel starts at in the original graph
   * to_node_ind: The index of the node that the travel ends at in the original graph
   *
   * @return
   * The traversal time of the minimal charge needed path (first),
   * the minimal charge use (second),
   * the minimal charge needed to get between the two nodes (third),
   * the length of the path (fourth)
   **/
  inline std::tuple<float,float,float,float> minChargeNecessaryFull(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept;
  inline float                               minChargeNecessary(    const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept;
  /**
   * @minLength
   *
   * @brief
   * Finds the minimal traversal length necessary to get between to nodes in the original graph.
   *
   * @parameters
   * from_node_ind: The index of the node that the travel starts at in the original graph
   * to_node_ind: The index of the node that the travel ends at in the original graph
   *
   * @return
   * The traversal time of the minimal length path (first),
   * the minimal charge use (second),
   * the minimal charge needed to get between the two nodes (third),
   * the length of the path (fourth)
   **/
  inline std::tuple<float,float,float,float> minLengthFull(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept;
  inline float                               minLength(    const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept;
  /**
   * @edgesToFile
   *
   * @brief
   * Exports all simplified edges each to their own edge file in the specified directory.
   *
   * @parameters
   * base_dir: The directory that all of the edge files will be exported to.
   **/
  void edgesToFile(const std::string& base_dir, const double max_area_for_straight) const;
private:
  // Original graph
  std::vector<std::unique_ptr<Node>> m_nodes;
  std::vector<std::unique_ptr<Edge>> m_edges;
  // Simplified graph
  std::vector<      std::unique_ptr<Node>>     m_simple_nodes;
  std::vector<      std::unique_ptr<EdgeBase>> m_simple_edges;
  std::forward_list<std::unique_ptr<Node>>     m_skipped_nodes; // Only used for memory management
  // Minimal paths between simplified nodes
  struct MinimalPath
  {
  public:
    float min_traversal_time; // The minimal traversal time possible
    float min_charge_used;    // The minimal amount of charge used possible
    float min_charge_needed;  // The minimal amount of charge needed to get between two nodes
    float min_length;         // The minimal path length needed to get between two nodes

    float charge_used_on_min_time_path; // The charge used along the minimal time path
    float charge_needed_for_min_time;   // The starting charge necessary to follow the minimal time path
    float length_of_min_time_path;      // The path length of the minimal time path

    float time_of_min_charge_path;      // The time used along the minimal charge path
    float charge_needed_for_min_charge; // The starting charge necessary to follow the minimal charge path
    float length_of_min_charge_path;    // The path length of the minimal charge path

    float time_of_min_charge_needed;         // The time used along the minimal charge needed path
    float charge_used_for_min_charge_needed; // The charge used while following the minimal charge needed path
    float length_of_min_charge_needed_path;  // The path length of the minimal charge needed path

    float time_of_min_length_path;        // The path time of the minimal length path
    float charge_used_on_min_length_path; // The charge used along the minimal length path
    float charge_needed_for_min_length;   // The starting charge necessary to follow the minimal length path
  };
  // Rows indicate the start location and columns are the target location
  Eigen::Matrix<MinimalPath,Eigen::Dynamic,Eigen::Dynamic> m_minimal_paths;
};


inline const std::vector<std::unique_ptr<Node>>& Graph::cgetOriginalNodes() const noexcept
{
  return this->m_nodes;
}

inline const std::vector<std::unique_ptr<Edge>>& Graph::cgetOriginalEdges() const noexcept
{
  return this->m_edges;
}

inline const std::vector<std::unique_ptr<Node>>& Graph::cgetSimplifiedNodes() const noexcept
{
  return this->m_simple_nodes;
}

inline const std::vector<std::unique_ptr<EdgeBase>>& Graph::cgetSimplifiedEdges() const noexcept
{
  return this->m_simple_edges;
}

inline const Node& Graph::cgetRootNode() const noexcept
{
  return *this->cgetOriginalNodes().front();
}

inline const Node& Graph::cgetSimplifiedRootNode() const noexcept
{
  return *this->cgetSimplifiedNodes().front();
}

inline uint32_t Graph::numberOriginalNodes() const noexcept
{
  return this->cgetOriginalNodes().size();
}

inline uint32_t Graph::numberOriginalEdges() const noexcept
{
  return this->cgetOriginalEdges().size();
}

inline uint32_t Graph::numberSimplifiedNodes() const noexcept
{
  return this->cgetSimplifiedNodes().size();
}

inline uint32_t Graph::numberSimplifiedEdges() const noexcept
{
  return this->cgetSimplifiedEdges().size();
}

inline std::tuple<float,float,float,float> Graph::minTravelTimeFull(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept
{
  return std::tuple<float,float,float,float>(this->m_minimal_paths(from_node_ind, to_node_ind).min_traversal_time,
                                             this->m_minimal_paths(from_node_ind, to_node_ind).charge_used_on_min_time_path,
                                             this->m_minimal_paths(from_node_ind, to_node_ind).charge_needed_for_min_time,
                                             this->m_minimal_paths(from_node_ind, to_node_ind).length_of_min_time_path);
}

inline float Graph::minTravelTime(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept
{
  return this->m_minimal_paths(from_node_ind, to_node_ind).min_traversal_time;
}

inline std::tuple<float,float,float,float> Graph::minChargeFull(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept
{
  return std::tuple<float,float,float,float>(this->m_minimal_paths(from_node_ind, to_node_ind).time_of_min_charge_path,
                                             this->m_minimal_paths(from_node_ind, to_node_ind).min_charge_used,
                                             this->m_minimal_paths(from_node_ind, to_node_ind).charge_needed_for_min_charge,
                                             this->m_minimal_paths(from_node_ind, to_node_ind).length_of_min_charge_path);
}

inline float Graph::minCharge(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept
{
  return this->m_minimal_paths(from_node_ind, to_node_ind).min_charge_used;
}

inline std::tuple<float,float,float,float> Graph::minChargeNecessaryFull(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept
{
  return std::tuple<float,float,float,float>(this->m_minimal_paths(from_node_ind, to_node_ind).time_of_min_charge_needed,
                                             this->m_minimal_paths(from_node_ind, to_node_ind).charge_used_for_min_charge_needed,
                                             this->m_minimal_paths(from_node_ind, to_node_ind).min_charge_needed,
                                             this->m_minimal_paths(from_node_ind, to_node_ind).length_of_min_charge_needed_path);
}

inline float Graph::minChargeNecessary(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept
{
  return this->m_minimal_paths(from_node_ind, to_node_ind).min_charge_needed;
}

inline std::tuple<float,float,float,float> Graph::minLengthFull(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept
{
  return std::tuple<float,float,float,float>(this->m_minimal_paths(from_node_ind, to_node_ind).time_of_min_length_path,
                                             this->m_minimal_paths(from_node_ind, to_node_ind).charge_used_on_min_length_path,
                                             this->m_minimal_paths(from_node_ind, to_node_ind).charge_needed_for_min_length,
                                             this->m_minimal_paths(from_node_ind, to_node_ind).min_length);
}

inline float Graph::minLength(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept
{
  return this->m_minimal_paths(from_node_ind, to_node_ind).min_length;
}
} // graph

#endif
/* graph.hpp */
