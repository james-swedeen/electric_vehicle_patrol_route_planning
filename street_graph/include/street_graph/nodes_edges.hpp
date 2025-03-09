/**
 * @File: nodes_edges.hpp
 * @Date: March 2024
 * @Author: James Swedeen
 *
 * @brief
 * Defines the nodes and edges in a directed street graph.
 **/

#ifndef STREET_GRAPH_NODES_EDGES_HPP
#define STREET_GRAPH_NODES_EDGES_HPP

/* C++ Headers */
#include<array>
#include<vector>
#include<memory>
#include<algorithm>
#include<execution>

/* Eigen Headers */
#include<Eigen/Dense>

namespace graph
{
// Forward declarations
class Node;
class EdgeBase;
class Edge;
class SuperEdge;
class Graph;

/**
 * @Node
 *
 * @brief
 * Defines a node in a directed graph.
 **/
class Node
{
public:
  // friend class declaration
  friend class Graph;
  inline static constexpr const Eigen::StorageOptions EIG_OPTIONS         = Eigen::StorageOptions(Eigen::ColMajor bitor Eigen::AutoAlign);
  inline static constexpr const Eigen::Index          MAX_CONNECTED_EDGES = 6;
  /**
   * @Default Constructor
   **/
  Node() = delete;
  /**
   * @Copy Constructor
   **/
  Node(const Node&) noexcept = default;
  /**
   * @Move Constructor
   **/
  Node(Node&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * position: The location of the node in a CRS frame with units of meters
   * entering_edges: The edges in the graph that go from this another node to this node
   * exiting_edges: The edges in the graph that go from this node to another node
   * graph_index: The index of this node in the graph this node is a part of
   **/
  inline Node(const Eigen::Ref<const Eigen::Matrix<float,3,1,EIG_OPTIONS>>&                                        position) noexcept;
  inline Node(const Eigen::Ref<const Eigen::Matrix<float,3,1,EIG_OPTIONS>>&                                        position,
              const Eigen::Ref<const Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1>>& entering_edges,
              const Eigen::Ref<const Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1>>& exiting_edges,
              const uint32_t                                                                                       graph_index) noexcept;
  /**
   * @Deconstructor
   **/
  ~Node() noexcept = default;
  /**
   * @Assignment Operators
   **/
  Node& operator=(const Node&)  noexcept = default;
  Node& operator=(      Node&&) noexcept = default;
  /**
   * @Get Functions
   **/
  inline const Eigen::Matrix<float,3,1,EIG_OPTIONS>&                                        cgetPosition()      const noexcept;
  inline const Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1>& cgetEnteringEdges() const noexcept;
  inline const Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1>& cgetExitingEdges()  const noexcept;
  inline Eigen::Matrix<const Node*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1>      cgetChildren()      const noexcept;
  inline Eigen::Matrix<const Node*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1>      cgetParents()       const noexcept;
  inline uint32_t                                                                           cgetGraphIndex()    const noexcept;
  /**
   * @Query Functions
   **/
  inline size_t numberEnteringEdges() const noexcept;
  inline size_t numberExitingEdges()  const noexcept;
  inline bool   isRootNode()          const noexcept;
  /**
   * @isRedundant
   *
   * @brief
   * Tests if this node is in the middle of a road and not at an intersection.
   **/
  inline bool isRedundant() const noexcept;
protected:
  /**
   * @addEnteringEdge
   *
   * @brief
   * Adds an edge that go from another node to this node.
   **/
  inline void addEnteringEdge(EdgeBase* const new_edge) noexcept;
  /**
   * @addExitingEdge
   *
   * @brief
   * Adds an edge that go from this node to another node.
   **/
  inline void addExitingEdge(EdgeBase* const new_edge) noexcept;
  /**
   * @replaceEnteringEdge
   *
   * @brief
   * Replaces an edge that go from another node to this node.
   **/
  inline void replaceEnteringEdge(EdgeBase* const old_edge, EdgeBase* const new_edge) noexcept;
  /**
   * @ReplaceExitingEdge
   *
   * @brief
   * replaces an edge that go from this node to another node.
   **/
  inline void replaceExitingEdge(EdgeBase* const old_edge, EdgeBase* const new_edge) noexcept;
  /**
   * @Get Functions
   **/
  inline Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1>& getEnteringEdges() noexcept;
  inline Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1>& getExitingEdges()  noexcept;
  /**
   * @Set Functions
   **/
  inline uint32_t setGraphIndex(const uint32_t new_graph_index) noexcept;
private:
  Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1> m_entering_edges;
  Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1> m_exiting_edges;
  Eigen::Matrix<float,3,1,EIG_OPTIONS>                                        m_position;
  uint32_t                                                                    m_graph_index;
};

/**
 * @EdgeBase
 *
 * @brief
 * Defines an edge in a directed graph.
 **/
class EdgeBase
{
public:
  // friend class declaration
  friend class SuperEdge;
  friend class Graph;
  /**
   * @Default Constructor
   **/
  EdgeBase() = delete;
  /**
   * @Copy Constructor
   **/
  EdgeBase(const EdgeBase&) noexcept = default;
  /**
   * @Move Constructor
   **/
  EdgeBase(EdgeBase&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * from_node: The node this edge starts at
   * to_node: The node this edge ends at
   * graph_index: The index of this edge in the graph this node is a part of
   * min_traversal_time: The minimal amount of time it takes to traverse this edge
   * traversal_charge: The amount of charge used when this edge is traversed in kWh
   * length: The length of the edge in meters
   **/
  inline EdgeBase(Node* const from_node,
                  Node* const to_node,
                  const float min_traversal_time,
                  const float traversal_charge,
                  const float length) noexcept;
  inline EdgeBase(Node* const    from_node,
                  Node* const    to_node,
                  const uint32_t graph_index,
                  const float    min_traversal_time,
                  const float    traversal_charge,
                  const float    length) noexcept;
  /**
   * @Deconstructor
   **/
  virtual ~EdgeBase() noexcept = default;
  /**
   * @Assignment Operators
   **/
  EdgeBase& operator=(const EdgeBase&)  = delete;
  EdgeBase& operator=(      EdgeBase&&) = delete;
  /**
   * @Get Functions
   **/
  inline const Node* cgetFromNode()         const noexcept;
  inline const Node* cgetToNode()           const noexcept;
  inline uint32_t    cgetGraphIndex()       const noexcept;
  inline float       cgetMinTraversalTime() const noexcept;
  inline float       cgetTraversalCharge()  const noexcept;
  inline float       cgetLength()           const noexcept;
  /**
   * @generateStraightSegments
   *
   * @brief
   * Used to generate a set of straight line segments that span this edge.
   *
   * @parameters
   * points: Is filled with the points that start and end each straight segment
   * time_points: The time since entering the edge that each point is reached
   **/
  virtual void generateStraightSegments(Eigen::Matrix<float,3,Eigen::Dynamic>& points,
                                        std::vector<float>&                    time_points) const = 0;
  /**
   * @generateDiscreetSteps
   *
   * @brief
   * Used to generate a vector of discreet points that follow this edge's trajectory.
   *
   * @parameters
   * length_step: The step size in terms of path length between the discreet points
   * output_vec: After running this function, a vector of points along this path.
   **/
  virtual void generateDiscreetSteps(const float                            length_step,
                                     Eigen::Matrix<float,3,Eigen::Dynamic>& output_vec) const = 0;
  /**
   * @stateAtTime
   *
   * @brief
   * Used to retrieve the location of a vehicle after a set about of time has passed since entering this edge.
   *
   * @parameters
   * time_spent: Time spent since entering this edge
   *
   * @return
   * The state at that time.
   **/
  virtual Eigen::Matrix<float,3,1> stateAtTime(const float time_spent) const = 0;
  /**
   * @chargeUsedAtTime
   *
   * @brief
   * Used to retrieve the amount of charge used by the vehicle after a set about of time has passed since entering this edge.
   *
   * @parameters
   * time_spent: Time spent since entering this edge
   *
   * @return
   * The charge used at that time.
   **/
  virtual float chargeUsedAtTime(const float time_spent) const = 0;
  /**
   * @chargeUsedBetweenTimes
   *
   * @brief
   * Used to retrieve the amount of charge used by the vehicle between two time points along the edge.
   *
   * @parameters
   * start_time: Time spent since entering this edge to start counting charge
   * end_time: Time spent since entering this edge to stop counting charge
   *
   * @return
   * The charge used in that time.
   **/
  virtual float chargeUsedBetweenTimes(const float start_time, const float end_time) const = 0;
  /**
   * @distanceTraversedBetweenTimes
   *
   * @brief
   * Used to retrieve the length covered by the vehicle between two time points along the edge.
   *
   * @parameters
   * start_time: Time spent since entering this edge to start counting charge
   * end_time: Time spent since entering this edge to stop counting charge
   *
   * @return
   * The path length covered in that time.
   **/
  virtual float distanceTraversedBetweenTimes(const float start_time, const float end_time) const = 0;
  /**
   * @findPointInterceptTime
   *
   * @brief
   * Finds the time at which a given point along this edge exits.
   * In other words the time that when given to startAtTime will produce the original point.
   *
   * @parameters
   * point: The point to find a time for
   * max_position_error: The max deviation from the line that is allowed to still call the point part of this edge
   *
   * @return
   * The time after entering this edge that the point happens at, or if the point if not on this edge it returns Nan
   **/
  virtual float findPointInterceptTime(const Eigen::Ref<const Eigen::Matrix<float,3,1>>& point,
                                        const float                                       max_position_error = 1e-8) const = 0;
protected:
  /**
   * @Get Functions
   **/
  inline Node* getFromNode() noexcept;
  inline Node* getToNode()   noexcept;
  /**
   * @Set Functions
   **/
  inline uint32_t setGraphIndex(const uint32_t new_graph_index) noexcept;
private:
  Node* const  m_from_node;
  Node* const  m_to_node;
  uint32_t     m_graph_index;
  const float  m_min_traversal_time;
  const float  m_traversal_charge;
  const float  m_length;
};

/**
 * @Edge
 *
 * @brief
 * A straight edge that hasn't been simplified.
 **/
class Edge
: public EdgeBase
{
public:
  /**
   * @Default Constructor
   **/
  Edge() = delete;
  /**
   * @Copy Constructor
   **/
  Edge(const Edge&) noexcept = default;
  /**
   * @Move Constructor
   **/
  Edge(Edge&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * from_node: The node this edge starts at
   * to_node: The node this edge ends at
   * graph_index: The index of this edge in the graph this node is a part of
   * length: The length of the edge in meters
   * speed_limit: The edge's speed limit in meters per a second
   * traversal_charge: The amount of charge used when this edge is traversed in kWh
   **/
  inline Edge(Node* const from_node,
              Node* const to_node,
              const float length,
              const float speed_limit,
              const float traversal_charge) noexcept;
  inline Edge(Node* const    from_node,
              Node* const    to_node,
              const uint32_t graph_index,
              const float    length,
              const float    speed_limit,
              const float    traversal_charge) noexcept;
  /**
   * @Deconstructor
   **/
  ~Edge() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  Edge& operator=(const Edge&)  = delete;
  Edge& operator=(      Edge&&) = delete;
  /**
   * @Get Functions
   **/
  inline float cgetSpeedLimit() const noexcept;
  /**
   * @generateStraightSegments
   *
   * @brief
   * Used to generate a set of straight line segments that span this edge.
   *
   * @parameters
   * points: Is filled with the points that start and end each straight segment
   * time_points: The time since entering the edge that each point is reached
   **/
  void generateStraightSegments(Eigen::Matrix<float,3,Eigen::Dynamic>& points,
                                std::vector<float>&                    time_points) const override;
  /**
   * @generateDiscreetSteps
   *
   * @brief
   * Used to generate a vector of discreet points that follow this edge's trajectory.
   *
   * @parameters
   * length_step: The step size in terms of path length between the discreet points
   * output_vec: After running this function, a vector of points along this path.
   **/
  void generateDiscreetSteps(const float                            length_step,
                             Eigen::Matrix<float,3,Eigen::Dynamic>& output_vec) const override;
  /**
   * @stateAtTime
   *
   * @brief
   * Used to retrieve the location of a vehicle after a set about of time has passed since entering this edge.
   *
   * @parameters
   * time_spent: Time spent since entering this edge
   *
   * @return
   * The state at that time.
   **/
  inline Eigen::Matrix<float,3,1> stateAtTime(const float time_spent) const final;
  /**
   * @chargeUsedAtTime
   *
   * @brief
   * Used to retrieve the amount of charge used by the vehicle after a set about of time has passed since entering this edge.
   *
   * @parameters
   * time_spent: Time spent since entering this edge
   *
   * @return
   * The charge used at that time.
   **/
  inline float chargeUsedAtTime(const float time_spent) const final;
  /**
   * @chargeUsedBetweenTimes
   *
   * @brief
   * Used to retrieve the amount of charge used by the vehicle between two time points along the edge.
   *
   * @parameters
   * start_time: Time spent since entering this edge to start counting charge
   * end_time: Time spent since entering this edge to stop counting charge
   *
   * @return
   * The charge used in that time.
   **/
  inline float chargeUsedBetweenTimes(const float start_time, const float end_time) const final;
  /**
   * @distanceTraversedBetweenTimes
   *
   * @brief
   * Used to retrieve the length covered by the vehicle between two time points along the edge.
   *
   * @parameters
   * start_time: Time spent since entering this edge to start counting charge
   * end_time: Time spent since entering this edge to stop counting charge
   *
   * @return
   * The path length covered in that time.
   **/
  inline float distanceTraversedBetweenTimes(const float start_time, const float end_time) const final;
  /**
   * @findPointInterceptTime
   *
   * @brief
   * Finds the time at which a given point along this edge exits.
   * In other words the time that when given to startAtTime will produce the original point.
   *
   * @parameters
   * point: The point to find a time for
   * max_position_error: The max deviation from the line that is allowed to still call the point part of this edge
   *
   * @return
   * The time after entering this edge that the point happens at, or if the point if not on this edge it returns Nan
   **/
  float findPointInterceptTime(const Eigen::Ref<const Eigen::Matrix<float,3,1>>& point,
                               const float                                       max_position_error = 1e-8) const override;
private:
  const float m_speed_limit;
};

/**
 * @SuperEdge
 *
 * @brief
 * An edge that is composed of multiple edges concatenated end to end.
 **/
class SuperEdge
: public EdgeBase
{
public:
  /**
   * @Default Constructor
   **/
  SuperEdge() = delete;
  /**
   * @Copy Constructor
   **/
  SuperEdge(const SuperEdge&) = delete;
  /**
   * @Move Constructor
   **/
  SuperEdge(SuperEdge&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * first_edge: The edge that ends where the second edge starts (this object takes control of memory freeing)
   * second_edge: The edge that starts where the first edge ends (this object takes control of memory freeing)
   * graph_index: The index of this edge in the graph this node is a part of
   **/
  inline SuperEdge(EdgeBase* const first_edge, EdgeBase* const second_edge) noexcept;
  inline SuperEdge(EdgeBase* const first_edge, EdgeBase* const second_edge, const uint32_t graph_index) noexcept;
  /**
   * @Deconstructor
   **/
  ~SuperEdge() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  SuperEdge& operator=(const SuperEdge&)  = delete;
  SuperEdge& operator=(      SuperEdge&&) = delete;
  /**
   * @Get Functions
   **/
  inline const std::pair<std::unique_ptr<EdgeBase>,std::unique_ptr<EdgeBase>>& cgetSubEdges() const noexcept;
  /**
   * @generateStraightSegments
   *
   * @brief
   * Used to generate a set of straight line segments that span this edge.
   *
   * @parameters
   * points: Is filled with the points that start and end each straight segment
   * time_points: The time since entering the edge that each point is reached
   **/
  void generateStraightSegments(Eigen::Matrix<float,3,Eigen::Dynamic>& points,
                                std::vector<float>&                    time_points) const override;
  /**
   * @generateDiscreetSteps
   *
   * @brief
   * Used to generate a vector of discreet points that follow this edge's trajectory.
   *
   * @parameters
   * length_step: The step size in terms of path length between the discreet points
   * output_vec: After running this function, a vector of points along this path.
   **/
  void generateDiscreetSteps(const float                            length_step,
                             Eigen::Matrix<float,3,Eigen::Dynamic>& output_vec) const override;
  /**
   * @stateAtTime
   *
   * @brief
   * Used to retrieve the location of a vehicle after a set about of time has passed since entering this edge.
   *
   * @parameters
   * time_spent: Time spent since entering this edge
   *
   * @return
   * The state at that time.
   **/
  Eigen::Matrix<float,3,1> stateAtTime(const float time_spent) const override;
  /**
   * @chargeUsedAtTime
   *
   * @brief
   * Used to retrieve the amount of charge used by the vehicle after a set about of time has passed since entering this edge.
   *
   * @parameters
   * time_spent: Time spent since entering this edge
   *
   * @return
   * The charge used at that time.
   **/
  float chargeUsedAtTime(const float time_spent) const override;
  /**
   * @chargeUsedBetweenTimes
   *
   * @brief
   * Used to retrieve the amount of charge used by the vehicle between two time points along the edge.
   *
   * @parameters
   * start_time: Time spent since entering this edge to start counting charge
   * end_time: Time spent since entering this edge to stop counting charge
   *
   * @return
   * The charge used in that time.
   **/
  float chargeUsedBetweenTimes(const float start_time, const float end_time) const override;
  /**
   * @distanceTraversedBetweenTimes
   *
   * @brief
   * Used to retrieve the length covered by the vehicle between two time points along the edge.
   *
   * @parameters
   * start_time: Time spent since entering this edge to start counting charge
   * end_time: Time spent since entering this edge to stop counting charge
   *
   * @return
   * The path length covered in that time.
   **/
  float distanceTraversedBetweenTimes(const float start_time, const float end_time) const override;
  /**
   * @findPointInterceptTime
   *
   * @brief
   * Finds the time at which a given point along this edge exits.
   * In other words the time that when given to startAtTime will produce the original point.
   *
   * @parameters
   * point: The point to find a time for
   * max_position_error: The max deviation from the line that is allowed to still call the point part of this edge
   *
   * @return
   * The time after entering this edge that the point happens at, or if the point if not on this edge it returns Nan
   **/
  float findPointInterceptTime(const Eigen::Ref<const Eigen::Matrix<float,3,1>>& point,
                               const float                                       max_position_error = 1e-8) const override;
private:
  const std::pair<std::unique_ptr<EdgeBase>,std::unique_ptr<EdgeBase>> m_sub_edges;
};



inline Node::Node(const Eigen::Ref<const Eigen::Matrix<float,3,1,EIG_OPTIONS>>& position) noexcept
 : m_position(position)
{}

inline Node::Node(const Eigen::Ref<const Eigen::Matrix<float,3,1,EIG_OPTIONS>>&                                        position,
                  const Eigen::Ref<const Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1>>& entering_edges,
                  const Eigen::Ref<const Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1>>& exiting_edges,
                  const uint32_t                                                                                       graph_index) noexcept
 : m_position(position),
   m_entering_edges(entering_edges),
   m_exiting_edges(exiting_edges),
   m_graph_index(graph_index)
{}

inline const Eigen::Matrix<float,3,1,Node::EIG_OPTIONS>& Node::cgetPosition() const noexcept
{
  return this->m_position;
}

inline const Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,Node::EIG_OPTIONS,Node::MAX_CONNECTED_EDGES,1>& Node::cgetEnteringEdges() const noexcept
{
  return this->m_entering_edges;
}

inline const Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,Node::EIG_OPTIONS,Node::MAX_CONNECTED_EDGES,1>& Node::cgetExitingEdges() const noexcept
{
  return this->m_exiting_edges;
}

inline Eigen::Matrix<const Node*,Eigen::Dynamic,1,Node::EIG_OPTIONS,Node::MAX_CONNECTED_EDGES,1> Node::cgetChildren() const noexcept
{
  const size_t num_children = this->numberExitingEdges();
  Eigen::Matrix<const Node*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1> output(num_children);

  for(size_t child_it = 0; child_it < num_children; ++child_it)
  {
    output[child_it] = this->cgetExitingEdges()[child_it]->cgetToNode();
  }

  return output;
}

inline Eigen::Matrix<const Node*,Eigen::Dynamic,1,Node::EIG_OPTIONS,Node::MAX_CONNECTED_EDGES,1> Node::cgetParents() const noexcept
{
  const size_t num_parents = this->numberEnteringEdges();
  Eigen::Matrix<const Node*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1> output(num_parents);

  for(size_t par_it = 0; par_it < num_parents; ++par_it)
  {
    output[par_it] = this->cgetEnteringEdges()[par_it]->cgetFromNode();
  }

  return output;
}

inline uint32_t Node::cgetGraphIndex() const noexcept
{
  return this->m_graph_index;
}

inline size_t Node::numberEnteringEdges() const noexcept
{
  return this->cgetEnteringEdges().size();
}

inline size_t Node::numberExitingEdges() const noexcept
{
  return this->cgetExitingEdges().size();
}

inline bool Node::isRootNode() const noexcept
{
  return this->cgetGraphIndex() == 0;
}

inline bool Node::isRedundant() const noexcept
{
  if((1 == this->numberExitingEdges())  and
     (1 == this->numberEnteringEdges()) and
     (this->cgetEnteringEdges()[0]->cgetFromNode() != this->cgetExitingEdges()[0]->cgetToNode()))
  {
    return true;
  }
  if((2 == this->numberExitingEdges()) and (2 == this->numberEnteringEdges()))
  {
    return ((this->cgetExitingEdges()[0]->cgetToNode() == this->cgetEnteringEdges()[0]->cgetFromNode()) or
            (this->cgetExitingEdges()[0]->cgetToNode() == this->cgetEnteringEdges()[1]->cgetFromNode())) and
           ((this->cgetExitingEdges()[1]->cgetToNode() == this->cgetEnteringEdges()[0]->cgetFromNode()) or
            (this->cgetExitingEdges()[1]->cgetToNode() == this->cgetEnteringEdges()[1]->cgetFromNode()));
  }
  return false;
}

inline void Node::addEnteringEdge(EdgeBase* const new_edge) noexcept
{
  const Eigen::Index                                                          num_enter_edges = this->cgetEnteringEdges().size();
  Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1> new_enter_edges(num_enter_edges + 1);
  new_enter_edges.topRows(num_enter_edges) = this->cgetEnteringEdges();
  new_enter_edges.bottomRows<1>()[0]       = new_edge;
  this->m_entering_edges = std::move(new_enter_edges);
  assert(this == this->cgetEnteringEdges().bottomRows<1>()[0]->cgetToNode());
//  assert(std::all_of(std::execution::par_unseq, this->cgetEnteringEdges().cbegin(), this->cgetEnteringEdges().cend(),
//                     [this] (const EdgeBase* const i) -> bool
//         {
//           return std::all_of(std::execution::par_unseq, this->cgetEnteringEdges().cbegin(), this->cgetEnteringEdges().cend(),
//                              [i] (const EdgeBase* const j) -> bool
//                              {
//                                return (i == j) or (i->cgetFromNode() != j->cgetFromNode());
//                              });
//         }));
}

inline void Node::addExitingEdge(EdgeBase* const new_edge) noexcept
{
  const Eigen::Index                                                          num_exit_edges = this->cgetExitingEdges().size();
  Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,EIG_OPTIONS,MAX_CONNECTED_EDGES,1> new_exit_edges(num_exit_edges + 1);
  new_exit_edges.topRows(num_exit_edges) = this->cgetExitingEdges();
  new_exit_edges.bottomRows<1>()[0]      = new_edge;
  this->m_exiting_edges = std::move(new_exit_edges);
  assert(this == this->cgetExitingEdges().bottomRows<1>()[0]->cgetFromNode());
//  assert(std::all_of(std::execution::par_unseq, this->cgetExitingEdges().cbegin(), this->cgetExitingEdges().cend(),
//                     [this] (const EdgeBase* const i) -> bool
//         {
//           return std::all_of(std::execution::par_unseq, this->cgetExitingEdges().cbegin(), this->cgetExitingEdges().cend(),
//                              [i] (const EdgeBase* const j) -> bool
//                              {
//                                return (i == j) or (i->cgetToNode() != j->cgetToNode());
//                              });
//         }));
}

inline void Node::replaceEnteringEdge(EdgeBase* const old_edge, EdgeBase* const new_edge) noexcept
{
  assert(this == new_edge->cgetToNode());
  assert(this->cgetEnteringEdges().cend() != std::find(this->getEnteringEdges().cbegin(), this->getEnteringEdges().cend(), old_edge));
  std::replace(this->getEnteringEdges().begin(), this->getEnteringEdges().end(), old_edge, new_edge);
//  assert(std::all_of(std::execution::par_unseq, this->cgetEnteringEdges().cbegin(), this->cgetEnteringEdges().cend(),
//                     [this] (const EdgeBase* const i) -> bool
//         {
//           return std::all_of(std::execution::par_unseq, this->cgetEnteringEdges().cbegin(), this->cgetEnteringEdges().cend(),
//                              [i] (const EdgeBase* const j) -> bool
//                              {
//                                return (i == j) or (i->cgetFromNode() != j->cgetFromNode());
//                              });
//         }));
}

inline void Node::replaceExitingEdge(EdgeBase* const old_edge, EdgeBase* const new_edge) noexcept
{
  assert(this == new_edge->cgetFromNode());
  assert(this->cgetExitingEdges().cend() != std::find(this->getExitingEdges().cbegin(), this->getExitingEdges().cend(), old_edge));
  std::replace(this->getExitingEdges().begin(), this->getExitingEdges().end(), old_edge, new_edge);
//  assert(std::all_of(std::execution::par_unseq, this->cgetExitingEdges().cbegin(), this->cgetExitingEdges().cend(),
//                     [this] (const EdgeBase* const i) -> bool
//         {
//           return std::all_of(std::execution::par_unseq, this->cgetExitingEdges().cbegin(), this->cgetExitingEdges().cend(),
//                              [i] (const EdgeBase* const j) -> bool
//                              {
//                                return (i == j) or (i->cgetToNode() != j->cgetToNode());
//                              });
//         }));
}

inline Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,Node::EIG_OPTIONS,Node::MAX_CONNECTED_EDGES,1>& Node::getEnteringEdges() noexcept
{
  return this->m_entering_edges;
}

inline Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,Node::EIG_OPTIONS,Node::MAX_CONNECTED_EDGES,1>& Node::getExitingEdges() noexcept
{
  return this->m_exiting_edges;
}

inline uint32_t Node::setGraphIndex(const uint32_t new_graph_index) noexcept
{
  return this->m_graph_index = new_graph_index;
}

inline EdgeBase::EdgeBase(Node* const from_node,
                          Node* const to_node,
                          const float min_traversal_time,
                          const float traversal_charge,
                          const float length) noexcept
 : m_from_node(from_node),
   m_to_node(to_node),
   m_min_traversal_time(min_traversal_time),
   m_traversal_charge(traversal_charge),
   m_length(length)
{
  assert(from_node != to_node);
}

inline EdgeBase::EdgeBase(Node* const    from_node,
                          Node* const    to_node,
                          const uint32_t graph_index,
                          const float    min_traversal_time,
                          const float    traversal_charge,
                          const float    length) noexcept
 : m_from_node(from_node),
   m_to_node(to_node),
   m_min_traversal_time(min_traversal_time),
   m_traversal_charge(traversal_charge),
   m_graph_index(graph_index),
   m_length(length)
{
  assert(from_node != to_node);
}

inline const Node* EdgeBase::cgetFromNode() const noexcept
{
  return this->m_from_node;
}

inline const Node* EdgeBase::cgetToNode() const noexcept
{
  return this->m_to_node;
}

inline uint32_t EdgeBase::cgetGraphIndex() const noexcept
{
  return this->m_graph_index;
}

inline float EdgeBase::cgetMinTraversalTime() const noexcept
{
  return this->m_min_traversal_time;
}

inline float EdgeBase::cgetTraversalCharge() const noexcept
{
  return this->m_traversal_charge;
}

inline float EdgeBase::cgetLength() const noexcept
{
  return this->m_length;
}

inline Node* EdgeBase::getFromNode() noexcept
{
  return this->m_from_node;
}

inline Node* EdgeBase::getToNode() noexcept
{
  return this->m_to_node;
}

inline uint32_t EdgeBase::setGraphIndex(const uint32_t new_graph_index) noexcept
{
  return this->m_graph_index = new_graph_index;
}

inline Edge::Edge(Node* const from_node,
                  Node* const to_node,
                  const float length,
                  const float speed_limit,
                  const float traversal_charge) noexcept
 : EdgeBase(from_node,
            to_node,
            length/speed_limit,
            traversal_charge,
            length),
   m_speed_limit(speed_limit)
{}

inline Edge::Edge(Node* const    from_node,
                  Node* const    to_node,
                  const uint32_t graph_index,
                  const float    length,
                  const float    speed_limit,
                  const float    traversal_charge) noexcept
 : EdgeBase(from_node,
            to_node,
            graph_index,
            length/speed_limit,
            traversal_charge,
            length),
   m_speed_limit(speed_limit)
{}

inline float Edge::cgetSpeedLimit() const noexcept
{
  return this->m_speed_limit;
}

inline Eigen::Matrix<float,3,1> Edge::stateAtTime(const float time_spent) const
{
  assert((time_spent - this->cgetMinTraversalTime()) < 1e-2);

  return this->cgetFromNode()->cgetPosition() + (time_spent*this->cgetSpeedLimit()*(this->cgetToNode()->cgetPosition() - this->cgetFromNode()->cgetPosition()).normalized());
}

inline float Edge::chargeUsedAtTime(const float time_spent) const
{
  assert((time_spent - this->cgetMinTraversalTime()) < 1e-2);

  const float percent_through = time_spent / this->cgetMinTraversalTime();

  return this->cgetTraversalCharge() * percent_through;
}

inline float Edge::chargeUsedBetweenTimes(const float start_time, const float end_time) const
{
  assert(start_time >= -1e-2);
  assert((end_time - this->cgetMinTraversalTime()) < 1e-2);
  assert(start_time <= end_time);

  return this->Edge::chargeUsedAtTime(end_time - start_time);
}

inline float Edge::distanceTraversedBetweenTimes(const float start_time, const float end_time) const
{
  assert(start_time >= -1e-2);
  assert((end_time - this->cgetMinTraversalTime()) < 1e-2);
  assert(start_time <= end_time);

  return (end_time - start_time) * this->m_speed_limit;
}

inline SuperEdge::SuperEdge(EdgeBase* const first_edge, EdgeBase* const second_edge) noexcept
 : EdgeBase(first_edge->getFromNode(),
            second_edge->getToNode(),
            first_edge->cgetMinTraversalTime() + second_edge->cgetMinTraversalTime(),
            first_edge->cgetTraversalCharge() + second_edge->cgetTraversalCharge(),
            first_edge->cgetLength() + second_edge->cgetLength()),
   m_sub_edges(first_edge, second_edge)
{}

inline SuperEdge::SuperEdge(EdgeBase* const first_edge, EdgeBase* const second_edge, const uint32_t graph_index) noexcept
 : EdgeBase(first_edge->getFromNode(),
            second_edge->getToNode(),
            graph_index,
            first_edge->cgetMinTraversalTime() + second_edge->cgetMinTraversalTime(),
            first_edge->cgetTraversalCharge() + second_edge->cgetTraversalCharge(),
            first_edge->cgetLength() + second_edge->cgetLength()),
   m_sub_edges(first_edge, second_edge)
{}

inline const std::pair<std::unique_ptr<EdgeBase>,std::unique_ptr<EdgeBase>>& SuperEdge::cgetSubEdges() const noexcept
{
  return this->m_sub_edges;
}
} // graph

#endif
/* nodes_edges.hpp */
