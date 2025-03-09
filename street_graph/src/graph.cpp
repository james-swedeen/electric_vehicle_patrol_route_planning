/**
 * @File: graph.cpp
 * @Date: March 2024
 * @Author: James Swedeen
 **/

/* C++ Headers */
#include<vector>
#include<unordered_set>
#include<string>
#include<filesystem>
#include<list>
#include<forward_list>
#include<memory>
#include<fstream>
#include<stdexcept>
#include<sstream>
#include<algorithm>
#include<execution>
#include<atomic>
#include<iostream> // TODO

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<street_graph/nodes_edges.hpp>
#include<street_graph/graph.hpp>

namespace graph
{
Graph::Graph(const std::string& nodes_file_path, const std::string& edges_file_path, const double max_area_for_straight, const bool need_min_paths)
{
  constexpr const double FLOAT_COMP_EPS = 1e-12;

  /// Load in original graph
  // Open files
  std::ifstream nodes_file(nodes_file_path);
  std::ifstream edges_file(edges_file_path);

  if(not nodes_file.is_open()) { throw std::runtime_error("Could not open nodes file: "+nodes_file_path); }
  if(not edges_file.is_open()) { throw std::runtime_error("Could not open edges file: "+edges_file_path); }

  // Read in nodes list
  std::forward_list<std::unique_ptr<Node>> node_list;
  uint32_t                                 nodes_count = 0;
  std::string                              line;

  // Get passed the labels
  std::getline(nodes_file, line, '\n');
  // Read the next line
  std::getline(nodes_file, line, '\n');
  while(nodes_file.good())
  {
    std::stringstream        line_stream(line, std::ios_base::in);
    std::string              value;
    Eigen::Matrix<float,3,1> position;
    // Get position x
    std::getline(line_stream, value, ',');
    position[0] = std::stod(value);
    // Get position y
    std::getline(line_stream, value, ',');
    position[1] = std::stod(value);
    // Get elevation
    std::getline(line_stream, value, '\n');
    position[2] = std::stod(value);
    // Make node
    node_list.push_front(std::make_unique<Node>(position));
    ++nodes_count;
    // Read the next line
    std::getline(nodes_file, line, '\n');
  }
  // Convert to nodes vector
  this->m_nodes.resize(nodes_count);
  for(long node_it = nodes_count-1; node_it >= 0; --node_it)
  {
    this->m_nodes[node_it].swap(node_list.front());
    node_list.pop_front();
  }
  // Add node indexes
  const boost::integer_range<uint32_t> node_inds_temp(0, nodes_count);
  std::for_each(std::execution::par_unseq, node_inds_temp.begin(), node_inds_temp.end(),
                [this] (const uint32_t node_ind) -> void { this->m_nodes[node_ind]->setGraphIndex(node_ind); });

  // Read in edges file
  this->m_edges.reserve(Node::MAX_CONNECTED_EDGES * nodes_count);
  // Get passed the labels
  std::getline(edges_file, line, '\n');
  // Read the next line
  std::getline(edges_file, line, '\n');
  while(edges_file.good())
  {
    std::stringstream line_stream(line, std::ios_base::in);
    std::string       value;
    // Get from node
    std::getline(line_stream, value, ',');
    const uint32_t from_node_ind = std::stoul(value);
    // Get to node
    std::getline(line_stream, value, ',');
    const uint32_t to_node_ind = std::stoul(value);
    // Get length
    const float length = (this->m_nodes[from_node_ind]->cgetPosition() - this->m_nodes[to_node_ind]->cgetPosition()).norm();
    // Get speed
    std::getline(line_stream, value, ',');
    const float speed = std::stod(value);
    // Get power usage
    std::getline(line_stream, value, '\n');
    const float traversal_charge = std::stod(value);
    // Make edge
    this->m_edges.emplace_back(std::make_unique<Edge>(this->m_nodes[from_node_ind].get(),
                                                      this->m_nodes[to_node_ind].  get(),
                                                      this->m_edges.size(),
                                                      length,
                                                      speed,
                                                      traversal_charge));
    this->m_nodes[from_node_ind]->addExitingEdge( this->m_edges.back().get());
    this->m_nodes[to_node_ind]->  addEnteringEdge(this->m_edges.back().get());
    // Read the next line
    std::getline(edges_file, line, '\n');
  }
//  this->m_edges.shrink_to_fit();
  #ifndef NDEBUG
    for(uint32_t node_ind = 0; node_ind < nodes_count; ++node_ind)
    {
      assert(node_ind == this->m_nodes[node_ind]->cgetGraphIndex());
    }
    for(uint32_t edge_ind = 0; edge_ind < this->numberOriginalEdges(); ++edge_ind)
    {
      assert(edge_ind == this->m_edges[edge_ind]->cgetGraphIndex());
    }
    for(uint32_t edge_ind = 0; edge_ind < this->numberOriginalEdges(); ++edge_ind)
    {
      for(uint32_t other_edge_ind = 0; other_edge_ind < this->numberOriginalEdges(); ++other_edge_ind)
      {
        if((this->cgetOriginalEdges()[edge_ind]->cgetFromNode() == this->cgetOriginalEdges()[other_edge_ind]->cgetToNode()) and
           (this->cgetOriginalEdges()[edge_ind]->cgetToNode()   == this->cgetOriginalEdges()[other_edge_ind]->cgetFromNode()))
        {
          assert((this->cgetOriginalEdges()[edge_ind]->cgetTraversalCharge() + this->cgetOriginalEdges()[other_edge_ind]->cgetTraversalCharge()) >= 0);
        }
      }
    }
  #endif
  // Finds the area of a triangle defined by three points
  const auto triangle_area_func =
  [] (const Eigen::Ref<const Eigen::Matrix<float,3,1>>& A, const Eigen::Ref<const Eigen::Matrix<float,3,1>>& B, const Eigen::Ref<const Eigen::Matrix<float,3,1>>& C) -> float
  {
    return std::fabs((A[0] * (B[1] - C[1])) + (B[0] * (C[1] - A[1])) + (C[0] * (A[1] - B[1]))) / float(2);
  };
  // Remove redundant nodes
  for(uint32_t node_ind = 1; node_ind < nodes_count; ++node_ind)
  {
    // Test for being redundant
    if((1 == this->m_nodes[node_ind]->numberExitingEdges())  and
       (1 == this->m_nodes[node_ind]->numberEnteringEdges()) and
       (this->m_nodes[node_ind]->cgetEnteringEdges()[0]->cgetFromNode() != this->m_nodes[node_ind]->cgetExitingEdges()[0]->cgetToNode()) and
       (std::signbit(this->m_nodes[node_ind]->cgetEnteringEdges()[0]->cgetTraversalCharge()) == std::signbit(this->m_nodes[node_ind]->cgetExitingEdges()[0]->cgetTraversalCharge())) and
       (max_area_for_straight >= triangle_area_func(this->m_nodes[node_ind]->cgetEnteringEdges()[0]->cgetFromNode()->cgetPosition(),
                                                    this->m_nodes[node_ind]->cgetPosition(),
                                                    this->m_nodes[node_ind]->cgetExitingEdges()[0]->cgetToNode()->cgetPosition())))
    {
      const uint32_t exiting_edge_ind  = this->m_nodes[node_ind]->cgetExitingEdges() [0]->cgetGraphIndex();
      const uint32_t entering_edge_ind = this->m_nodes[node_ind]->cgetEnteringEdges()[0]->cgetGraphIndex();

      const float len = this->m_edges[entering_edge_ind]->cgetLength() + this->m_edges[exiting_edge_ind]->cgetLength();
      this->m_edges.emplace_back(std::make_unique<Edge>(this->m_nodes[node_ind]->cgetEnteringEdges()[0]->getFromNode(),
                                                        this->m_nodes[node_ind]->cgetExitingEdges()[0]->getToNode(),
                                                        this->m_edges.size(),
                                                        len,
                                                        len / (this->m_edges[entering_edge_ind]->cgetMinTraversalTime() + this->m_edges[exiting_edge_ind]->cgetMinTraversalTime()),
                                                        this->m_edges[entering_edge_ind]->cgetTraversalCharge() + this->m_edges[exiting_edge_ind]->cgetTraversalCharge()));
      this->m_edges.back()->getFromNode()->replaceExitingEdge(this->m_edges[entering_edge_ind].get(), this->m_edges.back().get());
      this->m_edges.back()->getToNode()->replaceEnteringEdge( this->m_edges[exiting_edge_ind]. get(), this->m_edges.back().get());
      this->m_nodes[node_ind].         reset();
      this->m_edges[exiting_edge_ind]. reset();
      this->m_edges[entering_edge_ind].reset();
    }
    else if((2 == this->m_nodes[node_ind]->numberExitingEdges()) and
            (2 == this->m_nodes[node_ind]->numberEnteringEdges()))
    {
      const bool zz = this->m_nodes[node_ind]->cgetExitingEdges()[0]->cgetToNode() == this->m_nodes[node_ind]->cgetEnteringEdges()[0]->cgetFromNode();
      const bool zo = this->m_nodes[node_ind]->cgetExitingEdges()[0]->cgetToNode() == this->m_nodes[node_ind]->cgetEnteringEdges()[1]->cgetFromNode();
      const bool oz = this->m_nodes[node_ind]->cgetExitingEdges()[1]->cgetToNode() == this->m_nodes[node_ind]->cgetEnteringEdges()[0]->cgetFromNode();
      const bool oo = this->m_nodes[node_ind]->cgetExitingEdges()[1]->cgetToNode() == this->m_nodes[node_ind]->cgetEnteringEdges()[1]->cgetFromNode();

      if((zz and zo) or (oz and oo)) { continue; }

      if(zz and oo and
        (std::signbit(this->m_nodes[node_ind]->cgetEnteringEdges()[0]->cgetTraversalCharge()) == std::signbit(this->m_nodes[node_ind]->cgetExitingEdges()[1]->cgetTraversalCharge())) and
        (std::signbit(this->m_nodes[node_ind]->cgetEnteringEdges()[1]->cgetTraversalCharge()) == std::signbit(this->m_nodes[node_ind]->cgetExitingEdges()[0]->cgetTraversalCharge())) and
        (max_area_for_straight >= triangle_area_func(this->m_nodes[node_ind]->cgetEnteringEdges()[0]->cgetFromNode()->cgetPosition(),
                                                     this->m_nodes[node_ind]->cgetPosition(),
                                                     this->m_nodes[node_ind]->cgetExitingEdges()[1]->cgetToNode()->cgetPosition())))
      {
        const uint32_t exiting_edge_indz  = this->m_nodes[node_ind]->cgetExitingEdges() [0]->cgetGraphIndex();
        const uint32_t entering_edge_indz = this->m_nodes[node_ind]->cgetEnteringEdges()[0]->cgetGraphIndex();
        const uint32_t exiting_edge_indo  = this->m_nodes[node_ind]->cgetExitingEdges() [1]->cgetGraphIndex();
        const uint32_t entering_edge_indo = this->m_nodes[node_ind]->cgetEnteringEdges()[1]->cgetGraphIndex();

        // One way
        const float len1 = this->m_edges[entering_edge_indz]->cgetLength() + this->m_edges[exiting_edge_indo]->cgetLength();
        this->m_edges.emplace_back(std::make_unique<Edge>(this->m_nodes[node_ind]->cgetEnteringEdges()[0]->getFromNode(),
                                                          this->m_nodes[node_ind]->cgetExitingEdges()[1]->getToNode(),
                                                          this->m_edges.size(),
                                                          len1,
                                                          len1 / (this->m_edges[entering_edge_indz]->cgetMinTraversalTime() + this->m_edges[exiting_edge_indo]->cgetMinTraversalTime()),
                                                          this->m_edges[entering_edge_indz]->cgetTraversalCharge() + this->m_edges[exiting_edge_indo]->cgetTraversalCharge()));
        this->m_edges.back()->getFromNode()->replaceExitingEdge(this->m_edges[entering_edge_indz].get(), this->m_edges.back().get());
        this->m_edges.back()->getToNode()->replaceEnteringEdge( this->m_edges[exiting_edge_indo]. get(), this->m_edges.back().get());
        // The other way
        const float len2 = this->m_edges[entering_edge_indo]->cgetLength() + this->m_edges[exiting_edge_indz]->cgetLength();
        this->m_edges.emplace_back(std::make_unique<Edge>(this->m_nodes[node_ind]->cgetEnteringEdges()[1]->getFromNode(),
                                                          this->m_nodes[node_ind]->cgetExitingEdges()[0]->getToNode(),
                                                          this->m_edges.size(),
                                                          len2,
                                                          len2 / (this->m_edges[entering_edge_indo]->cgetMinTraversalTime() + this->m_edges[exiting_edge_indz]->cgetMinTraversalTime()),
                                                          this->m_edges[entering_edge_indo]->cgetTraversalCharge() + this->m_edges[exiting_edge_indz]->cgetTraversalCharge()));
        this->m_edges.back()->getFromNode()->replaceExitingEdge(this->m_edges[entering_edge_indo].get(), this->m_edges.back().get());
        this->m_edges.back()->getToNode()->replaceEnteringEdge( this->m_edges[exiting_edge_indz]. get(), this->m_edges.back().get());

        this->m_nodes[node_ind].          reset();
        this->m_edges[exiting_edge_indz]. reset();
        this->m_edges[exiting_edge_indo]. reset();
        this->m_edges[entering_edge_indz].reset();
        this->m_edges[entering_edge_indo].reset();
      }
      else if(zo and oz and
             (std::signbit(this->m_nodes[node_ind]->cgetEnteringEdges()[0]->cgetTraversalCharge()) == std::signbit(this->m_nodes[node_ind]->cgetExitingEdges()[0]->cgetTraversalCharge())) and
             (std::signbit(this->m_nodes[node_ind]->cgetEnteringEdges()[1]->cgetTraversalCharge()) == std::signbit(this->m_nodes[node_ind]->cgetExitingEdges()[1]->cgetTraversalCharge())) and
             (max_area_for_straight >= triangle_area_func(this->m_nodes[node_ind]->cgetEnteringEdges()[0]->cgetFromNode()->cgetPosition(),
                                                          this->m_nodes[node_ind]->cgetPosition(),
                                                          this->m_nodes[node_ind]->cgetExitingEdges()[0]->cgetToNode()->cgetPosition())))
      {
        const uint32_t exiting_edge_indz  = this->m_nodes[node_ind]->cgetExitingEdges() [0]->cgetGraphIndex();
        const uint32_t entering_edge_indz = this->m_nodes[node_ind]->cgetEnteringEdges()[0]->cgetGraphIndex();
        const uint32_t exiting_edge_indo  = this->m_nodes[node_ind]->cgetExitingEdges() [1]->cgetGraphIndex();
        const uint32_t entering_edge_indo = this->m_nodes[node_ind]->cgetEnteringEdges()[1]->cgetGraphIndex();

        // One way
        const float len1 = this->m_edges[entering_edge_indz]->cgetLength() + this->m_edges[exiting_edge_indz]->cgetLength();
        this->m_edges.emplace_back(std::make_unique<Edge>(this->m_nodes[node_ind]->cgetEnteringEdges()[0]->getFromNode(),
                                                          this->m_nodes[node_ind]->cgetExitingEdges()[0]->getToNode(),
                                                          this->m_edges.size(),
                                                          len1,
                                                          len1 / (this->m_edges[entering_edge_indz]->cgetMinTraversalTime() + this->m_edges[exiting_edge_indz]->cgetMinTraversalTime()),
                                                          this->m_edges[entering_edge_indz]->cgetTraversalCharge() + this->m_edges[exiting_edge_indz]->cgetTraversalCharge()));
        this->m_edges.back()->getFromNode()->replaceExitingEdge(this->m_edges[entering_edge_indz].get(), this->m_edges.back().get());
        this->m_edges.back()->getToNode()->replaceEnteringEdge( this->m_edges[exiting_edge_indz]. get(), this->m_edges.back().get());
        // The other way
        const float len2 = this->m_edges[entering_edge_indo]->cgetLength() + this->m_edges[exiting_edge_indo]->cgetLength();
        this->m_edges.emplace_back(std::make_unique<Edge>(this->m_nodes[node_ind]->cgetEnteringEdges()[1]->getFromNode(),
                                                          this->m_nodes[node_ind]->cgetExitingEdges()[1]->getToNode(),
                                                          this->m_edges.size(),
                                                          len2,
                                                          len2 / (this->m_edges[entering_edge_indo]->cgetMinTraversalTime() + this->m_edges[exiting_edge_indo]->cgetMinTraversalTime()),
                                                          this->m_edges[entering_edge_indo]->cgetTraversalCharge() + this->m_edges[exiting_edge_indo]->cgetTraversalCharge()));
        this->m_edges.back()->getFromNode()->replaceExitingEdge(this->m_edges[entering_edge_indo].get(), this->m_edges.back().get());
        this->m_edges.back()->getToNode()->replaceEnteringEdge( this->m_edges[exiting_edge_indo]. get(), this->m_edges.back().get());

        this->m_nodes[node_ind].          reset();
        this->m_edges[exiting_edge_indz]. reset();
        this->m_edges[exiting_edge_indo]. reset();
        this->m_edges[entering_edge_indz].reset();
        this->m_edges[entering_edge_indo].reset();
      }
    }
  }
  // Clear empty nodes and edges
  this->m_nodes.erase(std::remove_if(this->m_nodes.begin(), this->m_nodes.end(),
                                     [] (const std::unique_ptr<Node>& node) -> bool { return node.get() == nullptr; }),
                      this->m_nodes.end());
  this->m_edges.erase(std::remove_if(this->m_edges.begin(), this->m_edges.end(),
                                     [] (const std::unique_ptr<Edge>& edge) -> bool { return edge.get() == nullptr; }),
                      this->m_edges.end());
  // Update indexes
                                       nodes_count = this->numberOriginalNodes();
  const uint32_t                       edges_count = this->numberOriginalEdges();
  const boost::integer_range<uint32_t> node_inds(0, nodes_count);
  std::for_each(node_inds.begin(), node_inds.end(),
                [this] (const uint32_t node_ind) -> void { this->m_nodes[node_ind]->setGraphIndex(node_ind); });
  for(uint32_t edge_ind = 0; edge_ind < edges_count; ++edge_ind)
  {
    this->m_edges[edge_ind]->setGraphIndex(edge_ind);
  }
  this->m_nodes.shrink_to_fit();
  this->m_edges.shrink_to_fit();

  #ifndef NDEBUG
    for(uint32_t node_ind = 0; node_ind < nodes_count; ++node_ind)
    {
      assert(node_ind == this->m_nodes[node_ind]->cgetGraphIndex());
    }
    for(uint32_t edge_ind = 0; edge_ind < this->numberOriginalEdges(); ++edge_ind)
    {
      assert(edge_ind == this->m_edges[edge_ind]->cgetGraphIndex());
    }
    for(uint32_t edge_ind = 0; edge_ind < this->numberOriginalEdges(); ++edge_ind)
    {
      for(uint32_t other_edge_ind = 0; other_edge_ind < this->numberOriginalEdges(); ++other_edge_ind)
      {
        if((this->cgetOriginalEdges()[edge_ind]->cgetFromNode() == this->cgetOriginalEdges()[other_edge_ind]->cgetToNode()) and
           (this->cgetOriginalEdges()[edge_ind]->cgetToNode()   == this->cgetOriginalEdges()[other_edge_ind]->cgetFromNode()))
        {
          assert((this->cgetOriginalEdges()[edge_ind]->cgetTraversalCharge() + this->cgetOriginalEdges()[other_edge_ind]->cgetTraversalCharge()) >= 0);
        }
      }
    }
  #endif

  /// Generate simplified graph
  this->m_simple_nodes.resize(nodes_count);
  this->m_simple_edges.reserve(std::ceil(this->numberOriginalEdges()*1.5));

  // Copy original graph over without dead end nodes
  std::for_each(node_inds.begin(), node_inds.end(),
                [this] (const uint32_t ind) -> void
                {
                  this->m_simple_nodes[ind].reset(new Node(this->cgetOriginalNodes()[ind]->cgetPosition(),
                                                           Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,Node::EIG_OPTIONS,Node::MAX_CONNECTED_EDGES,1>(),
                                                           Eigen::Matrix<EdgeBase*,Eigen::Dynamic,1,Node::EIG_OPTIONS,Node::MAX_CONNECTED_EDGES,1>(),
                                                           ind));
                });
  for(uint32_t node_ind = 0; node_ind < nodes_count; ++node_ind)
  {
    for(auto out_edge_it = this->cgetOriginalNodes()[node_ind]->cgetExitingEdges().cbegin();
        out_edge_it != this->cgetOriginalNodes()[node_ind]->cgetExitingEdges().cend();
        ++out_edge_it)
    {
      const uint32_t to_node_ind = (*out_edge_it)->cgetToNode()->cgetGraphIndex();

      this->m_simple_edges.emplace_back(std::make_unique<Edge>(this->m_simple_nodes[node_ind].   get(),
                                                               this->m_simple_nodes[to_node_ind].get(),
                                                               this->m_simple_edges.size(),
                                                               static_cast<Edge*>(*out_edge_it)->cgetLength(),
                                                               static_cast<Edge*>(*out_edge_it)->cgetSpeedLimit(),
                                                               (*out_edge_it)->                  cgetTraversalCharge()));
      this->m_simple_nodes[node_ind]->   addExitingEdge( this->m_simple_edges.back().get());
      this->m_simple_nodes[to_node_ind]->addEnteringEdge(this->m_simple_edges.back().get());
    }
  }
  // Remove redundant nodes
  for(uint32_t node_ind = 1; node_ind < nodes_count; ++node_ind)
  {
    // Test for being redundant
    if((1 == this->m_simple_nodes[node_ind]->numberExitingEdges())  and
       (1 == this->m_simple_nodes[node_ind]->numberEnteringEdges()) and
       (this->m_simple_nodes[node_ind]->cgetEnteringEdges()[0]->cgetFromNode() != this->m_simple_nodes[node_ind]->cgetExitingEdges()[0]->cgetToNode()))
    {
      const uint32_t exiting_edge_ind  = this->m_simple_nodes[node_ind]->cgetExitingEdges() [0]->cgetGraphIndex();
      const uint32_t entering_edge_ind = this->m_simple_nodes[node_ind]->cgetEnteringEdges()[0]->cgetGraphIndex();

      this->m_simple_edges.emplace_back(std::make_unique<SuperEdge>(this->m_simple_edges[entering_edge_ind].release(),
                                                                    this->m_simple_edges[exiting_edge_ind]. release(),
                                                                    this->m_simple_edges.size()));
      this->m_simple_edges.back()->getFromNode()->replaceExitingEdge(static_cast<SuperEdge*>(this->m_simple_edges.back().get())->cgetSubEdges().first.get(),
                                                                     this->m_simple_edges.back().get());
      this->m_simple_edges.back()->getToNode()->replaceEnteringEdge(static_cast<SuperEdge*>(this->m_simple_edges.back().get())->cgetSubEdges().second.get(),
                                                                    this->m_simple_edges.back().get());
      this->m_skipped_nodes.push_front(std::move(std::unique_ptr<Node>(this->m_simple_nodes[node_ind].release())));
      this->m_skipped_nodes.front()->setGraphIndex(-1);
    }
    else if((2 == this->m_simple_nodes[node_ind]->numberExitingEdges()) and
            (2 == this->m_simple_nodes[node_ind]->numberEnteringEdges()))
    {
      const bool zz = this->m_simple_nodes[node_ind]->cgetExitingEdges()[0]->cgetToNode() == this->m_simple_nodes[node_ind]->cgetEnteringEdges()[0]->cgetFromNode();
      const bool zo = this->m_simple_nodes[node_ind]->cgetExitingEdges()[0]->cgetToNode() == this->m_simple_nodes[node_ind]->cgetEnteringEdges()[1]->cgetFromNode();
      const bool oz = this->m_simple_nodes[node_ind]->cgetExitingEdges()[1]->cgetToNode() == this->m_simple_nodes[node_ind]->cgetEnteringEdges()[0]->cgetFromNode();
      const bool oo = this->m_simple_nodes[node_ind]->cgetExitingEdges()[1]->cgetToNode() == this->m_simple_nodes[node_ind]->cgetEnteringEdges()[1]->cgetFromNode();

      if((zz and zo) or (oz and oo)) { continue; }

      if(zz and oo)
      {
        const uint32_t exiting_edge_indz  = this->m_simple_nodes[node_ind]->cgetExitingEdges() [0]->cgetGraphIndex();
        const uint32_t entering_edge_indz = this->m_simple_nodes[node_ind]->cgetEnteringEdges()[0]->cgetGraphIndex();
        const uint32_t exiting_edge_indo  = this->m_simple_nodes[node_ind]->cgetExitingEdges() [1]->cgetGraphIndex();
        const uint32_t entering_edge_indo = this->m_simple_nodes[node_ind]->cgetEnteringEdges()[1]->cgetGraphIndex();

        // One way
        this->m_simple_edges.emplace_back(std::make_unique<SuperEdge>(this->m_simple_edges[entering_edge_indz].release(),
                                                                      this->m_simple_edges[exiting_edge_indo]. release(),
                                                                      this->m_simple_edges.size()));
        this->m_simple_edges.back()->getFromNode()->replaceExitingEdge(static_cast<SuperEdge*>(this->m_simple_edges.back().get())->cgetSubEdges().first.get(),
                                                                       this->m_simple_edges.back().get());
        this->m_simple_edges.back()->getToNode()->replaceEnteringEdge(static_cast<SuperEdge*>(this->m_simple_edges.back().get())->cgetSubEdges().second.get(),
                                                                      this->m_simple_edges.back().get());
        // The other way
        this->m_simple_edges.emplace_back(std::make_unique<SuperEdge>(this->m_simple_edges[entering_edge_indo].release(),
                                                                      this->m_simple_edges[exiting_edge_indz]. release(),
                                                                      this->m_simple_edges.size()));
        this->m_simple_edges.back()->getFromNode()->replaceExitingEdge(static_cast<SuperEdge*>(this->m_simple_edges.back().get())->cgetSubEdges().first.get(),
                                                                       this->m_simple_edges.back().get());
        this->m_simple_edges.back()->getToNode()->replaceEnteringEdge(static_cast<SuperEdge*>(this->m_simple_edges.back().get())->cgetSubEdges().second.get(),
                                                                      this->m_simple_edges.back().get());
        this->m_skipped_nodes.push_front(std::move(std::unique_ptr<Node>(this->m_simple_nodes[node_ind].release())));
        this->m_skipped_nodes.front()->setGraphIndex(-1);
      }
      else if(zo and oz)
      {
        const uint32_t exiting_edge_indz  = this->m_simple_nodes[node_ind]->cgetExitingEdges() [0]->cgetGraphIndex();
        const uint32_t entering_edge_indz = this->m_simple_nodes[node_ind]->cgetEnteringEdges()[0]->cgetGraphIndex();
        const uint32_t exiting_edge_indo  = this->m_simple_nodes[node_ind]->cgetExitingEdges() [1]->cgetGraphIndex();
        const uint32_t entering_edge_indo = this->m_simple_nodes[node_ind]->cgetEnteringEdges()[1]->cgetGraphIndex();

        // One way
        this->m_simple_edges.emplace_back(std::make_unique<SuperEdge>(this->m_simple_edges[entering_edge_indz].release(),
                                                                      this->m_simple_edges[exiting_edge_indz]. release(),
                                                                      this->m_simple_edges.size()));
        this->m_simple_edges.back()->getFromNode()->replaceExitingEdge(static_cast<SuperEdge*>(this->m_simple_edges.back().get())->cgetSubEdges().first.get(),
                                                                       this->m_simple_edges.back().get());
        this->m_simple_edges.back()->getToNode()->replaceEnteringEdge(static_cast<SuperEdge*>(this->m_simple_edges.back().get())->cgetSubEdges().second.get(),
                                                                      this->m_simple_edges.back().get());
        // The other way
        this->m_simple_edges.emplace_back(std::make_unique<SuperEdge>(this->m_simple_edges[entering_edge_indo].release(),
                                                                      this->m_simple_edges[exiting_edge_indo]. release(),
                                                                      this->m_simple_edges.size()));
        this->m_simple_edges.back()->getFromNode()->replaceExitingEdge(static_cast<SuperEdge*>(this->m_simple_edges.back().get())->cgetSubEdges().first.get(),
                                                                       this->m_simple_edges.back().get());
        this->m_simple_edges.back()->getToNode()->replaceEnteringEdge(static_cast<SuperEdge*>(this->m_simple_edges.back().get())->cgetSubEdges().second.get(),
                                                                      this->m_simple_edges.back().get());
        this->m_skipped_nodes.push_front(std::move(std::unique_ptr<Node>(this->m_simple_nodes[node_ind].release())));
        this->m_skipped_nodes.front()->setGraphIndex(-1);
      }
    }
//    #ifndef NDEBUG
//      std::vector<EdgeBase*> used_edges;
//      used_edges.reserve(this->numberSimplifiedEdges());
//      for(uint32_t node_ind = 0; node_ind < nodes_count; ++node_ind)
//      {
//        // If empty
//        if(not this->m_simple_nodes[node_ind]) { continue; }
//        assert(not std::any_of(std::execution::par_unseq,
//                               this->m_simple_nodes[node_ind]->cgetExitingEdges().cbegin(),
//                               this->m_simple_nodes[node_ind]->cgetExitingEdges().cend(),
//                               [this,node_ind] (const EdgeBase* const exit_edge) -> bool
//                               {
//                                 return std::any_of(std::execution::par_unseq,
//                                                    this->m_simple_nodes[node_ind]->cgetEnteringEdges().cbegin(),
//                                                    this->m_simple_nodes[node_ind]->cgetEnteringEdges().cend(),
//                                                    [exit_edge] (const EdgeBase* const in_edge) -> bool
//                                                    { return exit_edge == in_edge; });
//                              }));
//        assert(not std::any_of(std::execution::par_unseq,
//                               this->m_simple_nodes[node_ind]->cgetExitingEdges().cbegin(),
//                               this->m_simple_nodes[node_ind]->cgetExitingEdges().cend(),
//                               [this,used_edges] (const EdgeBase* const exit_edge) -> bool
//                               {
//                                 return std::any_of(std::execution::par_unseq, used_edges.cbegin(), used_edges.cend(),
//                                                    [exit_edge] (const EdgeBase* const used_edge) -> bool
//                                                    { return exit_edge == used_edge; });
//                               }));
//        used_edges.insert(used_edges.end(), this->m_simple_nodes[node_ind]->cgetExitingEdges().cbegin(), this->m_simple_nodes[node_ind]->cgetExitingEdges().cend());
//      }
//    #endif
  }
  // Clear empty nodes and edges
  this->m_simple_nodes.erase(std::remove_if(std::execution::unseq, this->m_simple_nodes.begin(), this->m_simple_nodes.end(),
                                            [] (const std::unique_ptr<Node>& node) -> bool { return node.get() == nullptr; }),
                             this->m_simple_nodes.end());
  this->m_simple_edges.erase(std::remove_if(std::execution::unseq, this->m_simple_edges.begin(), this->m_simple_edges.end(),
                                            [] (const std::unique_ptr<EdgeBase>& edge) -> bool { return edge.get() == nullptr; }),
                             this->m_simple_edges.end());
  // Update indexes
  const uint32_t                       num_simple_nodes = this->numberSimplifiedNodes();
  const uint32_t                       num_simple_edges = this->numberSimplifiedEdges();
  const boost::integer_range<uint32_t> simple_node_inds(0, num_simple_nodes);
  std::for_each(std::execution::unseq, simple_node_inds.begin(), simple_node_inds.end(),
                [this] (const uint32_t node_ind) -> void { this->m_simple_nodes[node_ind]->setGraphIndex(node_ind); });
  for(uint32_t edge_ind = 0; edge_ind < num_simple_edges; ++edge_ind)
  {
    this->m_simple_edges[edge_ind]->setGraphIndex(edge_ind);
  }
  this->m_simple_nodes.shrink_to_fit();
  this->m_simple_edges.shrink_to_fit();

  if(need_min_paths)
  {
    /// Generate minimum travel time and charge trajectories to and from all simplified nodes
    const boost::integer_range<uint32_t> orig_node_inds(0, nodes_count);
    const uint32_t                       num_orig_edges = this->numberOriginalEdges();
    this->m_minimal_paths.resize(nodes_count, nodes_count);
    std::for_each(std::execution::par_unseq, orig_node_inds.begin(), orig_node_inds.end(),
    [this,&orig_node_inds,nodes_count,num_orig_edges] (const uint32_t start_ind) -> void
    {
      // Initialize
      std::for_each(std::execution::unseq, orig_node_inds.begin(), orig_node_inds.end(),
      [this,start_ind,num_orig_edges] (const uint32_t target_ind) -> void
      {
        MinimalPath* const cur_min_path = &this->m_minimal_paths(start_ind, target_ind);
        if(start_ind != target_ind)
        {
          cur_min_path->min_traversal_time = std::numeric_limits<float>::infinity();
          cur_min_path->min_charge_used    = std::numeric_limits<float>::infinity();
          cur_min_path->min_charge_needed  = std::numeric_limits<float>::infinity();
          cur_min_path->min_length         = std::numeric_limits<float>::infinity();
        }
        else
        {
          cur_min_path->min_traversal_time                = 0;
          cur_min_path->min_charge_used                   = 0;
          cur_min_path->min_charge_needed                 = 0;
          cur_min_path->min_length                        = 0;
          cur_min_path->charge_used_on_min_time_path      = 0;
          cur_min_path->charge_needed_for_min_time        = 0;
          cur_min_path->length_of_min_time_path           = 0;
          cur_min_path->time_of_min_charge_path           = 0;
          cur_min_path->charge_needed_for_min_charge      = 0;
          cur_min_path->length_of_min_charge_path         = 0;
          cur_min_path->time_of_min_charge_needed         = 0;
          cur_min_path->charge_used_for_min_charge_needed = 0;
          cur_min_path->length_of_min_charge_needed_path  = 0;
          cur_min_path->time_of_min_length_path           = 0;
          cur_min_path->charge_used_on_min_length_path    = 0;
          cur_min_path->charge_needed_for_min_length      = 0;
        }
      });
      // Plan min time
      std::unordered_set<uint32_t> to_process(orig_node_inds.begin(), orig_node_inds.end()); // col index
      while(not to_process.empty())
      {
        // Pop best
        const auto min_it = std::min_element(std::execution::unseq, to_process.cbegin(), to_process.cend(),
                  [this,start_ind] (const uint32_t first, const uint32_t second) -> bool
                  { return this->m_minimal_paths(start_ind, first).min_traversal_time < this->m_minimal_paths(start_ind, second).min_traversal_time; });
        assert(min_it != to_process.cend());
        const uint32_t best_ind = *min_it;
        to_process.erase(min_it);
        // For each child
        const auto child_edges_end = this->cgetOriginalNodes()[best_ind]->cgetExitingEdges().cend();
        for(auto child_edge_it = this->cgetOriginalNodes()[best_ind]->cgetExitingEdges().cbegin();
            child_edge_it != child_edges_end;
            ++child_edge_it)
        {
          const uint32_t child_ind = (*child_edge_it)->cgetToNode()->cgetGraphIndex();
          // If in to process queue
//          if(std::any_of(std::execution::unseq, to_process.cbegin(), to_process.cend(),
//                         [child_ind] (const uint32_t to_process_ind) -> bool { return to_process_ind == child_ind; }))
          if(to_process.cend() != to_process.find(child_ind))
          {
            const float pos_cost = this->m_minimal_paths(start_ind, best_ind).min_traversal_time + (*child_edge_it)->cgetMinTraversalTime();
            if(pos_cost < this->m_minimal_paths(start_ind, child_ind).min_traversal_time) // If it's an improvement
            {
              this->m_minimal_paths(start_ind, child_ind).min_traversal_time           = pos_cost;
              this->m_minimal_paths(start_ind, child_ind).charge_used_on_min_time_path = this->m_minimal_paths(start_ind, best_ind).charge_used_on_min_time_path + (*child_edge_it)->cgetTraversalCharge();
              this->m_minimal_paths(start_ind, child_ind).charge_needed_for_min_time   = std::max<float>(this->m_minimal_paths(start_ind, best_ind).charge_needed_for_min_time,
                                                                                                         this->m_minimal_paths(start_ind, child_ind).charge_used_on_min_time_path);
              this->m_minimal_paths(start_ind, child_ind).length_of_min_time_path      = this->m_minimal_paths(start_ind, best_ind).length_of_min_time_path + (*child_edge_it)->cgetLength();
            }
          }
        }
      }
      // Plan min charge needed
      to_process = std::move(std::unordered_set<uint32_t>(orig_node_inds.begin(), orig_node_inds.end())); // col index
      while(not to_process.empty())
      {
        // Pop best
        const auto min_it = std::min_element(std::execution::unseq, to_process.cbegin(), to_process.cend(),
                  [this,start_ind] (const uint32_t first, const uint32_t second) -> bool
                  { return this->m_minimal_paths(start_ind, first).min_charge_needed < this->m_minimal_paths(start_ind, second).min_charge_needed; });
        assert(min_it != to_process.cend());
        const uint32_t best_ind = *min_it;
        to_process.erase(min_it);
        // For each child
        const auto child_edges_end = this->cgetOriginalNodes()[best_ind]->cgetExitingEdges().cend();
        for(auto child_edge_it = this->cgetOriginalNodes()[best_ind]->cgetExitingEdges().cbegin();
            child_edge_it != child_edges_end;
            ++child_edge_it)
        {
          const uint32_t child_ind = (*child_edge_it)->cgetToNode()->cgetGraphIndex();
          // If in to process queue
//          if(std::any_of(std::execution::unseq, to_process.cbegin(), to_process.cend(),
//                         [child_ind] (const uint32_t to_process_ind) -> bool { return to_process_ind == child_ind; }))
          if(to_process.cend() != to_process.find(child_ind))
          {
            const float pos_charge_used = this->m_minimal_paths(start_ind, best_ind).charge_used_for_min_charge_needed + (*child_edge_it)->cgetTraversalCharge();
            const float pos_cost = std::max<float>(this->m_minimal_paths(start_ind, best_ind).min_charge_needed, pos_charge_used);
            if(pos_cost < this->m_minimal_paths(start_ind, child_ind).min_charge_needed) // If it's an improvement
            {
              this->m_minimal_paths(start_ind, child_ind).min_charge_needed                 = pos_cost;
              this->m_minimal_paths(start_ind, child_ind).time_of_min_charge_needed         = this->m_minimal_paths(start_ind, best_ind).time_of_min_charge_needed + (*child_edge_it)->cgetMinTraversalTime();
              this->m_minimal_paths(start_ind, child_ind).charge_used_for_min_charge_needed = pos_charge_used;
              this->m_minimal_paths(start_ind, child_ind).length_of_min_charge_needed_path  = this->m_minimal_paths(start_ind, best_ind).length_of_min_charge_needed_path + (*child_edge_it)->cgetLength();
            }
          }
        }
      }
      // Plan min charge
      bool improvement_made;
      for(uint32_t relax_it = 0; relax_it < nodes_count; ++relax_it)
      {
        improvement_made = false;

        std::for_each(this->cgetOriginalEdges().cbegin(), this->cgetOriginalEdges().cend(),
          [&improvement_made,this,start_ind] (const std::unique_ptr<Edge>& edge) -> void
          {
            const float new_charge_used = this->m_minimal_paths(start_ind, edge->cgetFromNode()->cgetGraphIndex()).min_charge_used + edge->cgetTraversalCharge();
            if(new_charge_used < this->m_minimal_paths(start_ind, edge->cgetToNode()->cgetGraphIndex()).min_charge_used)
            {
              this->m_minimal_paths(start_ind, edge->cgetToNode()->cgetGraphIndex()).min_charge_used              = new_charge_used;
              this->m_minimal_paths(start_ind, edge->cgetToNode()->cgetGraphIndex()).time_of_min_charge_path      = this->m_minimal_paths(start_ind, edge->cgetFromNode()->cgetGraphIndex()).time_of_min_charge_path + edge->cgetMinTraversalTime();
              this->m_minimal_paths(start_ind, edge->cgetToNode()->cgetGraphIndex()).charge_needed_for_min_charge = std::max<float>(this->m_minimal_paths(start_ind, edge->cgetFromNode()->cgetGraphIndex()).charge_needed_for_min_charge,
                                                                                                                                    new_charge_used);
              this->m_minimal_paths(start_ind, edge->cgetToNode()->cgetGraphIndex()).length_of_min_charge_path    = this->m_minimal_paths(start_ind, edge->cgetFromNode()->cgetGraphIndex()).length_of_min_charge_path + edge->cgetLength();

              improvement_made = true;
            }
          });

        if(not improvement_made)
        {
          break;
        }
      }
      // Check for negative-weight cycles
      if(improvement_made)
      {
        throw std::runtime_error("This street graph has negative-charge-used cycles. Finding minimum charge paths is impossible.");
      }
      // Plan min length
      to_process = std::move(std::unordered_set<uint32_t>(orig_node_inds.begin(), orig_node_inds.end())); // col index
      while(not to_process.empty())
      {
        // Pop best
        const auto min_it = std::min_element(std::execution::unseq, to_process.cbegin(), to_process.cend(),
                  [this,start_ind] (const uint32_t first, const uint32_t second) -> bool
                  { return this->m_minimal_paths(start_ind, first).min_length < this->m_minimal_paths(start_ind, second).min_length; });
        assert(min_it != to_process.cend());
        const uint32_t best_ind = *min_it;
        to_process.erase(min_it);
        // For each child
        const auto child_edges_end = this->cgetOriginalNodes()[best_ind]->cgetExitingEdges().cend();
        for(auto child_edge_it = this->cgetOriginalNodes()[best_ind]->cgetExitingEdges().cbegin();
            child_edge_it != child_edges_end;
            ++child_edge_it)
        {
          const uint32_t child_ind = (*child_edge_it)->cgetToNode()->cgetGraphIndex();
          // If in to process queue
//          if(std::any_of(std::execution::unseq, to_process.cbegin(), to_process.cend(),
//                         [child_ind] (const uint32_t to_process_ind) -> bool { return to_process_ind == child_ind; }))
          if(to_process.cend() != to_process.find(child_ind))
          {
            const float pos_cost = this->m_minimal_paths(start_ind, best_ind).min_length + (*child_edge_it)->cgetLength();
            if(pos_cost < this->m_minimal_paths(start_ind, child_ind).min_length) // If it's an improvement
            {
              this->m_minimal_paths(start_ind, child_ind).min_length                     = pos_cost;
              this->m_minimal_paths(start_ind, child_ind).charge_used_on_min_length_path = this->m_minimal_paths(start_ind, best_ind).charge_used_on_min_length_path + (*child_edge_it)->cgetTraversalCharge();
              this->m_minimal_paths(start_ind, child_ind).charge_needed_for_min_length   = std::max<float>(this->m_minimal_paths(start_ind, best_ind).charge_needed_for_min_length,
                                                                                                           this->m_minimal_paths(start_ind, child_ind).charge_used_on_min_length_path);
              this->m_minimal_paths(start_ind, child_ind).time_of_min_length_path        = this->m_minimal_paths(start_ind, best_ind).time_of_min_length_path + (*child_edge_it)->cgetMinTraversalTime();
            }
          }
        }
      }
    });
    #ifndef NDEBUG
      std::for_each(orig_node_inds.begin(), orig_node_inds.end(),
      [&] (const uint32_t start_ind) -> void
      {
        std::for_each(orig_node_inds.begin(), orig_node_inds.end(),
        [&] (const uint32_t target_ind) -> void
        {
          assert(this->m_minimal_paths(start_ind, target_ind).min_traversal_time < std::numeric_limits<float>::infinity());
          assert(this->m_minimal_paths(start_ind, target_ind).min_charge_used    < std::numeric_limits<float>::infinity());
          assert(this->m_minimal_paths(start_ind, target_ind).min_charge_needed  < std::numeric_limits<float>::infinity());
          assert(this->m_minimal_paths(start_ind, target_ind).min_length         < std::numeric_limits<float>::infinity());
        });
      });
    #endif
  }
}

std::vector<const EdgeBase*> Graph::findEdgesBetweenSimpleNodes(const uint32_t from_node_ind, const uint32_t to_node_ind) const noexcept
{
  std::vector<const EdgeBase*> output;
  output.reserve(2);

  auto edge_it = std::find_if(std::execution::par_unseq,
                              this->cgetSimplifiedEdges().cbegin(),
                              this->cgetSimplifiedEdges().cend(),
                              [from_node_ind, to_node_ind] (const std::unique_ptr<EdgeBase>& edge_it) -> bool
                              {
                                return (edge_it->cgetFromNode()->cgetGraphIndex() == from_node_ind) and
                                       (edge_it->cgetToNode()->  cgetGraphIndex() == to_node_ind);
                              });
  while(this->cgetSimplifiedEdges().cend() != edge_it)
  {
    output.emplace_back(edge_it->get());
    edge_it = std::find_if(std::execution::par_unseq,
                           std::next(edge_it),
                           this->cgetSimplifiedEdges().cend(),
                           [from_node_ind, to_node_ind] (const std::unique_ptr<EdgeBase>& edge_it) -> bool
                           {
                             return (edge_it->cgetFromNode()->cgetGraphIndex() == from_node_ind) and
                                    (edge_it->cgetToNode()->  cgetGraphIndex() == to_node_ind);
                           });
  }

  return output;
}

Eigen::Matrix<float,3,2> Graph::boundingBoxOriginal() const noexcept
{
  Eigen::Matrix<float,3,2> output;
  const boost::integer_range<uint32_t> node_inds(0, this->numberOriginalNodes());
  for(uint32_t dim_it = 0; dim_it < 3; ++dim_it)
  {
    const auto min_max = std::minmax_element(std::execution::par_unseq, node_inds.begin(), node_inds.end(),
                                             [dim_it,this] (const uint32_t i, const uint32_t j) -> bool
                                             { return this->cgetOriginalNodes()[i]->cgetPosition()[dim_it] < this->cgetOriginalNodes()[j]->cgetPosition()[dim_it]; });
    output(dim_it, 0) = this->cgetOriginalNodes()[*min_max.first]-> cgetPosition()[dim_it];
    output(dim_it, 1) = this->cgetOriginalNodes()[*min_max.second]->cgetPosition()[dim_it];
  }
  return output;
}

Eigen::Matrix<float,3,2> Graph::boundingBoxSimplified() const noexcept
{
  Eigen::Matrix<float,3,2> output;
  const boost::integer_range<uint32_t> node_inds(0, this->numberSimplifiedNodes());
  for(uint32_t dim_it = 0; dim_it < 3; ++dim_it)
  {
    const auto min_max = std::minmax_element(std::execution::par_unseq, node_inds.begin(), node_inds.end(),
                                             [dim_it,this] (const uint32_t i, const uint32_t j) -> bool
                                             { return this->cgetSimplifiedNodes()[i]->cgetPosition()[dim_it] < this->cgetSimplifiedNodes()[j]->cgetPosition()[dim_it]; });
    output(dim_it, 0) = this->cgetSimplifiedNodes()[*min_max.first]-> cgetPosition()[dim_it];
    output(dim_it, 1) = this->cgetSimplifiedNodes()[*min_max.second]->cgetPosition()[dim_it];
  }
  return output;
}

double Graph::totalEdgeLength() const noexcept
{
  double output(0);

  std::for_each(this->cgetSimplifiedEdges().cbegin(), this->cgetSimplifiedEdges().cend(),
                [&output] (const std::unique_ptr<EdgeBase>& edge) -> void
  {
    output += edge->cgetLength();
  });

  return output;
}

void Graph::edgesToFile(const std::string& base_dir, const double max_area_for_straight) const
{
  // Finds the area of a triangle defined by three points
  const auto triangle_area_func =
  [] (const Eigen::Ref<const Eigen::Matrix<float,3,1>>& A, const Eigen::Ref<const Eigen::Matrix<float,3,1>>& B, const Eigen::Ref<const Eigen::Matrix<float,3,1>>& C) -> float
  {
    return std::fabs((A[0] * (B[1] - C[1])) + (B[0] * (C[1] - A[1])) + (C[0] * (A[1] - B[1]))) / double(2);
  };
  const uint32_t num_edges = this->numberSimplifiedEdges();
  // Make base directory
  std::filesystem::create_directories(base_dir);
  // Find streets, simple edge ind, start node, target node
  std::vector<std::tuple<uint32_t,const Node*,const Node*,Eigen::Matrix<float,3,Eigen::Dynamic>>> streets;
  streets.reserve(num_edges);
  for(uint32_t edge_ind = 0; edge_ind < num_edges; ++edge_ind)
  {
    if(not std::any_of(std::execution::par_unseq, streets.cbegin(), streets.cend(),
           [&] (const std::tuple<uint32_t,const Node*, const Node*,Eigen::Matrix<float,3,Eigen::Dynamic>>& street) -> bool
           {
             return ((std::get<1>(street) == this->cgetSimplifiedEdges()[edge_ind]->cgetFromNode()) and (std::get<2>(street) == this->cgetSimplifiedEdges()[edge_ind]->cgetToNode())) or
                    ((std::get<2>(street) == this->cgetSimplifiedEdges()[edge_ind]->cgetFromNode()) and (std::get<1>(street) == this->cgetSimplifiedEdges()[edge_ind]->cgetToNode()));
           }))
    {
      Eigen::Matrix<float,3,Eigen::Dynamic> edge_points;
      std::vector<float>                    temp;
      this->cgetSimplifiedEdges()[edge_ind]->generateStraightSegments(edge_points, temp);
      streets.emplace_back(edge_ind, this->cgetSimplifiedEdges()[edge_ind]->cgetFromNode(), this->cgetSimplifiedEdges()[edge_ind]->cgetToNode(), edge_points);
    }
  }
  // Loop through edges
  uint32_t street_ind = 0;
  while(not streets.empty())
  {
    // Get next street
    Eigen::Matrix<float,3,Eigen::Dynamic> edge_points = std::get<3>(streets.back());
    streets.pop_back();
    // Find preppend streets
    {
      const auto new_end = std::remove_if(std::execution::par_unseq, streets.begin(), streets.end(),
      [&] (const std::tuple<uint32_t,const Node*,const Node*,Eigen::Matrix<float,3,Eigen::Dynamic>>& street) -> bool
      {
        return max_area_for_straight > triangle_area_func(std::get<3>(street).col(std::get<3>(street).cols() - 2),
                                                          std::get<3>(street).col(std::get<3>(street).cols() - 1),
                                                          edge_points.col(1));
      });
      if(streets.end() != new_end)
      {
        assert(new_end == std::prev(streets.end()));
        Eigen::Matrix<float,3,Eigen::Dynamic> new_edge_points(3, edge_points.cols() + std::get<3>(*new_end).cols() - 1);
        new_edge_points.leftCols(std::get<3>(*new_end).cols()) = std::get<3>(*new_end);
        new_edge_points.rightCols(edge_points.cols()) = edge_points;
        edge_points = new_edge_points;
        streets.pop_back();
      }
    }
    // Find append streets
    {
      const auto new_end = std::remove_if(std::execution::par_unseq, streets.begin(), streets.end(),
      [&] (const std::tuple<uint32_t,const Node*,const Node*,Eigen::Matrix<float,3,Eigen::Dynamic>>& street) -> bool
      {
        return max_area_for_straight > triangle_area_func(edge_points.col(edge_points.cols() - 2),
                                                          edge_points.rightCols<1>(),
                                                          std::get<3>(street).col(1));
      });
      if(streets.end() != new_end)
      {
        assert(new_end == std::prev(streets.end()));
        Eigen::Matrix<float,3,Eigen::Dynamic> new_edge_points(3, edge_points.cols() + std::get<3>(*new_end).cols() - 1);
        new_edge_points.leftCols(edge_points.cols()) = edge_points;
        new_edge_points.rightCols(std::get<3>(*new_end).cols()) = std::get<3>(*new_end);
        edge_points = new_edge_points;
        streets.pop_back();
      }
    }

    // Send to file
    std::ofstream file(base_dir + "/edge_" + std::to_string(street_ind) + ".csv");
    // Header info
    file << "x,y";
    // Edge info
    const Eigen::Index num_points = edge_points.cols();
    for(Eigen::Index point_ind = 0; point_ind < num_points; ++point_ind)
    {
      file << "\n" << edge_points(0, point_ind) << "," << edge_points(1, point_ind);
    }

    file.flush();
    file.close();
  }
}
} // graph

/* graph.cpp */
