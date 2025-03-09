/**
 * @File: planning_graph.cpp
 * @Date: May 2024
 * @Author: James Swedeen
 **/

/* C++ Headers */
#include<utility>
#include<memory>
#include<vector>
#include<unordered_set>
#include<list>
#include<forward_list>
#include<algorithm>
#include<execution>
#include<iostream> // TODO

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<street_graph/graph.hpp>
#include<street_graph/planning_graph.hpp>

graph::PlanningGraph::PlanningGraph(const Graph&                 street_graph,
                                    const std::vector<uint32_t>& hotspot_locations,
                                    const float                  agent_max_soc,
                                    const float                  max_time,
                                    const float                  max_trip_length,
                                    const bool                   using_soc_constraints,
                                    const bool                   using_distance_constraints) noexcept
 : m_ptp_paths(                  hotspot_locations.size() + 1, hotspot_locations.size() + 1),
   m_min_time_ptp_paths(         hotspot_locations.size() + 1, hotspot_locations.size() + 1),
   m_min_charge_ptp_paths(       hotspot_locations.size() + 1, hotspot_locations.size() + 1),
   m_min_charge_needed_ptp_paths(hotspot_locations.size() + 1, hotspot_locations.size() + 1),
   m_min_end_charge_ptp_paths(   hotspot_locations.size() + 1, hotspot_locations.size() + 1),
   m_min_length_ptp_paths(       hotspot_locations.size() + 1, hotspot_locations.size() + 1),
   m_number_vertices(hotspot_locations.size() + 1)
{
  /// Speed up next calculation by merging redundant edges
  const boost::integer_range<uint32_t> orig_edge_inds(0, street_graph.numberOriginalEdges());
  const boost::integer_range<uint32_t> orig_node_inds(0, street_graph.numberOriginalNodes());

  std::vector<std::vector<const Edge*>> simple_edges(          street_graph.numberOriginalEdges());
  std::vector<float>                    simple_edges_time(     street_graph.numberOriginalEdges());
  std::vector<float>                    simple_edges_charge(   street_graph.numberOriginalEdges());
  std::vector<float>                    simple_edges_length(   street_graph.numberOriginalEdges());
  std::vector<std::vector<uint32_t>>    simple_enter_edge_inds(street_graph.numberOriginalNodes());
  std::vector<std::vector<uint32_t>>    simple_exit_edge_inds( street_graph.numberOriginalNodes());

  std::for_each(std::execution::unseq, orig_edge_inds.begin(), orig_edge_inds.end(),
  [&] (const uint32_t edge_ind) -> void
  {
    simple_edges[        edge_ind].reserve(street_graph.numberOriginalEdges());
    simple_edges[        edge_ind].emplace_back(dynamic_cast<const Edge*>(street_graph.cgetOriginalEdges()[edge_ind].get()));
    simple_edges_time[   edge_ind] = street_graph.cgetOriginalEdges()[edge_ind]->cgetMinTraversalTime();
    simple_edges_charge[ edge_ind] = street_graph.cgetOriginalEdges()[edge_ind]->cgetTraversalCharge();
    simple_edges_length[ edge_ind] = street_graph.cgetOriginalEdges()[edge_ind]->cgetLength();
  });
  std::for_each(orig_node_inds.begin(), orig_node_inds.end(),
  [&] (const uint32_t node_ind) -> void
  {
    simple_enter_edge_inds[node_ind].reserve(street_graph.cgetOriginalNodes()[node_ind]->numberEnteringEdges());
    simple_exit_edge_inds[ node_ind].reserve(street_graph.cgetOriginalNodes()[node_ind]->numberExitingEdges());
    std::for_each(street_graph.cgetOriginalNodes()[node_ind]->cgetEnteringEdges().cbegin(),
                  street_graph.cgetOriginalNodes()[node_ind]->cgetEnteringEdges().cend(),
    [&] (const EdgeBase* const enter_edge) -> void
    {
      simple_enter_edge_inds[node_ind].emplace_back(enter_edge->cgetGraphIndex());
    });
    std::for_each(street_graph.cgetOriginalNodes()[node_ind]->cgetExitingEdges().cbegin(),
                  street_graph.cgetOriginalNodes()[node_ind]->cgetExitingEdges().cend(),
    [&] (const EdgeBase* const exit_edge) -> void
    {
      simple_exit_edge_inds[node_ind].emplace_back(exit_edge->cgetGraphIndex());
    });
  });

  for(uint32_t node_ind = 1; node_ind < street_graph.numberOriginalNodes(); ++node_ind)
  {
    if(std::any_of(std::execution::unseq, hotspot_locations.cbegin(), hotspot_locations.cend(),
       [node_ind] (const uint32_t hs_orig_ind) -> bool
       {
         return node_ind == hs_orig_ind;
       }))
    { continue; }

    // Test for being redundant
    if((1 == simple_exit_edge_inds[ node_ind].size()) and
       (1 == simple_enter_edge_inds[node_ind].size()) and
       (simple_edges[simple_enter_edge_inds[node_ind][0]].front()->cgetFromNode() != simple_edges[simple_exit_edge_inds[node_ind][0]].back()->cgetToNode()) and
       (std::signbit(simple_edges[simple_enter_edge_inds[node_ind][0]].front()->cgetTraversalCharge()) == std::signbit(simple_edges[simple_exit_edge_inds[node_ind][0]].front()->cgetTraversalCharge())))
    {
      const uint32_t exiting_edge_ind  = simple_exit_edge_inds[ node_ind][0];
      const uint32_t entering_edge_ind = simple_enter_edge_inds[node_ind][0];

      simple_edges[       entering_edge_ind].insert(simple_edges[entering_edge_ind].end(), simple_edges[exiting_edge_ind].cbegin(), simple_edges[exiting_edge_ind].cend());
      simple_edges_time[  entering_edge_ind] += simple_edges_time[  exiting_edge_ind];
      simple_edges_charge[entering_edge_ind] += simple_edges_charge[exiting_edge_ind];
      simple_edges_length[entering_edge_ind] += simple_edges_length[exiting_edge_ind];
      std::replace(simple_enter_edge_inds[simple_edges[exiting_edge_ind].back()->cgetToNode()->cgetGraphIndex()].begin(),
                   simple_enter_edge_inds[simple_edges[exiting_edge_ind].back()->cgetToNode()->cgetGraphIndex()].end(),
                   exiting_edge_ind,
                   entering_edge_ind);
      simple_enter_edge_inds[node_ind].erase(std::remove_if(simple_enter_edge_inds[node_ind].begin(), simple_enter_edge_inds[node_ind].end(),
                                             [entering_edge_ind] (const uint32_t edge_ind) -> bool { return entering_edge_ind == edge_ind; }));
    }
    else if((2 == simple_exit_edge_inds[ node_ind].size()) and
            (2 == simple_enter_edge_inds[node_ind].size()))
    {
      const bool zz = simple_edges[simple_exit_edge_inds[node_ind][0]].back()->cgetToNode() == simple_edges[simple_enter_edge_inds[node_ind][0]].front()->cgetFromNode();
      const bool zo = simple_edges[simple_exit_edge_inds[node_ind][0]].back()->cgetToNode() == simple_edges[simple_enter_edge_inds[node_ind][1]].front()->cgetFromNode();
      const bool oz = simple_edges[simple_exit_edge_inds[node_ind][1]].back()->cgetToNode() == simple_edges[simple_enter_edge_inds[node_ind][0]].front()->cgetFromNode();
      const bool oo = simple_edges[simple_exit_edge_inds[node_ind][1]].back()->cgetToNode() == simple_edges[simple_enter_edge_inds[node_ind][1]].front()->cgetFromNode();

      if((zz and zo) or (oz and oo)) { continue; }

      const uint32_t exiting_edge_indz  = simple_exit_edge_inds[ node_ind][0];
      const uint32_t entering_edge_indz = simple_enter_edge_inds[node_ind][0];
      const uint32_t exiting_edge_indo  = simple_exit_edge_inds[ node_ind][1];
      const uint32_t entering_edge_indo = simple_enter_edge_inds[node_ind][1];

      if(zz and
         oo and
         (std::signbit(simple_edges[entering_edge_indz].front()->cgetTraversalCharge()) == std::signbit(simple_edges[exiting_edge_indo].front()->cgetTraversalCharge())) and
         (std::signbit(simple_edges[entering_edge_indo].front()->cgetTraversalCharge()) == std::signbit(simple_edges[exiting_edge_indz].front()->cgetTraversalCharge())))
      {
        // One way
        simple_edges[       entering_edge_indz].insert(simple_edges[entering_edge_indz].end(), simple_edges[exiting_edge_indo].cbegin(), simple_edges[exiting_edge_indo].cend());
        simple_edges_time[  entering_edge_indz] += simple_edges_time[  exiting_edge_indo];
        simple_edges_charge[entering_edge_indz] += simple_edges_charge[exiting_edge_indo];
        simple_edges_length[entering_edge_indz] += simple_edges_length[exiting_edge_indo];
        std::replace(simple_enter_edge_inds[simple_edges[exiting_edge_indo].back()->cgetToNode()->cgetGraphIndex()].begin(),
                     simple_enter_edge_inds[simple_edges[exiting_edge_indo].back()->cgetToNode()->cgetGraphIndex()].end(),
                     exiting_edge_indo,
                     entering_edge_indz);
        // The other way
        simple_edges[       entering_edge_indo].insert(simple_edges[entering_edge_indo].end(), simple_edges[exiting_edge_indz].cbegin(), simple_edges[exiting_edge_indz].cend());
        simple_edges_time[  entering_edge_indo] += simple_edges_time[  exiting_edge_indz];
        simple_edges_charge[entering_edge_indo] += simple_edges_charge[exiting_edge_indz];
        simple_edges_length[entering_edge_indo] += simple_edges_length[exiting_edge_indz];
        std::replace(simple_enter_edge_inds[simple_edges[exiting_edge_indz].back()->cgetToNode()->cgetGraphIndex()].begin(),
                     simple_enter_edge_inds[simple_edges[exiting_edge_indz].back()->cgetToNode()->cgetGraphIndex()].end(),
                     exiting_edge_indz,
                     entering_edge_indo);

        simple_enter_edge_inds[node_ind].erase(std::remove_if(simple_enter_edge_inds[node_ind].begin(), simple_enter_edge_inds[node_ind].end(),
                                               [entering_edge_indz,entering_edge_indo] (const uint32_t edge_ind) -> bool { return (entering_edge_indz == edge_ind) or (entering_edge_indo == edge_ind); }),
                                               simple_enter_edge_inds[node_ind].end());
      }
      else if(zo and
              oz and
              (std::signbit(simple_edges[entering_edge_indz].front()->cgetTraversalCharge()) == std::signbit(simple_edges[exiting_edge_indz].front()->cgetTraversalCharge())) and
              (std::signbit(simple_edges[entering_edge_indo].front()->cgetTraversalCharge()) == std::signbit(simple_edges[exiting_edge_indo].front()->cgetTraversalCharge())))
      {
        // One way
        simple_edges[       entering_edge_indz].insert(simple_edges[entering_edge_indz].end(), simple_edges[exiting_edge_indz].cbegin(), simple_edges[exiting_edge_indz].cend());
        simple_edges_time[  entering_edge_indz] += simple_edges_time[  exiting_edge_indz];
        simple_edges_charge[entering_edge_indz] += simple_edges_charge[exiting_edge_indz];
        simple_edges_length[entering_edge_indz] += simple_edges_length[exiting_edge_indz];
        std::replace(simple_enter_edge_inds[simple_edges[exiting_edge_indz].back()->cgetToNode()->cgetGraphIndex()].begin(),
                     simple_enter_edge_inds[simple_edges[exiting_edge_indz].back()->cgetToNode()->cgetGraphIndex()].end(),
                     exiting_edge_indz,
                     entering_edge_indz);
        // The other way
        simple_edges[       entering_edge_indo].insert(simple_edges[entering_edge_indo].end(), simple_edges[exiting_edge_indo].cbegin(), simple_edges[exiting_edge_indo].cend());
        simple_edges_time[  entering_edge_indo] += simple_edges_time[  exiting_edge_indo];
        simple_edges_charge[entering_edge_indo] += simple_edges_charge[exiting_edge_indo];
        simple_edges_length[entering_edge_indo] += simple_edges_length[exiting_edge_indo];
        std::replace(simple_enter_edge_inds[simple_edges[exiting_edge_indo].back()->cgetToNode()->cgetGraphIndex()].begin(),
                     simple_enter_edge_inds[simple_edges[exiting_edge_indo].back()->cgetToNode()->cgetGraphIndex()].end(),
                     exiting_edge_indo,
                     entering_edge_indo);

        simple_enter_edge_inds[node_ind].erase(std::remove_if(simple_enter_edge_inds[node_ind].begin(), simple_enter_edge_inds[node_ind].end(),
                                               [entering_edge_indz,entering_edge_indo] (const uint32_t edge_ind) -> bool { return (entering_edge_indz == edge_ind) or (entering_edge_indo == edge_ind); }),
                                               simple_enter_edge_inds[node_ind].end());
      }
    }
  }
  std::for_each(orig_edge_inds.begin(), orig_edge_inds.end(),
  [&] (const uint32_t edge_ind) -> void
  {
    simple_edges[edge_ind].shrink_to_fit();
  });
  std::for_each(orig_node_inds.begin(), orig_node_inds.end(),
  [&] (const uint32_t node_ind) -> void
  {
    simple_enter_edge_inds[node_ind].shrink_to_fit();
    simple_exit_edge_inds[ node_ind].shrink_to_fit();
  });
  #ifndef NDEBUG
    std::for_each(orig_node_inds.begin(), orig_node_inds.end(),
    [&] (const uint32_t orig_node_ind) -> void
    {
      std::for_each(simple_enter_edge_inds[orig_node_ind].cbegin(), simple_enter_edge_inds[orig_node_ind].cend(),
      [&] (const uint32_t enter_edge_ind) -> void
      {
        assert(orig_node_ind == simple_edges[enter_edge_ind].back()->cgetToNode()->cgetGraphIndex());
      });
      std::for_each(simple_exit_edge_inds[orig_node_ind].cbegin(), simple_exit_edge_inds[orig_node_ind].cend(),
      [&] (const uint32_t exit_edge_ind) -> void
      {
        assert(orig_node_ind == simple_edges[exit_edge_ind].front()->cgetFromNode()->cgetGraphIndex());
      });
    });
    std::for_each(simple_edges.cbegin(), simple_edges.cend(),
    [] (const std::vector<const Edge*>& simple_edge) -> void
    {
      const auto simple_edge_end = simple_edge.cend();
      for(auto edge_it = std::next(simple_edge.cbegin()); edge_it != simple_edge_end; ++edge_it)
      {
        assert((*std::prev(edge_it))->cgetToNode() == (*edge_it)->cgetFromNode());
        assert(std::signbit((*std::prev(edge_it))->cgetTraversalCharge()) == std::signbit((*edge_it)->cgetTraversalCharge()));
      }
    });
  #endif

  std::vector<uint32_t> simple_edges_to_node_inds(street_graph.numberOriginalEdges());
  std::for_each(std::execution::unseq, orig_edge_inds.begin(), orig_edge_inds.end(),
  [&] (const uint32_t orig_edge_ind) -> void
  {
    simple_edges_to_node_inds[orig_edge_ind] = simple_edges[orig_edge_ind].back()->cgetToNode()->cgetGraphIndex();
  });

  /// Find minimal point to point paths
  // Rows indicate the start vertex and columns are the target vertex
  const boost::integer_range<uint32_t> vertex_inds(0, this->numberVertices());
  std::mutex                           paths_mux;

  std::vector<uint32_t> to_orig_inds_map(this->numberVertices());
  to_orig_inds_map[0] = 0;
  for(uint32_t vert_ind = 1; vert_ind < this->numberVertices(); ++vert_ind)
  {
    to_orig_inds_map[vert_ind] = hotspot_locations[vert_ind - 1];
  }
  const auto to_orig_inds_map_end = to_orig_inds_map.cend();
  std::for_each(std::execution::par_unseq, vertex_inds.begin(), vertex_inds.end(),
  [&] (const uint32_t start_ind) -> void
  {
    const uint32_t start_orig_ind = to_orig_inds_map[start_ind];

    std::for_each(std::execution::par_unseq, vertex_inds.begin(), vertex_inds.end(),
    [&] (const uint32_t target_ind) -> void
    {
      if(start_ind == target_ind) { return; }
      this->m_ptp_paths(start_ind, target_ind).reserve(999999);

      const uint32_t target_orig_ind = to_orig_inds_map[target_ind];

      /// Add min time, charge, and charge needed paths to get started
      // Find min time path
      {
        // Original node index, path time, past edges
        std::unordered_set<uint32_t>                           to_process(orig_node_inds.begin(), orig_node_inds.end());
        std::vector<std::pair<float,std::vector<const Edge*>>> info(street_graph.numberOriginalNodes(), std::pair<float,std::vector<const Edge*>>(std::numeric_limits<float>::infinity(), std::vector<const Edge*>()));
        info[start_orig_ind].first = 0;
        while(not to_process.empty())
        {
          // Pop best
          const auto min_it = std::min_element(std::execution::unseq, to_process.cbegin(), to_process.cend(),
                    [this,target_orig_ind,&street_graph,&info]
                    (const uint32_t first, const uint32_t second) -> bool
                    {
                      const float first_cost  = info[first].first  + street_graph.minTravelTime(first,  target_orig_ind);
                      const float second_cost = info[second].first + street_graph.minTravelTime(second, target_orig_ind);
                      return first_cost < second_cost;
                    });
          assert(min_it != to_process.cend());
          const uint32_t best_ind = *min_it;
          if(best_ind == target_orig_ind) { break; }
          to_process.erase(min_it);
          // For each child
          const auto child_edges_end = street_graph.cgetOriginalNodes()[best_ind]->cgetExitingEdges().cend();
          for(auto child_edge_it = street_graph.cgetOriginalNodes()[best_ind]->cgetExitingEdges().cbegin();
              child_edge_it != child_edges_end;
              ++child_edge_it)
          {
            const uint32_t child_ind = (*child_edge_it)->cgetToNode()->cgetGraphIndex();
            // If in to process queue
            if(to_process.cend() != to_process.find(child_ind))
            {
              const float pos_cost = info[best_ind].first + (*child_edge_it)->cgetMinTraversalTime();
              if(pos_cost < info[child_ind].first) // If it's an improvement
              {
                info[child_ind].first = pos_cost;
                info[child_ind].second.clear();
                info[child_ind].second.reserve(info[best_ind].second.size() + 1);
                info[child_ind].second.insert(info[child_ind].second.begin(), info[best_ind].second.cbegin(), info[best_ind].second.cend());
                info[child_ind].second.emplace_back(dynamic_cast<const Edge*>(*child_edge_it));
              }
            }
          }
        }
        // Find path info for best path
        float path_time          = 0;
        float path_charge        = 0;
        float path_length        = 0;
        float path_min_charge    = 0;
        float path_charge_needed = 0;
        float path_end_charge    = 0;
        std::for_each(info[target_orig_ind].second.cbegin(), info[target_orig_ind].second.cend(),
        [&] (const Edge* const path_edge) -> void
        {
          path_time          += path_edge->cgetMinTraversalTime();
          path_charge        += path_edge->cgetTraversalCharge();
          path_length        += path_edge->cgetLength();
          path_charge_needed = std::max<double>(path_charge_needed, path_charge);
          if(path_charge < path_min_charge)
          {
            path_end_charge = 0;
            path_min_charge = path_charge;
          }
          else
          {
            path_end_charge += path_edge->cgetTraversalCharge();
          }
        });
        // Check validity conditions
        const bool soc_constraints_good  = (not using_soc_constraints)      or (path_charge_needed <= agent_max_soc) or (path_end_charge <= agent_max_soc);
        const bool dist_constraints_good = (not using_distance_constraints) or (path_length        <= max_trip_length);
        if((path_time <= max_time) and soc_constraints_good and dist_constraints_good)
        {
          std::lock_guard<std::mutex> lock(paths_mux);
          this->m_ptp_paths(start_ind, target_ind).emplace_back(start_ind,
                                                                target_ind,
                                                                std::move(info[target_orig_ind].second),
                                                                path_time,
                                                                path_charge,
                                                                path_charge_needed,
                                                                path_end_charge,
                                                                path_length);
        }
      }
      // Find minimum charge needed path
      {
        // Original node index, path charge used, path charge needed, past edges
        std::unordered_set<uint32_t>                                  to_process(orig_node_inds.begin(), orig_node_inds.end());
        std::vector<std::tuple<float,float,std::vector<const Edge*>>> info(street_graph.numberOriginalNodes(), std::tuple<float,float,std::vector<const Edge*>>(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), std::vector<const Edge*>()));
        std::get<0>(info[start_orig_ind]) = 0;
        std::get<1>(info[start_orig_ind]) = 0;
        while(not to_process.empty())
        {
          // Pop best
          const auto min_it = std::min_element(std::execution::unseq, to_process.cbegin(), to_process.cend(),
                    [this,target_orig_ind,&street_graph,&info]
                    (const uint32_t first, const uint32_t second) -> bool
                    {
                      return std::get<1>(info[first]) < std::get<1>(info[second]);
                    });
          assert(min_it != to_process.cend());
          const uint32_t best_ind = *min_it;
          if(best_ind == target_orig_ind) { break; }
          to_process.erase(min_it);
          // For each child
          const auto child_edges_end = street_graph.cgetOriginalNodes()[best_ind]->cgetExitingEdges().cend();
          for(auto child_edge_it = street_graph.cgetOriginalNodes()[best_ind]->cgetExitingEdges().cbegin();
              child_edge_it != child_edges_end;
              ++child_edge_it)
          {
            const uint32_t child_ind = (*child_edge_it)->cgetToNode()->cgetGraphIndex();
            // If in to process queue
            if(to_process.cend() != to_process.find(child_ind))
            {
              const float pos_charge_used = std::get<0>(info[best_ind]) + (*child_edge_it)->cgetTraversalCharge();
              const float pos_cost        = std::max<float>(std::get<1>(info[best_ind]), pos_charge_used);
              if(pos_cost < std::get<1>(info[child_ind])) // If it's an improvement
              {
                std::get<0>(info[child_ind]) = pos_charge_used;
                std::get<1>(info[child_ind]) = pos_cost;
                std::get<2>(info[child_ind]).clear();
                std::get<2>(info[child_ind]).reserve(std::get<2>(info[best_ind]).size() + 1);
                std::get<2>(info[child_ind]).insert(std::get<2>(info[child_ind]).begin(), std::get<2>(info[best_ind]).cbegin(), std::get<2>(info[best_ind]).cend());
                std::get<2>(info[child_ind]).emplace_back(dynamic_cast<const Edge*>(*child_edge_it));
              }
            }
          }
        }
        // Find path info for best path
        float path_time          = 0;
        float path_charge        = 0;
        float path_length        = 0;
        float path_min_charge    = 0;
        float path_charge_needed = 0;
        float path_end_charge    = 0;
        std::for_each(std::get<2>(info[target_orig_ind]).cbegin(), std::get<2>(info[target_orig_ind]).cend(),
        [&] (const Edge* const path_edge) -> void
        {
          path_time          += path_edge->cgetMinTraversalTime();
          path_charge        += path_edge->cgetTraversalCharge();
          path_length        += path_edge->cgetLength();
          path_charge_needed = std::max<double>(path_charge_needed, path_charge);
          if(path_charge < path_min_charge)
          {
            path_end_charge = 0;
            path_min_charge = path_charge;
          }
          else
          {
            path_end_charge += path_edge->cgetTraversalCharge();
          }
        });
        // Check validity conditions
        const bool soc_constraints_good  = (not using_soc_constraints)      or (path_charge_needed <= agent_max_soc) or (path_end_charge <= agent_max_soc);
        const bool dist_constraints_good = (not using_distance_constraints) or (path_length        <= max_trip_length);
        if((path_time <= max_time) and soc_constraints_good and dist_constraints_good)
        {
          std::lock_guard<std::mutex> lock(paths_mux);
          this->m_ptp_paths(start_ind, target_ind).emplace_back(start_ind,
                                                                target_ind,
                                                                std::move(std::get<2>(info[target_orig_ind])),
                                                                path_time,
                                                                path_charge,
                                                                path_charge_needed,
                                                                path_end_charge,
                                                                path_length);
        }
      }
      // Find min charge path
      {
        // Original node index, path time, past edges
        std::vector<std::pair<float,std::vector<const Edge*>>> info(street_graph.numberOriginalNodes(), std::pair<float,std::vector<const Edge*>>(std::numeric_limits<float>::infinity(), std::vector<const Edge*>()));
        info[start_orig_ind].first = 0;

        bool improvement_made;
        for(uint32_t relax_it = 0; relax_it < street_graph.numberOriginalNodes(); ++relax_it)
        {
          improvement_made = false;

          std::for_each(street_graph.cgetOriginalEdges().cbegin(), street_graph.cgetOriginalEdges().cend(),
            [&improvement_made,this,start_ind,&info] (const std::unique_ptr<Edge>& edge) -> void
            {
              const float new_charge_used = info[edge->cgetFromNode()->cgetGraphIndex()].first + edge->cgetTraversalCharge();
              if(new_charge_used < info[edge->cgetToNode()->cgetGraphIndex()].first)
              {
                info[edge->cgetToNode()->cgetGraphIndex()].first = new_charge_used;
                info[edge->cgetToNode()->cgetGraphIndex()].second.clear();
                info[edge->cgetToNode()->cgetGraphIndex()].second.reserve(info[edge->cgetFromNode()->cgetGraphIndex()].second.size() + 1);
                info[edge->cgetToNode()->cgetGraphIndex()].second.insert(info[edge->cgetToNode()->  cgetGraphIndex()].second.begin(),
                                                                         info[edge->cgetFromNode()->cgetGraphIndex()].second.cbegin(),
                                                                         info[edge->cgetFromNode()->cgetGraphIndex()].second.cend());
                info[edge->cgetToNode()->cgetGraphIndex()].second.emplace_back(edge.get());

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
        // Find path info for best path
        float path_time          = 0;
        float path_charge        = 0;
        float path_length        = 0;
        float path_min_charge    = 0;
        float path_charge_needed = 0;
        float path_end_charge    = 0;
        std::for_each(info[target_orig_ind].second.cbegin(), info[target_orig_ind].second.cend(),
        [&] (const Edge* const path_edge) -> void
        {
          path_time          += path_edge->cgetMinTraversalTime();
          path_charge        += path_edge->cgetTraversalCharge();
          path_length        += path_edge->cgetLength();
          path_charge_needed = std::max<double>(path_charge_needed, path_charge);
          if(path_charge < path_min_charge)
          {
            path_end_charge = 0;
            path_min_charge = path_charge;
          }
          else
          {
            path_end_charge += path_edge->cgetTraversalCharge();
          }
        });
        // Check validity conditions
        const bool soc_constraints_good  = (not using_soc_constraints)      or (path_charge_needed <= agent_max_soc) or (path_end_charge <= agent_max_soc);
        const bool dist_constraints_good = (not using_distance_constraints) or (path_length        <= max_trip_length);
        if((path_time <= max_time) and soc_constraints_good and dist_constraints_good)
        {
          std::lock_guard<std::mutex> lock(paths_mux);
          this->m_ptp_paths(start_ind, target_ind).emplace_back(start_ind,
                                                                target_ind,
                                                                std::move(info[target_orig_ind].second),
                                                                path_time,
                                                                path_charge,
                                                                path_charge_needed,
                                                                path_end_charge,
                                                                path_length);
        }
      }
      // Find min length path
      {
        // Original node index, path length, past edges
        std::unordered_set<uint32_t>                           to_process(orig_node_inds.begin(), orig_node_inds.end());
        std::vector<std::pair<float,std::vector<const Edge*>>> info(street_graph.numberOriginalNodes(), std::pair<float,std::vector<const Edge*>>(std::numeric_limits<float>::infinity(), std::vector<const Edge*>()));
        info[start_orig_ind].first = 0;
        while(not to_process.empty())
        {
          // Pop best
          const auto min_it = std::min_element(std::execution::unseq, to_process.cbegin(), to_process.cend(),
                    [this,target_orig_ind,&street_graph,&info]
                    (const uint32_t first, const uint32_t second) -> bool
                    {
                      const float first_cost  = info[first].first  + street_graph.minLength(first,  target_orig_ind);
                      const float second_cost = info[second].first + street_graph.minLength(second, target_orig_ind);
                      return first_cost < second_cost;
                    });
          assert(min_it != to_process.cend());
          const uint32_t best_ind = *min_it;
          if(best_ind == target_orig_ind) { break; }
          to_process.erase(min_it);
          // For each child
          const auto child_edges_end = street_graph.cgetOriginalNodes()[best_ind]->cgetExitingEdges().cend();
          for(auto child_edge_it = street_graph.cgetOriginalNodes()[best_ind]->cgetExitingEdges().cbegin();
              child_edge_it != child_edges_end;
              ++child_edge_it)
          {
            const uint32_t child_ind = (*child_edge_it)->cgetToNode()->cgetGraphIndex();
            // If in to process queue
            if(to_process.cend() != to_process.find(child_ind))
            {
              const float pos_cost = info[best_ind].first + (*child_edge_it)->cgetLength();
              if(pos_cost < info[child_ind].first) // If it's an improvement
              {
                info[child_ind].first = pos_cost;
                info[child_ind].second.clear();
                info[child_ind].second.reserve(info[best_ind].second.size() + 1);
                info[child_ind].second.insert(info[child_ind].second.begin(), info[best_ind].second.cbegin(), info[best_ind].second.cend());
                info[child_ind].second.emplace_back(dynamic_cast<const Edge*>(*child_edge_it));
              }
            }
          }
        }
        // Find path info for best path
        float path_time          = 0;
        float path_charge        = 0;
        float path_length        = 0;
        float path_min_charge    = 0;
        float path_charge_needed = 0;
        float path_end_charge    = 0;
        std::for_each(info[target_orig_ind].second.cbegin(), info[target_orig_ind].second.cend(),
        [&] (const Edge* const path_edge) -> void
        {
          path_time          += path_edge->cgetMinTraversalTime();
          path_charge        += path_edge->cgetTraversalCharge();
          path_length        += path_edge->cgetLength();
          path_charge_needed = std::max<float>(path_charge_needed, path_charge);
          if(path_charge < path_min_charge)
          {
            path_end_charge = 0;
            path_min_charge = path_charge;
          }
          else
          {
            path_end_charge += path_edge->cgetTraversalCharge();
          }
        });
        // Check validity conditions
        const bool soc_constraints_good  = (not using_soc_constraints)      or (path_charge_needed <= agent_max_soc) or (path_end_charge <= agent_max_soc);
        const bool dist_constraints_good = (not using_distance_constraints) or (path_length        <= max_trip_length);
        if((path_time <= max_time) and soc_constraints_good and dist_constraints_good)
        {
          std::lock_guard<std::mutex> lock(paths_mux);
          this->m_ptp_paths(start_ind, target_ind).emplace_back(start_ind,
                                                                target_ind,
                                                                std::move(info[target_orig_ind].second),
                                                                path_time,
                                                                path_charge,
                                                                path_charge_needed,
                                                                path_end_charge,
                                                                path_length);
        }
      }

      assert(not this->m_ptp_paths(start_ind, target_ind).empty());
    });
  });

  // Remove any sub-optimal paths
  std::for_each(std::execution::par_unseq, vertex_inds.begin(), vertex_inds.end(),
  [&] (const uint32_t start_ind) -> void
  {
    std::for_each(std::execution::par_unseq, vertex_inds.begin(), vertex_inds.end(),
    [&] (const uint32_t target_ind) -> void
    {
      std::sort(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).begin(), this->m_ptp_paths(start_ind, target_ind).end(),
        [] (const Path& i, const Path& j) -> bool
        {
          if(i.traversal_time == j.traversal_time)
          {
            if(i.traversal_charge == j.traversal_charge)
            {
              if(i.traversal_charge_needed == j.traversal_charge_needed)
              {
                if(i.end_traversal_charge == j.end_traversal_charge)
                {
                  return i.length < j.length;
                }
                else
                {
                  return i.end_traversal_charge < j.end_traversal_charge;
                }
              }
              else
              {
                return i.traversal_charge_needed < j.traversal_charge_needed;
              }
            }
            else
            {
              return i.traversal_charge < j.traversal_charge;
            }
          }
          else
          {
            return i.traversal_time < j.traversal_time;
          }
        });
      this->m_ptp_paths(start_ind, target_ind).erase(std::unique(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).begin(), this->m_ptp_paths(start_ind, target_ind).end(),
        [using_soc_constraints,using_distance_constraints] (const Path& i, const Path& j) -> bool
        {
          return (i.traversal_time == j.traversal_time) and
                 (
                  (not using_soc_constraints) or
                  (
                   (i.traversal_charge        == j.traversal_charge)        and
                   (i.traversal_charge_needed == j.traversal_charge_needed) and
                   (i.end_traversal_charge    == j.end_traversal_charge)
                  )
                 ) and
                 ((not using_distance_constraints) or (i.length == j.length));
        }),
        this->m_ptp_paths(start_ind, target_ind).end());
      this->m_ptp_paths(start_ind, target_ind).erase(std::remove_if(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).begin(), this->m_ptp_paths(start_ind, target_ind).end(),
        [this,start_ind,target_ind,using_soc_constraints,using_distance_constraints] (const Path& remove) -> bool
        {
          return std::any_of(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).cbegin(),
                             this->m_ptp_paths(start_ind, target_ind).cend(),
                             [&remove,using_soc_constraints,using_distance_constraints] (const Path& comp) -> bool
                             {
                               return (&remove               != &comp)               and
                                      (remove.traversal_time >= comp.traversal_time) and
                                      (
                                       (not using_soc_constraints) or
                                       (
                                        (remove.traversal_charge        >= comp.traversal_charge)        and
                                        (remove.traversal_charge_needed >= comp.traversal_charge_needed) and
                                        (remove.end_traversal_charge    >= comp.end_traversal_charge)
                                       )
                                      ) and
                                      ((not using_distance_constraints) or (remove.length >= comp.length));
                             });
        }),
        this->m_ptp_paths(start_ind, target_ind).end());
    });
  });
  /// Find all point to point paths
  std::for_each(std::execution::par_unseq, vertex_inds.begin(), vertex_inds.end(),
  [&] (const uint32_t start_ind) -> void
  {
    const uint32_t start_orig_ind = to_orig_inds_map[start_ind];

    std::for_each(std::execution::par_unseq, vertex_inds.begin(), vertex_inds.end(),
    [&] (const uint32_t target_ind) -> void
    {
      if(start_ind == target_ind) { return; }

      const uint32_t target_orig_ind = to_orig_inds_map[target_ind];

      // Next steps in paths being considered, cur edge inds list - cur traversal time - cur traversal charge - cur charge necessary - cur end charge usage - cur min charge - cur distance
      std::list<std::tuple<std::vector<uint32_t>,float,float,float,float,float,float>> next_step;
      // Add start node
      std::for_each(simple_exit_edge_inds[start_orig_ind].cbegin(),
                    simple_exit_edge_inds[start_orig_ind].cend(),
      [&] (const uint32_t start_edge_ind) -> void
      {
        const float traversal_time   = simple_edges_time[  start_edge_ind];
        const float traversal_charge = simple_edges_charge[start_edge_ind];
        const float charge_needed    = std::max<float>(0, traversal_charge);
        const float end_charge_usage = charge_needed;
        const float min_charge       = std::min<float>(0, traversal_charge);
        const float distance         = simple_edges_length[start_edge_ind];

        const bool soc_constraints_good  = (not using_soc_constraints)      or ((charge_needed <= agent_max_soc) and (end_charge_usage <= agent_max_soc));
        const bool dist_constraints_good = (not using_distance_constraints) or (distance <= max_trip_length);
        if((traversal_time <= max_time) and soc_constraints_good and dist_constraints_good)
        {
          next_step.emplace_back(std::vector<uint32_t>({start_edge_ind}), traversal_time, traversal_charge, charge_needed, end_charge_usage, min_charge, distance);
        }
      });
      // Recurs through graph
      while(not next_step.empty())
      {
        // Get next step
        const std::tuple<std::vector<uint32_t>,float,float,float,float,float,float>& next_ref = next_step.back();
        const std::vector<uint32_t> past_edge_inds = std::move(std::get<0>(next_ref));
        const float                 time           = std::get<1>(next_ref);
        const float                 charge         = std::get<2>(next_ref);
        const float                 charge_needed  = std::get<3>(next_ref);
        const float                 end_charge     = std::get<4>(next_ref);
        const float                 min_charge     = std::get<5>(next_ref);
        const float                 length         = std::get<6>(next_ref);
        next_step.pop_back();

        const uint32_t cur_orig_ind = simple_edges_to_node_inds[past_edge_inds.back()];

        // If to a target
        if(cur_orig_ind == target_orig_ind)
        {
          // Check for usefulness
          if(std::all_of(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).cbegin(), this->m_ptp_paths(start_ind, target_ind).cend(),
             [&] (const Path& past_path) -> bool
             {
               return (time < past_path.traversal_time) or
                      (
                       using_soc_constraints and
                       (
                        (charge        < past_path.traversal_charge)        or
                        (charge_needed < past_path.traversal_charge_needed) or
                        (end_charge    < past_path.end_traversal_charge)
                       )
                      ) or
                      (using_distance_constraints and (length < past_path.length));
             }))
          {
            std::vector<const Edge*> path_edges;
            path_edges.reserve(street_graph.numberOriginalEdges());
            std::for_each(past_edge_inds.cbegin(), past_edge_inds.cend(),
            [&] (const uint32_t simple_edge_ind) -> void
            {
              path_edges.insert(path_edges.end(), simple_edges[simple_edge_ind].cbegin(), simple_edges[simple_edge_ind].cend());
            });
            path_edges.shrink_to_fit();
            // Add path to candidates
            paths_mux.lock();
            this->m_ptp_paths(start_ind, target_ind).emplace_back(start_ind,
                                                                  target_ind,
                                                                  std::move(path_edges),
                                                                  time,
                                                                  charge,
                                                                  charge_needed,
                                                                  end_charge,
                                                                  length);
            // Remove any sub-optimal paths
            this->m_ptp_paths(start_ind, target_ind).erase(std::remove_if(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).begin(), this->m_ptp_paths(start_ind, target_ind).end(),
                                                           [this,start_ind,target_ind,using_soc_constraints,using_distance_constraints] (const Path& remove) -> bool
                                                           {
                                                             return std::any_of(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).cbegin(),
                                                                                this->m_ptp_paths(start_ind, target_ind).cend(),
                                                                                [&remove,using_soc_constraints,using_distance_constraints] (const Path& comp) -> bool
                                                                                {
                                                                                  return (&remove               != &comp)               and
                                                                                         (remove.traversal_time >= comp.traversal_time) and
                                                                                         (
                                                                                          (not using_soc_constraints) or
                                                                                          (
                                                                                           (remove.traversal_charge        >= comp.traversal_charge)        and
                                                                                           (remove.traversal_charge_needed >= comp.traversal_charge_needed) and
                                                                                           (remove.end_traversal_charge    >= comp.end_traversal_charge)
                                                                                          )
                                                                                         ) and
                                                                                         ((not using_distance_constraints) or (remove.length >= comp.length));
                                                                                });
                                                           }),
                                                           this->m_ptp_paths(start_ind, target_ind).end());
            paths_mux.unlock();
          }
        }
        else // Not to target
        {
          // Recurs
          std::for_each(simple_exit_edge_inds[cur_orig_ind].cbegin(), simple_exit_edge_inds[cur_orig_ind].cend(),
          [&] (const uint32_t next_edge_ind) -> void
          {
            const uint32_t next_edge_to_ind_orig = simple_edges_to_node_inds[next_edge_ind];
            // Check for a loop
            if((start_orig_ind == next_edge_to_ind_orig) or
               std::any_of(std::execution::unseq, past_edge_inds.cbegin(), past_edge_inds.cend(),
               [&simple_edges_to_node_inds,next_edge_to_ind_orig] (const uint32_t past_edge_ind) -> bool
               {
                 return next_edge_to_ind_orig == simple_edges_to_node_inds[past_edge_ind];
               }))
            { return; }

            // Calculate new edge info
            const float next_time          = time   + simple_edges_time[  next_edge_ind];
            const float next_charge        = charge + simple_edges_charge[next_edge_ind];
            const float next_length        = length + simple_edges_length[next_edge_ind];
            const float next_charge_needed = std::max<float>(charge_needed, next_charge);
            float next_end_charge;
            float next_min_charge;
            if(next_charge < min_charge)
            {
              next_end_charge = 0;
              next_min_charge = next_charge;
            }
              else // Not new min
              {
              next_end_charge = end_charge + simple_edges_charge[next_edge_ind];
              next_min_charge = min_charge;
            }
            // Check validity conditions
            const bool soc_constraints_good  = (not using_soc_constraints)      or (
                                                                                    (next_charge_needed <= agent_max_soc) and
                                                                                    (next_end_charge <= agent_max_soc)    and
                                                                                    (street_graph.minChargeNecessary(cur_orig_ind, 0) <= (agent_max_soc - next_charge))
                                                                                   );
            const bool dist_constraints_good = (not using_distance_constraints) or (
                                                                                    (next_length <= max_trip_length) and
                                                                                    (street_graph.minLength(cur_orig_ind, 0) <= (max_trip_length - next_length))
                                                                                   );
            if((not (next_time <= max_time))                                          or
               ((next_time + street_graph.minTravelTime(cur_orig_ind, 0)) > max_time) or
               (not soc_constraints_good)                                             or
               (not dist_constraints_good))
            { return; }
            // Check for usefulness TODO: this is heuristic and technically not valid
            {
              const float possible_time          = next_time          + street_graph.minTravelTime(     start_orig_ind, target_orig_ind);
              const float possible_charge        = next_charge        + street_graph.minCharge(         start_orig_ind, target_orig_ind);
              const float possible_length        = next_length        + street_graph.minLength(         start_orig_ind, target_orig_ind);
              const float possible_charge_needed = next_charge_needed + street_graph.minChargeNecessary(start_orig_ind, target_orig_ind);
              if(std::any_of(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).cbegin(), this->m_ptp_paths(start_ind, target_ind).cend(),
                 [possible_time,possible_charge,possible_charge_needed,possible_length,using_soc_constraints,using_distance_constraints] (const Path& comp) -> bool
                 {
                   return (possible_time >= comp.traversal_time) and
                          (
                           (not using_soc_constraints) or
                           (
                            (possible_charge        >= comp.traversal_charge) and
                            (possible_charge_needed >= comp.traversal_charge_needed)
                           )
                          ) and
                          ((not using_distance_constraints) or (possible_length >= comp.length));
                 }))
              { return; }
            }
            // Add to next step list
            next_step.emplace_back(std::vector<uint32_t>(), next_time, next_charge, next_charge_needed, next_end_charge, next_min_charge, next_length);
            std::get<0>(next_step.back()).reserve(past_edge_inds.size() + 1);
            std::get<0>(next_step.back()).insert(std::get<0>(next_step.back()).end(), past_edge_inds.cbegin(), past_edge_inds.cend());
            std::get<0>(next_step.back()).emplace_back(next_edge_ind);
          });
        }
      }
    });
  });
  // Remove any sub-optimal paths
  std::for_each(vertex_inds.begin(), vertex_inds.end(),
  [&] (const uint32_t start_ind) -> void
  {
    std::for_each(vertex_inds.begin(), vertex_inds.end(),
    [&] (const uint32_t target_ind) -> void
    {
      std::sort(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).begin(), this->m_ptp_paths(start_ind, target_ind).end(),
        [] (const Path& i, const Path& j) -> bool
        {
          if(i.traversal_time == j.traversal_time)
          {
            if(i.traversal_charge == j.traversal_charge)
            {
              if(i.traversal_charge_needed == j.traversal_charge_needed)
              {
                if(i.end_traversal_charge == j.end_traversal_charge)
                {
                  return i.length < j.length;
                }
                else
                {
                  return i.end_traversal_charge < j.end_traversal_charge;
                }
              }
              else
              {
                return i.traversal_charge_needed < j.traversal_charge_needed;
              }
            }
            else
            {
              return i.traversal_charge < j.traversal_charge;
            }
          }
          else
          {
            return i.traversal_time < j.traversal_time;
          }
        });
      this->m_ptp_paths(start_ind, target_ind).erase(std::unique(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).begin(), this->m_ptp_paths(start_ind, target_ind).end(),
        [using_soc_constraints,using_distance_constraints] (const Path& i, const Path& j) -> bool
        {
          return (i.traversal_time == j.traversal_time) and
                 (
                  (not using_soc_constraints) or
                  (
                   (i.traversal_charge        == j.traversal_charge)        and
                   (i.traversal_charge_needed == j.traversal_charge_needed) and
                   (i.end_traversal_charge    == j.end_traversal_charge)
                  )
                 ) and
                 ((not using_distance_constraints) or (i.length == j.length));
        }),
        this->m_ptp_paths(start_ind, target_ind).end());
      this->m_ptp_paths(start_ind, target_ind).erase(std::remove_if(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).begin(), this->m_ptp_paths(start_ind, target_ind).end(),
        [this,start_ind,target_ind,using_soc_constraints,using_distance_constraints] (const Path& remove) -> bool
        {
          return std::any_of(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).cbegin(),
                             this->m_ptp_paths(start_ind, target_ind).cend(),
                             [&remove,using_soc_constraints,using_distance_constraints] (const Path& comp) -> bool
                             {
                               return (&remove               != &comp)               and
                                      (remove.traversal_time >= comp.traversal_time) and
                                      (
                                       (not using_soc_constraints) or
                                       (
                                        (remove.traversal_charge        >= comp.traversal_charge)        and
                                        (remove.traversal_charge_needed >= comp.traversal_charge_needed) and
                                        (remove.end_traversal_charge    >= comp.end_traversal_charge)
                                       )
                                      ) and
                                      ((not using_distance_constraints) or (remove.length >= comp.length));
                             });
        }),
        this->m_ptp_paths(start_ind, target_ind).end());
    });
  });
  std::for_each(vertex_inds.begin(), vertex_inds.end(),
  [&] (const uint32_t start_ind) -> void
  {
    std::for_each(vertex_inds.begin(), vertex_inds.end(),
    [&] (const uint32_t target_ind) -> void
    {
      this->m_ptp_paths(start_ind, target_ind).shrink_to_fit();
    });
  });
  this->m_number_paths = this->m_ptp_paths.unaryExpr([] (const std::vector<Path>& paths) -> uint32_t { return paths.size(); }).count();
  /// Find min paths
  std::for_each(std::execution::par_unseq, vertex_inds.begin(), vertex_inds.end(),
  [&] (const uint32_t start_ind) -> void
  {
    std::for_each(std::execution::par_unseq, vertex_inds.begin(), vertex_inds.end(),
    [&] (const uint32_t target_ind) -> void
    {
      if(start_ind != target_ind)
      {
        auto find_it = std::min_element(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).cbegin(),
                                        this->m_ptp_paths(start_ind, target_ind).cend(),
                                        [] (const Path& i, const Path& j) -> bool { return i.traversal_time < j.traversal_time; });
        assert(this->m_ptp_paths(start_ind, target_ind).cend() != find_it);
        this->m_min_time_ptp_paths(start_ind, target_ind) = *find_it;
        assert(std::fabs(this->m_min_time_ptp_paths(start_ind, target_ind).traversal_time - street_graph.minTravelTime(to_orig_inds_map[start_ind], to_orig_inds_map[target_ind])) < double(1e-4));

        find_it = std::min_element(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).cbegin(),
                                   this->m_ptp_paths(start_ind, target_ind).cend(),
                                   [] (const Path& i, const Path& j) -> bool { return i.traversal_charge < j.traversal_charge; });
        assert(this->m_ptp_paths(start_ind, target_ind).cend() != find_it);
        this->m_min_charge_ptp_paths(start_ind, target_ind) = *find_it;
        assert(std::fabs(this->m_min_charge_ptp_paths(start_ind, target_ind).traversal_charge - street_graph.minCharge(to_orig_inds_map[start_ind], to_orig_inds_map[target_ind])) < double(1e-8));

        find_it = std::min_element(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).cbegin(),
                                   this->m_ptp_paths(start_ind, target_ind).cend(),
                                   [] (const Path& i, const Path& j) -> bool { return i.traversal_charge_needed < j.traversal_charge_needed; });
        assert(this->m_ptp_paths(start_ind, target_ind).cend() != find_it);
        this->m_min_charge_needed_ptp_paths(start_ind, target_ind) = *find_it;

        find_it = std::min_element(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).cbegin(),
                                   this->m_ptp_paths(start_ind, target_ind).cend(),
                                   [] (const Path& i, const Path& j) -> bool { return i.end_traversal_charge < j.end_traversal_charge; });
        assert(this->m_ptp_paths(start_ind, target_ind).cend() != find_it);
        this->m_min_end_charge_ptp_paths(start_ind, target_ind) = *find_it;

        find_it = std::min_element(std::execution::unseq, this->m_ptp_paths(start_ind, target_ind).cbegin(),
                                   this->m_ptp_paths(start_ind, target_ind).cend(),
                                   [] (const Path& i, const Path& j) -> bool { return i.length < j.length; });
        assert(this->m_ptp_paths(start_ind, target_ind).cend() != find_it);
        this->m_min_length_ptp_paths(start_ind, target_ind) = *find_it;
      }
    });
  });
}

/* planning_graph.cpp */
