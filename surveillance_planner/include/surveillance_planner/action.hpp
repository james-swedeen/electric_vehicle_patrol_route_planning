/**
 * @File: action.hpp
 * @Date: March 2024
 * @Author: James Swedeen
 *
 * @brief
 * Defines the Action class that is used to define t planner solutions.
 **/

#ifndef SURVEILLANCE_PLANNING_ACTION_HPP
#define SURVEILLANCE_PLANNING_ACTION_HPP

/* C++ Headers */
#include<cstddef>
#include<utility>
#include<forward_list>
#include<mutex>
#include<ostream>

/* Eigen Headers */
#include<Eigen/Dense>

/* Street Graph Headers */
#include<street_graph/planning_graph.hpp>

namespace plan
{
/**
 * @Action
 *
 * @brief
 * Used to denote the planning graph vertex an agent is at, the path used to get there, and the dwell time at that
 * vertex.
 **/
struct Action
{
public:
  const graph::Path* prev_path;  // The path used to get to this vertex or a null pointer if this is the first action in a plan
  uint32_t           vertex_ind; // The PlanningGraph vertex index that this action is related to
  uint32_t           dwell_time; // The time in minutes to dwell at this vertex, if this vertex is the depot then charge

  inline Action()                         noexcept = default;
  inline Action(const Action&)            noexcept = default;
  inline Action(Action&&)                 noexcept = default;
  inline Action& operator=(const Action&) noexcept = default;
  inline Action& operator=(Action&&)      noexcept = default;
  inline ~Action()                        noexcept = default;

  /**
   * @Constructor
   *
   * @brief
   * This constructor initializes the class for use.
   *
   * @parameters
   * vertex_ind: The index of the vertex this action ends at
   * prev_path: The path the goes from the previous action to this action
   * dwell_time: The length of time to dwell at this vertex
   **/
  inline Action(const uint32_t vertex_ind, const graph::Path* const prev_path, const uint32_t dwell_time) noexcept;
  /**
   * @Equal Operator
   **/
  inline bool operator==(const Action& rhs) const;
  /**
   * @To Stream Operator
   **/
  friend inline std::ostream& operator<<(std::ostream& os, const Action& action) noexcept
  {
    os << "vert ind: " << action.vertex_ind << "\tprev path ptr: " << action.prev_path << "\tdwell time: " << action.dwell_time;
    return os;
  }
};
} // plan


inline plan::Action::Action(const uint32_t vertex_ind, const graph::Path* const prev_path, const uint32_t dwell_time) noexcept
 : prev_path(prev_path),
   vertex_ind(vertex_ind),
   dwell_time(dwell_time)
{}

inline bool plan::Action::operator==(const Action& rhs) const
{
  return (this->prev_path  == rhs.prev_path)  and
         (this->vertex_ind == rhs.vertex_ind) and
         (this->dwell_time == rhs.dwell_time);

}

#endif
/* action.hpp */
