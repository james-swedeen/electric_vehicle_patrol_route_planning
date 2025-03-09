/**
 * @File: hotspot.hpp
 * @Date: May 2024
 * @Author: James Swedeen
 *
 * @brief
 * Defines a place that needs to be patrolled.
 **/

#ifndef SURVEILLANCE_PLANNING_HOTSPOT_HPP
#define SURVEILLANCE_PLANNING_HOTSPOT_HPP

/* C++ Headers */
#include<cmath>
#include<memory>
#include<vector>
#include<list>
#include<algorithm>
#include<execution>
#include<mutex>

/* Street Graph Headers */
#include<street_graph/planning_graph.hpp>

/* Local Header */
#include<surveillance_planner/action.hpp>

namespace plan
{
/**
 * @Hotspot
 *
 * @brief
 * Defines a place that needs to be patrolled.
 **/
class Hotspot
{
public:
  /**
   * @Default Constructor
   **/
  inline Hotspot() noexcept = default;
  /**
   * @Copy Constructor
   **/
  inline Hotspot(const Hotspot&) noexcept = default;
  /**
   * @Move Constructor
   **/
  inline Hotspot(Hotspot&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor initializes the class for use.
   *
   * @parameters
   * vertex_ind: The index of this hotspot in the planning_graph
   **/
  inline Hotspot(const uint32_t vertex_ind,
                 const float    initial_exp_event_rate,
                 const float    min_exp_event_rate           = 0.041, // Number from Koper, Christopher S. "Just enough police presence: Reducing crime and disorderly behavior by optimizing patrol time in crime hot spots." Justice quarterly 12.4 (1995): 649-672.
                 const float    max_exp_event_rate           = 0.161,
                 const float    exp_event_rate_obs_slope     = float (0.041-0.161)/float (15*60),
                 const float    exp_event_rate_not_obs_slope = float (0.161-0.041)/float (3*60*60)) noexcept;
  /**
   * @Deconstructor
   **/
  inline ~Hotspot() noexcept = default;
  /**
   * @Assignment Operators
   **/
  inline Hotspot& operator=(const Hotspot&)  noexcept = default;
  inline Hotspot& operator=(      Hotspot&&) noexcept = default;
  /**
   * @Get Functions
   **/
  inline uint32_t cgetGraphIndex()                        const noexcept;
  inline float    cgetMinExpectedEventRate()              const noexcept;
  inline float    cgetMaxExpectedEventRate()              const noexcept;
  inline float    cgetExpectedEventRate()                 const noexcept;
  inline float    cgetExpectedEventRateSlopeObserved()    const noexcept;
  inline float    cgetExpectedEventRateSlopeNotObserved() const noexcept;
  /**
   * @propagateAgentPresent
   *
   * @brief
   * Used to integrate the expected event rate forward in time given that an agent is present.
   *
   * @parameters
   * prop_time: The simulation time to propagate for
   **/
  inline void propagateAgentPresent(const float prop_time) noexcept;
  /**
   * @propagateNoAgentPresent
   *
   * @brief
   * Used to integrate the expected event rate forward in time given that no agent is present.
   *
   * @parameters
   * prop_time: The simulation time to propagate for
   **/
  inline void propagateNoAgentPresent(const float prop_time) noexcept;
  /**
   * @minDwellTime
   *
   * @brief
   * Finds the amount of time needed to be spent in this hotspot to get the ECR down to the minimum.
   *
   * @parameters
   * ecr: The ecr to calculate the min dwell time with
   * target_ecr: The ecr to get to while calculating the min time
   *
   * @return
   * The min dwell time.
   **/
  inline float minDwellTime()                                          const noexcept;
  inline float minDwellTime(const float ecr)                          const noexcept;
  inline float minDwellTime(const float ecr, const float target_ecr) const noexcept;
private:
  // Changing variables
  float    m_exp_event_rate; // heuristic estimate of the probability of a dispatch call-in occurring from a given location within the next half an hour
  // Class defining variables
  float    m_min_exp_event_rate;           // The min possible expected event rate
  float    m_max_exp_event_rate;           // The max possible expected event rate
  float    m_exp_event_rate_obs_slope;     // Increase per a second of the expected event rate when an agent is not present
  float    m_exp_event_rate_not_obs_slope; // Decrease per a second of the expected event rate when an agent is present
  uint32_t m_vertex_ind;                   // The index of this hotspot in the planning_graph
};


inline Hotspot::Hotspot(const uint32_t vertex_ind,
                        const float    initial_exp_event_rate,
                        const float    min_exp_event_rate,
                        const float    max_exp_event_rate,
                        const float    exp_event_rate_obs_slope,
                        const float    exp_event_rate_not_obs_slope) noexcept
 : m_exp_event_rate(initial_exp_event_rate),
   m_min_exp_event_rate(min_exp_event_rate),
   m_max_exp_event_rate(max_exp_event_rate),
   m_exp_event_rate_obs_slope(exp_event_rate_obs_slope),
   m_exp_event_rate_not_obs_slope(exp_event_rate_not_obs_slope),
   m_vertex_ind(vertex_ind)
{
  assert(this->m_exp_event_rate_obs_slope < 0);
  assert(this->m_exp_event_rate_not_obs_slope > 0);
  assert(this->m_exp_event_rate >= this->m_min_exp_event_rate);
  assert(this->m_exp_event_rate <= this->m_max_exp_event_rate);
  assert(this->m_min_exp_event_rate <= this->m_max_exp_event_rate);
}

inline uint32_t Hotspot::cgetGraphIndex() const noexcept
{
  return this->m_vertex_ind;
}

inline float Hotspot::cgetMinExpectedEventRate() const noexcept
{
  return this->m_min_exp_event_rate;
}

inline float Hotspot::cgetMaxExpectedEventRate() const noexcept
{
  return this->m_max_exp_event_rate;
}

inline float Hotspot::cgetExpectedEventRate() const noexcept
{
  return this->m_exp_event_rate;
}

inline float Hotspot::cgetExpectedEventRateSlopeObserved() const noexcept
{
  return this->m_exp_event_rate_obs_slope;
}

inline float Hotspot::cgetExpectedEventRateSlopeNotObserved() const noexcept
{
  return this->m_exp_event_rate_not_obs_slope;
}

inline void Hotspot::propagateAgentPresent(const float prop_time) noexcept
{
  assert(prop_time >= -1e-4);

  this->m_exp_event_rate = std::max<float>(this->cgetExpectedEventRate() + (this->cgetExpectedEventRateSlopeObserved() * prop_time), this->cgetMinExpectedEventRate());
}

inline void Hotspot::propagateNoAgentPresent(const float prop_time) noexcept
{
  assert(prop_time >= -1e-4);

  this->m_exp_event_rate = std::min<float>(this->cgetExpectedEventRate() + (this->cgetExpectedEventRateSlopeNotObserved() * prop_time), this->cgetMaxExpectedEventRate());
}

inline float Hotspot::minDwellTime() const noexcept
{
  return this->minDwellTime(this->cgetExpectedEventRate());
}

inline float Hotspot::minDwellTime(const float ecr) const noexcept
{
  return this->minDwellTime(ecr, this->cgetMinExpectedEventRate());
}

inline float Hotspot::minDwellTime(const float ecr, const float target_ecr) const noexcept
{
  const float diff = ecr - target_ecr;

  if(0 >= diff)
  {
    return 0;
  }
  else
  {
    return -diff / this->cgetExpectedEventRateSlopeObserved();
  }
}
} // plan

#endif
/* hotspot.hpp */
