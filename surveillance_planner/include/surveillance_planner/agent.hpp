/**
 * @File: agent.hpp
 * @Date: March 2024
 * @Author: James Swedeen
 *
 * @brief
 * Represents the surveillance vehicles that are planned for.
 **/

#ifndef SURVEILLANCE_PLANNING_AGENT_HPP
#define SURVEILLANCE_PLANNING_AGENT_HPP

/* C++ Headers */
#include<limits>
#include<list>

/* Street Graph Headers */
#include<street_graph/planning_graph.hpp>

namespace plan
{
class Agent
{
public:
  /**
   * @Default Constructor
   **/
  Agent() noexcept = default;
  /**
   * @Copy Constructor
   **/
  inline Agent(const Agent&) noexcept = default;
  /**
   * @Move Constructor
   **/
  inline Agent(Agent&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * max_state_of_charge: The max SOC
   * max_charge_rate: Max rate of charging
   * max_trip_length: Max distance the can be covered without charging (when distance-based constraints are being used)
   * max_charge_time: Time needed to be spent on the charger to get to max charge from zero SOC
   * cur_state_of_charge: Current SOC
   * cur_state_of_charge: Current trip length
   * shift_start_time: After this point the agent can leave the depot
   * shift_end_time: After this point the agent must spend all time at the depot
   * auxiliary_power_kws: Power draw that happens regardless of the agent moving
   **/
  inline Agent(const float max_state_of_charge,
               const float max_charge_rate,
               const float max_trip_length,
               const float cur_state_of_charge,
               const float cur_trip_length,
               const float shift_start_time,
               const float shift_end_time,
               const float auxiliary_power_kws = float(3000) / float(60) / float(60) / float(1000)) noexcept;
  inline Agent(const float max_state_of_charge,
               const float max_charge_rate,
               const float max_trip_length,
               const float max_charge_time,
               const float cur_state_of_charge,
               const float cur_trip_length,
               const float shift_start_time,
               const float shift_end_time,
               const float auxiliary_power_kws = float(3000) / float(60) / float(60) / float(1000)) noexcept;
  /**
   * @Deconstructor
   **/
  inline ~Agent() noexcept = default;
  /**
   * @Assignment Operators
   **/
  inline Agent& operator=(const Agent&)  noexcept = default;
  inline Agent& operator=(      Agent&&) noexcept = default;
  /**
   * @Get Functions
   **/
  inline float cgetMaxStateOfCharge()     const noexcept;
  inline float cgetMaxChargeRate()        const noexcept;
  inline float cgetMaxTripLength()        const noexcept;
  inline float cgetCurrentStateOfCharge() const noexcept;
  inline float cgetCurrentTripLength()    const noexcept;
  inline float cgetShiftStartTime()       const noexcept;
  inline float cgetShiftEndTime()         const noexcept;
  inline float cgetAuxiliaryPower()       const noexcept;
  /**
   * @shiftDuration
   *
   * @brief
   * The length of time in seconds that this agent's shift lasts.
   **/
  inline float shiftDuration() const noexcept;
  /**
   * @tripLengthLeft
   *
   * @brief
   * The trip length left for use after the current trip length has been used.
   *
   * @parameters
   * cur_trip_length: The trip length to calculate it with
   * max_trip_length: The max trip length to use
   *
   * @return
   * The trip length left.
   **/
  inline float tripLengthLeft()                                                           const noexcept;
  inline float tripLengthLeft(const float cur_trip_length)                               const noexcept;
  inline float tripLengthLeft(const float cur_trip_length, const float max_trip_length) const noexcept;
  /**
   * @minChargeTime
   *
   * @brief
   * Finds the amount of time needed to be spent on the charger to get to max charge given the current charge level.
   *
   * @parameters
   * state_of_charge: The state of charge to calculate the min charge time with
   * target_state_of_charge: The state of charge to get to while calculating the min charge time
   *
   * @return
   * The min charge time.
   **/
  inline float minChargeTime()                                                                  const noexcept;
  inline float minChargeTime(const float state_of_charge)                                      const noexcept;
  inline float minChargeTime(const float state_of_charge, const float target_state_of_charge) const noexcept;
  /**
   * @maxChargeTime
   *
   * @brief
   * Finds the amount of time needed to be spent on the charger to get to max charge from zero SOC.
   *
   * @return
   * The min charge time.
   **/
  inline float maxChargeTime() const noexcept;
  /**
   * @chargeFor
   *
   * @brief
   * Charges the agent for a given amount of time.
   *
   * @parameters
   * time: The amount of time to charge
   *
   * @return
   * The agent with a new charge level.
   **/
  inline Agent chargeFor(       const float time) const noexcept;
  inline void  chargeForInPlace(const float time)       noexcept;
  /**
   * @resetTripCounter
   *
   * @brief
   * Resets the current trip distance covered to zero and sets current SOC to max.
   *
   * @return
   * The agent with a reset trip counter.
   **/
  inline Agent resetTripCounter()        const noexcept;
  inline void  resetTripCounterInPlace()       noexcept;
  /**
   * @propagateAlongTime
   *
   * @brief
   * Propagates this agent forward in time assuming the agent is not charging or traversing a path.
   *
   * @parameters
   * time_step: How long to propagate for
   *
   * @return
   * The propagated agent.
   **/
  inline Agent propagateAlongTime(       const float time_step) const noexcept;
  inline void  propagateAlongTimeInPlace(const float time_step)       noexcept;
  /**
   * @propagateAlongPath
   *
   * @brief
   * Propagates this agent along a given graph path.
   * Assumes that the agent is getting propagated from the start of the edge to the end.
   *
   * @parameters
   * path: The path to propagate the agent along
   *
   * @return
   * The propagated agent.
   **/
  inline Agent propagateAlongPath(       const graph::Path& path) const noexcept;
  inline void  propagateAlongPathInPlace(const graph::Path& path)       noexcept;
  /**
   * @propagateAlongPath
   *
   * @brief
   * Propagates this agent along a given path.
   *
   * @parameters
   * path: The path to propagate the agent along
   * path_start_time: The time after entering this path to start propagating
   * path_end_time: The time after entering this path to stop propagating
   *
   * @return
   * The propagated agent.
   **/
  inline Agent propagateAlongPath(       const graph::Path& path, const float path_start_time, const float path_end_time) const noexcept;
  inline void  propagateAlongPathInPlace(const graph::Path& path, const float path_start_time, const float path_end_time)       noexcept;
private:
  // Agent defining variables
  float m_max_state_of_charge; // Max charge possible in kWh
  float m_max_charge_rate;     // The fastest this agent can charge in kWh/sec
  float m_max_trip_length;     // Max distance the can be covered without charging (when distance-based constraints are being used)
  float m_max_charge_time;     // Time needed to be spent on the charger to get to max charge from zero SOC
  float m_shift_start_time;    // First time that the agent is ready for use
  float m_shift_end_time;      // Last time that the agent is ready for use
  float m_auxiliary_power_kws; // The wattage per a second used at all times

  // Current defining variables
  float m_state_of_charge; // The current charge level of the agent in kWh
  float m_cur_trip_length; // How far has been traversed since the last charge
};


inline Agent::Agent(const float max_state_of_charge,
                    const float max_charge_rate,
                    const float max_trip_length,
                    const float cur_state_of_charge,
                    const float cur_trip_length,
                    const float shift_start_time,
                    const float shift_end_time,
                    const float auxiliary_power_kws) noexcept
 : m_max_state_of_charge(max_state_of_charge),
   m_max_charge_rate(max_charge_rate),
   m_max_trip_length(max_trip_length),
   //m_max_charge_time(60 * size_t(std::ceil((double(1) / double(60)) * (max_state_of_charge / max_charge_rate)))),
   m_max_charge_time(max_state_of_charge / max_charge_rate),
   m_shift_start_time(shift_start_time),
   m_shift_end_time(shift_end_time),
   m_auxiliary_power_kws(auxiliary_power_kws),
   m_state_of_charge(cur_state_of_charge),
   m_cur_trip_length(cur_trip_length)
{
  assert(this->m_max_state_of_charge >= 0);
  assert(this->m_max_charge_rate     >= 0);
  assert(this->m_max_trip_length     >= 0);
  assert(this->m_state_of_charge     >= 0);
  assert(this->m_state_of_charge     <= this->m_max_state_of_charge);
  assert(this->m_shift_end_time      >= this->m_shift_start_time);
}

inline Agent::Agent(const float max_state_of_charge,
                    const float max_charge_rate,
                    const float max_trip_length,
                    const float max_charge_time,
                    const float cur_state_of_charge,
                    const float cur_trip_length,
                    const float shift_start_time,
                    const float shift_end_time,
                    const float auxiliary_power_kws) noexcept
 : m_max_state_of_charge(max_state_of_charge),
   m_max_charge_rate(max_charge_rate),
   m_max_trip_length(max_trip_length),
   m_max_charge_time(max_charge_time),
   m_shift_start_time(shift_start_time),
   m_shift_end_time(shift_end_time),
   m_auxiliary_power_kws(auxiliary_power_kws),
   m_state_of_charge(cur_state_of_charge),
   m_cur_trip_length(cur_trip_length)
{
  assert(this->m_max_state_of_charge >= 0);
  assert(this->m_max_charge_rate     >= 0);
  assert(this->m_max_trip_length     >= 0);
  assert(this->m_max_charge_time     >= 0);
  assert(std::fabs(this->m_max_charge_time - (this->m_max_state_of_charge / this->m_max_charge_rate)) < 1.0e-8);
  assert(this->m_state_of_charge     >= 0);
  assert(this->m_state_of_charge     <= this->m_max_state_of_charge);
  assert(this->m_shift_end_time      >= this->m_shift_start_time);
}

inline float Agent::cgetMaxStateOfCharge() const noexcept
{
  return this->m_max_state_of_charge;
}

inline float Agent::cgetMaxChargeRate() const noexcept
{
  return this->m_max_charge_rate;
}

inline float Agent::cgetMaxTripLength() const noexcept
{
  return this->m_max_trip_length;
}

inline float Agent::cgetCurrentStateOfCharge() const noexcept
{
  return this->m_state_of_charge;
}

inline float Agent::cgetCurrentTripLength() const noexcept
{
  return this->m_cur_trip_length;
}

inline float Agent::cgetShiftStartTime() const noexcept
{
  return this->m_shift_start_time;
}

inline float Agent::cgetShiftEndTime() const noexcept
{
  return this->m_shift_end_time;
}

inline float Agent::cgetAuxiliaryPower() const noexcept
{
  return this->m_auxiliary_power_kws;
}

inline float Agent::shiftDuration() const noexcept
{
  return this->cgetShiftEndTime() - this->cgetShiftStartTime();
}

inline float Agent::tripLengthLeft() const noexcept
{
  return this->tripLengthLeft(this->cgetCurrentTripLength());
}

inline float Agent::tripLengthLeft(const float cur_trip_length) const noexcept
{
  return this->tripLengthLeft(cur_trip_length, this->cgetMaxTripLength());
}

inline float Agent::tripLengthLeft(const float cur_trip_length, const float max_trip_length) const noexcept
{
  assert(max_trip_length >= cur_trip_length);
  return max_trip_length - cur_trip_length;
}

inline float Agent::minChargeTime() const noexcept
{
  return this->minChargeTime(this->cgetCurrentStateOfCharge());
}

inline float Agent::minChargeTime(const float state_of_charge) const noexcept
{
  return this->minChargeTime(state_of_charge, this->cgetMaxStateOfCharge());
}

inline float Agent::minChargeTime(const float state_of_charge, const float target_state_of_charge) const noexcept
{
  assert(target_state_of_charge >= 0);
  assert(state_of_charge        >= 0);
  if(state_of_charge < target_state_of_charge)
  {
    return (target_state_of_charge - state_of_charge) / this->cgetMaxChargeRate();
  }
  else
  {
    return 0;
  }
}

inline float Agent::maxChargeTime() const noexcept
{
  return this->m_max_charge_time;
}

inline Agent Agent::chargeFor(const float time) const noexcept
{
  return Agent(this->cgetMaxStateOfCharge(),
               this->cgetMaxChargeRate(),
               this->cgetMaxTripLength(),
               this->maxChargeTime(),
               std::min<float>(this->cgetMaxStateOfCharge(), this->cgetCurrentStateOfCharge() + (time * this->cgetMaxChargeRate())),
               this->cgetCurrentTripLength(),
               this->cgetShiftStartTime(),
               this->cgetShiftEndTime(),
               this->cgetAuxiliaryPower());
}

inline void Agent::chargeForInPlace(const float time) noexcept
{
  this->m_state_of_charge = std::min<float>(this->cgetMaxStateOfCharge(), this->cgetCurrentStateOfCharge() + (time * this->cgetMaxChargeRate()));
}

inline Agent Agent::resetTripCounter() const noexcept
{
  return Agent(this->cgetMaxStateOfCharge(),
               this->cgetMaxChargeRate(),
               this->cgetMaxTripLength(),
               this->maxChargeTime(),
               this->cgetMaxStateOfCharge(),
               0,
               this->cgetShiftStartTime(),
               this->cgetShiftEndTime(),
               this->cgetAuxiliaryPower());
}

inline void Agent::resetTripCounterInPlace() noexcept
{
  this->m_cur_trip_length = 0;
  this->m_state_of_charge = this->cgetMaxStateOfCharge();
}

inline Agent Agent::propagateAlongTime(const float time_step) const noexcept
{
  return Agent(this->cgetMaxStateOfCharge(),
               this->cgetMaxChargeRate(),
               this->cgetMaxTripLength(),
               this->maxChargeTime(),
               this->cgetCurrentStateOfCharge() - (time_step * this->cgetAuxiliaryPower()),
               this->cgetCurrentTripLength(),
               this->cgetShiftStartTime(),
               this->cgetShiftEndTime(),
               this->cgetAuxiliaryPower());
}

inline void Agent::propagateAlongTimeInPlace(const float time_step) noexcept
{
  this->m_state_of_charge = this->cgetCurrentStateOfCharge() - (time_step * this->m_auxiliary_power_kws);
}

inline Agent Agent::propagateAlongPath(const graph::Path& path) const noexcept
{
  return Agent(this->cgetMaxStateOfCharge(),
               this->cgetMaxChargeRate(),
               this->cgetMaxTripLength(),
               this->maxChargeTime(),
               std::min<float>(this->cgetMaxStateOfCharge() - path.end_traversal_charge, this->cgetCurrentStateOfCharge() - path.traversal_charge),
               this->cgetCurrentTripLength() + path.length,
               this->cgetShiftStartTime(),
               this->cgetShiftEndTime(),
               this->cgetAuxiliaryPower());
}

inline void Agent::propagateAlongPathInPlace(const graph::Path& path) noexcept
{
  this->m_state_of_charge = std::min<float>(this->cgetMaxStateOfCharge() - path.end_traversal_charge, this->m_state_of_charge - path.traversal_charge);
  this->m_cur_trip_length += path.length;
}

inline Agent Agent::propagateAlongPath(const graph::Path& path, const float path_start_time, const float path_end_time) const noexcept
{
  assert((path_start_time - path.traversal_time) < 1e-8);
  assert((path_end_time   - path.traversal_time) < 1e-8);

  return Agent(this->cgetMaxStateOfCharge(),
               this->cgetMaxChargeRate(),
               this->cgetMaxTripLength(),
               this->maxChargeTime(),
               std::min<float>(this->cgetMaxStateOfCharge(), this->cgetCurrentStateOfCharge() - path.chargeUsedBetweenTimes(path_start_time, path_end_time)),
               this->cgetCurrentTripLength() + path.distanceTraversedBetweenTimes(path_start_time, path_end_time),
               this->cgetShiftStartTime(),
               this->cgetShiftEndTime(),
               this->cgetAuxiliaryPower());
}

inline void Agent::propagateAlongPathInPlace(const graph::Path& path, const float path_start_time, const float path_end_time) noexcept
{
  assert((path_start_time - path.traversal_time) < 1e-8);
  assert((path_end_time   - path.traversal_time) < 1e-8);

  this->m_state_of_charge = std::min<float>(this->cgetMaxStateOfCharge(), this->m_state_of_charge - path.chargeUsedBetweenTimes(path_start_time, path_end_time));
  this->m_cur_trip_length += path.distanceTraversedBetweenTimes(path_start_time, path_end_time);
}
} // plan

#endif
/* agent.hpp */
