/**
 * @File: nodes_edges.cpp
 * @Date: March 2024
 * @Author: James Swedeen
 **/

/* C++ Headers */
#include<cmath>
#include<iostream> // TODO

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<street_graph/nodes_edges.hpp>

namespace graph
{
void Edge::generateStraightSegments(Eigen::Matrix<float,3,Eigen::Dynamic>& points,
                                    std::vector<float>&                    time_points) const
{
  points.resize(3, 2);
  time_points.resize(2);
  points.col(0) = this->cgetFromNode()->cgetPosition();
  points.col(1) = this->cgetToNode()->  cgetPosition();
  time_points[0] = 0;
  time_points[1] = this->cgetMinTraversalTime();
}

void Edge::generateDiscreetSteps(const float                            length_step,
                                 Eigen::Matrix<float,3,Eigen::Dynamic>& output_vec) const
{
  const Eigen::Index size = std::max<Eigen::Index>(2, std::round(this->cgetLength()/length_step));

  output_vec.resize(3, size);

  for(Eigen::Index dim_it = 0; dim_it < 3; ++dim_it)
  {
    output_vec.row(dim_it).setLinSpaced(size,
                                        this->cgetFromNode()->cgetPosition()[dim_it],
                                        this->cgetToNode()->  cgetPosition()[dim_it]);
  }
  output_vec.leftCols<1>()  = this->cgetFromNode()->cgetPosition();
  output_vec.rightCols<1>() = this->cgetToNode()->  cgetPosition();
}

float Edge::findPointInterceptTime(const Eigen::Ref<const Eigen::Matrix<float,3,1>>& point,
                                   const float                                       max_position_error) const
{
  const Eigen::Matrix<float,3,1> centered_point = point - this->cgetFromNode()->cgetPosition();

  const float dist_along_edge = centered_point.transpose() * (this->cgetToNode()->cgetPosition() - this->cgetFromNode()->cgetPosition()).normalized();
  const float dist_from_line  = centered_point.norm() - dist_along_edge;

  if((dist_along_edge < -max_position_error) or
     (dist_along_edge > (this->cgetLength() + max_position_error)) or
     (dist_from_line  > max_position_error))
  {
    return std::numeric_limits<float>::quiet_NaN();
  }

  return dist_along_edge / this->cgetSpeedLimit();
}

void SuperEdge::generateStraightSegments(Eigen::Matrix<float,3,Eigen::Dynamic>& points,
                                         std::vector<float>&                    time_points) const
{
  Eigen::Matrix<float,3,Eigen::Dynamic> sub_points1;
  Eigen::Matrix<float,3,Eigen::Dynamic> sub_points2;
  std::vector<float>                    sub_times2;

  this->cgetSubEdges().first-> generateStraightSegments(sub_points1, time_points);
  this->cgetSubEdges().second->generateStraightSegments(sub_points2, sub_times2);

  const Eigen::Index first_len  = sub_points1.cols();
  const Eigen::Index second_len = sub_points2.cols();

  points.resize(3, first_len+second_len-1);
  time_points.reserve(first_len+second_len-1);
  points.leftCols( first_len)  = sub_points1;
  points.rightCols(second_len) = sub_points2;
  for(size_t it = 1; it < second_len; ++it)
  {
    time_points.emplace_back(sub_times2[it] + this->cgetSubEdges().first->cgetMinTraversalTime());
  }
}

void SuperEdge::generateDiscreetSteps(const float                            length_step,
                                      Eigen::Matrix<float,3,Eigen::Dynamic>& output_vec) const
{
  Eigen::Matrix<float,3,Eigen::Dynamic> out_first;
  Eigen::Matrix<float,3,Eigen::Dynamic> out_second;
  this->cgetSubEdges().first-> generateDiscreetSteps(length_step, out_first);
  this->cgetSubEdges().second->generateDiscreetSteps(length_step, out_second);
  const Eigen::Index first_len  = out_first. cols();
  const Eigen::Index second_len = out_second.cols();
  output_vec.resize(3, first_len+second_len);
  output_vec.leftCols( first_len)  = out_first;
  output_vec.rightCols(second_len) = out_second;
}

Eigen::Matrix<float,3,1> SuperEdge::stateAtTime(const float time_spent) const
{
  assert((time_spent - this->cgetMinTraversalTime()) < 1e-4);

  if(this->cgetSubEdges().first->cgetMinTraversalTime() >= time_spent)
  {
    return this->cgetSubEdges().first->stateAtTime(time_spent);
  }
  else
  {
    return this->cgetSubEdges().second->stateAtTime(time_spent - this->cgetSubEdges().first->cgetMinTraversalTime());
  }
}

float SuperEdge::chargeUsedAtTime(const float time_spent) const
{
  assert((time_spent - this->cgetMinTraversalTime()) < 1e-4);

  if(this->cgetSubEdges().first->cgetMinTraversalTime() >= time_spent)
  {
    return this->cgetSubEdges().first->chargeUsedAtTime(time_spent);
  }
  else
  {
    return this->cgetSubEdges().first ->cgetTraversalCharge() +
           this->cgetSubEdges().second->chargeUsedAtTime(time_spent - this->cgetSubEdges().first->cgetMinTraversalTime());
  }
}

float SuperEdge::chargeUsedBetweenTimes(const float start_time, const float end_time) const
{
  assert(start_time >= -1e-4);
  assert((end_time - this->cgetMinTraversalTime()) < 1e-4);
  assert(start_time <= end_time);

  const bool start_after = start_time >= this->cgetSubEdges().first->cgetMinTraversalTime();
  const bool end_after   = end_time   >= this->cgetSubEdges().first->cgetMinTraversalTime();

  if(start_after and end_after)
  {
    return this->cgetSubEdges().second->chargeUsedBetweenTimes(start_time - this->cgetSubEdges().first->cgetMinTraversalTime(),
                                                               end_time   - this->cgetSubEdges().first->cgetMinTraversalTime());
  }
  else if((not start_after) and (not end_after))
  {
    return this->cgetSubEdges().first->chargeUsedBetweenTimes(start_time, end_time);
  }
  else // start is in first and end is in second
  {
    return this->cgetSubEdges().first-> chargeUsedBetweenTimes(start_time, this->cgetSubEdges().first->cgetMinTraversalTime()) +
           this->cgetSubEdges().second->chargeUsedAtTime(end_time - this->cgetSubEdges().first->cgetMinTraversalTime());
  }
}

float SuperEdge::distanceTraversedBetweenTimes(const float start_time, const float end_time) const
{
  assert(start_time >= -1e-4);
  assert((end_time - this->cgetMinTraversalTime()) < 1e-4);
  assert(start_time <= end_time);

  const bool start_after = start_time >= this->cgetSubEdges().first->cgetMinTraversalTime();
  const bool end_after   = end_time   >= this->cgetSubEdges().first->cgetMinTraversalTime();

  if(start_after and end_after)
  {
    return this->cgetSubEdges().second->distanceTraversedBetweenTimes(start_time - this->cgetSubEdges().first->cgetMinTraversalTime(),
                                                                      end_time   - this->cgetSubEdges().first->cgetMinTraversalTime());
  }
  else if((not start_after) and (not end_after))
  {
    return this->cgetSubEdges().first->distanceTraversedBetweenTimes(start_time, end_time);
  }
  else // start is in first and end is in second
  {
    return this->cgetSubEdges().first-> distanceTraversedBetweenTimes(start_time, this->cgetSubEdges().first->cgetMinTraversalTime()) +
           this->cgetSubEdges().second->distanceTraversedBetweenTimes(0,          end_time - this->cgetSubEdges().first->cgetMinTraversalTime());
  }
}

float SuperEdge::findPointInterceptTime(const Eigen::Ref<const Eigen::Matrix<float,3,1>>& point,
                                        const float                                       max_position_error) const
{
  const float first_time = this->cgetSubEdges().first->findPointInterceptTime(point, max_position_error);
  if(not std::isnan(first_time))
  {
    return first_time;
  }

  const float second_time = this->cgetSubEdges().second->findPointInterceptTime(point, max_position_error);

  return second_time + this->cgetSubEdges().first->cgetMinTraversalTime();
}
} // graph

/* nodes_edges.cpp */
