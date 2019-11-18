#pragma once

#ifndef OMNIMAPPER_ROS_TIME_UTILS_H_
#define OMNIMAPPER_ROS_TIME_UTILS_H_

#include <omnimapper/time.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "rclcpp/rclcpp.hpp"

namespace omnimapper {
/** Converts a ros Time to Boost posix time.  Just syntactic sugar to match our
 * other function. */
boost::posix_time::ptime rostime2ptime(rclcpp::Time r_time);

/** Converts a ptime to a rclcpp::Time.  This can be done by computing the total
 * nanoseconds since epoch. */
rclcpp::Time ptime2rostime(boost::posix_time::ptime p_time);

/** \brief An omnimapper compatible time functor, which will return the current
 * ros time. */
class GetROSTimeFunctor : public omnimapper::GetTimeFunctor {
 public:
  GetROSTimeFunctor(std::shared_ptr<rclcpp::Node> ros_node)
      : ros_node_(ros_node) {}
  omnimapper::Time operator()() { return (rostime2ptime(ros_node_->now())); }

 private:
  std::shared_ptr<rclcpp::Node> ros_node_;
};

}  // namespace omnimapper

#endif
