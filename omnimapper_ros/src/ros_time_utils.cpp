#include "omnimapper_ros/ros_time_utils.h"

#include <boost/date_time/posix_time/posix_time.hpp>

#include "rclcpp/rclcpp.hpp"

namespace omnimapper {

const boost::posix_time::ptime rostime2ptime(const rclcpp::Time& r_time) {
  return boost::posix_time::ptime(
      boost::gregorian::date(1970, 1, 1),
      boost::posix_time::microseconds(r_time.nanoseconds() / 1000));
}

const rclcpp::Time ptime2rostime(const boost::posix_time::ptime& p_time) {
  boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970, 1, 1));
  boost::posix_time::time_duration diff = p_time - time_t_epoch;
  rclcpp::Time r_time(diff.total_nanoseconds());
  return r_time;
}

}  // namespace omnimapper
