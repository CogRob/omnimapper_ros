#pragma once

static inline const boost::posix_time::ptime RosTimeToBoost(
    const rclcpp::Time& t) {
  return boost::posix_time::ptime(
      boost::gregorian::date(1970, 1, 1),
      boost::posix_time::microseconds(t.nanoseconds() / 1000));
}
