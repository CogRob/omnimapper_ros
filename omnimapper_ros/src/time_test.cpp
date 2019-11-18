#include <rclcpp/rclcpp.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "boost/date_time/local_time/local_time.hpp"

#include "omnimapper_ros/ros_time_utils.h"

class TimeTest {
 public:
   std::shared_ptr<rclcpp::Node> n_;

  TimeTest(std::shared_ptr<rclcpp::Node> n) : n_(n) {
    rclcpp::Time t1_rclcpp = n_->now();
    std::cout << "rclcpp Time: " << t1_rclcpp.seconds() << std::endl;

    boost::posix_time::ptime t1_boost = omnimapper::rostime2ptime(t1_rclcpp);
    std::string t1_boost_str = boost::posix_time::to_simple_string(t1_boost);
    std::cout << "Boost Time: " << t1_boost << std::endl;

    rclcpp::Time t1_rclcpp_again = omnimapper::ptime2rostime (t1_boost);
    std::cout << "rclcpp Time: " << t1_rclcpp.seconds() << std::endl;

    boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970, 1, 1));
    boost::posix_time::time_duration diff = t1_boost - time_t_epoch;
    std::cout << "Since epoch: " << diff.total_nanoseconds() << std::endl;

    rclcpp::Time t1_rclcpp_again_again(diff.total_nanoseconds());
    std::cout << "rclcpp Time again: " << t1_rclcpp_again_again.seconds() << std::endl;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<rclcpp::Node>("time_test");

  TimeTest ttn(ros_node);

  rclcpp::spin(ros_node);
  rclcpp::shutdown();
  return 0;
}
