#include "omnimapper_ros/ros_tf_utils.h"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

gtsam::Pose3 omnimapper::tf2pose3(
    const geometry_msgs::msg::TransformStamped& transform_msg) {
  tf2::Transform transform;
  tf2::fromMsg(transform_msg.transform, transform);

  tf2::Vector3 axis = transform.getRotation().getAxis();
  double len = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
  assert(len != 0);
  gtsam::Vector gtsam_axis =
      (gtsam::Vector(3) << axis[0] / len, axis[1] / len, axis[2] / len);
  double angle = transform.getRotation().getAngle();
  return gtsam::Pose3(
      gtsam::Rot3::rodriguez(gtsam_axis, angle),
      gtsam::Point3(transform.getOrigin().x(), transform.getOrigin().y(),
                    transform.getOrigin().z()));
}

tf2::Transform omnimapper::pose3totf(gtsam::Pose3& pose) {
  tf2::Transform t;
  t.setOrigin(tf2::Vector3(pose.x(), pose.y(), pose.z()));
  tf2::Quaternion q;
  gtsam::Vector ypr = pose.rotation().ypr();
  q.setRPY(ypr[2], ypr[1], ypr[0]);
  t.setRotation(q);
  return t;
}
