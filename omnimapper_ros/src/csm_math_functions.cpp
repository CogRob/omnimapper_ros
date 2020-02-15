#include "omnimapper_ros/csm_math_functions.h"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

bool CheckMoveFarEnough(
    const geometry_msgs::msg::TransformStamped& last_pose_msg,
    const geometry_msgs::msg::TransformStamped& curr_pose_msg, double distance,
    double rot_thresh) {
  tf2::Transform last_pose, curr_pose;
  tf2::fromMsg(last_pose_msg.transform, last_pose);
  tf2::fromMsg(curr_pose_msg.transform, curr_pose);

  double dx = curr_pose.getOrigin().x() - last_pose.getOrigin().x();
  double dy = curr_pose.getOrigin().y() - last_pose.getOrigin().y();
  double dz = curr_pose.getOrigin().z() - last_pose.getOrigin().z();
  double dist = sqrt(dx * dx + dy * dy + dz * dz);
  double ang_dist = fabs(WrapToPi(tf2::getYaw(last_pose.getRotation()) -
                                  tf2::getYaw(curr_pose.getRotation())));
  return (dist >= distance) || (ang_dist >= rot_thresh);
}

gtsam::Pose3 GetPose(
    const geometry_msgs::msg::TransformStamped& transform_msg) {
  tf2::Transform transform;
  tf2::fromMsg(transform_msg.transform, transform);

  tf2::Vector3 axis = transform.getRotation().getAxis();
  gtsam::Vector gtsam_axis(3);
  double axis_norm =
      sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
  if (fabs(axis_norm - 1.0) < 0.01) {
    gtsam_axis << axis[0] / axis_norm, axis[1] / axis_norm,
                  axis[2] / axis_norm;
  } else {
    printf("Axis failure !  Axis length %lf\n", axis_norm);
    gtsam_axis << 0, 0, 1;
  }
  double angle = transform.getRotation().getAngle();
  return gtsam::Pose3(
      gtsam::Rot3::rodriguez(gtsam_axis, angle),
      gtsam::Point3(transform.getOrigin().x(), transform.getOrigin().y(),
                    transform.getOrigin().z()));
}
gtsam::Pose3 GetRelativePose(
    const geometry_msgs::msg::TransformStamped& last_pose,
    const geometry_msgs::msg::TransformStamped& curr_pose) {
  gtsam::Pose3 initial = GetPose(last_pose);
  gtsam::Pose3 final = GetPose(curr_pose);

  gtsam::Pose3 odo = initial.between(final);

  return odo;
}

tf2::Transform Pose3ToTransform(const gtsam::Pose3& ps) {
  tf2::Transform t;
  t.setOrigin(tf2::Vector3(ps.x(), ps.y(), ps.z()));
  tf2::Quaternion q;
  gtsam::Vector ypr = ps.rotation().ypr();
  q.setRPY(ypr[2], ypr[1], ypr[0]);
  t.setRotation(q);
  return t;
}
gtsam::Pose3 TransformToPose3(const tf2::Transform& t) {
  double roll, pitch, yaw;
  tf2::Matrix3x3 m(t.getRotation());
  m.getRPY(roll, pitch, yaw);
  return gtsam::Pose3(
      gtsam::Rot3::ypr(yaw, pitch, roll),
      gtsam::Point3(t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z()));
}

tf2::Transform Pose2ToTransform(const gtsam::Pose2& ps) {
  tf2::Transform t;
  t.setOrigin(tf2::Vector3(ps.x(), ps.y(), 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, ps.theta());
  t.setRotation(q);
  return t;
}
gtsam::Pose2 TransformToPose2(const tf2::Transform& t) {
  double roll, pitch, yaw;
  tf2::Matrix3x3 m(t.getRotation());
  m.getRPY(roll, pitch, yaw);
  return gtsam::Pose2(yaw, gtsam::Point2(t.getOrigin().x(), t.getOrigin().y()));
}

gtsam::Point3 btVectorToPoint3(const tf2::Vector3& vec) {
  return gtsam::Point3(vec.getX(), vec.getY(), vec.getZ());
}
gtsam::Pose3 btTransformToPose3(const tf2::Transform& transform) {
  tf2::Vector3 col0 = transform.getBasis().getColumn(0);
  tf2::Vector3 col1 = transform.getBasis().getColumn(1);
  tf2::Vector3 col2 = transform.getBasis().getColumn(2);
  gtsam::Rot3 rot(btVectorToPoint3(col0), btVectorToPoint3(col1),
                  btVectorToPoint3(col2));
  gtsam::Point3 trans = btVectorToPoint3(transform.getOrigin());
  return gtsam::Pose3(rot, trans);
}

double WrapToPi(double diff) {
  double ret = diff;
  while (ret > M_PI) {
    ret -= 2.0 * M_PI;
  }
  while (ret < -M_PI) {
    ret += 2.0 * M_PI;
  }
  return ret;
}

gtsam::noiseModel::Diagonal::shared_ptr GetOdoCovariance(
    const gtsam::Pose3& odo, double sroll, double spitch, double syaw,
    double sx, double sy, double sz) {
  double l_scale =
      sqrt(odo.x() * odo.x() + odo.y() * odo.y() + odo.z() * odo.z()) / 1.5;
  if (l_scale < 0.1) l_scale = 0.1;
  gtsam::Vector ypr = odo.rotation().ypr();
  double a_scale =
      sqrt(ypr[0] * ypr[0] + ypr[1] * ypr[1] + ypr[2] * ypr[2]) / 1.5;
  if (a_scale < 0.1) a_scale = 0.1;

  gtsam::Vector noise_vector(6);
  noise_vector <<  a_scale*(sroll),
    a_scale*(spitch),
    a_scale*(syaw),
    l_scale*(sx),
    l_scale*(sy),
    l_scale*(sz);

  return gtsam::noiseModel::Diagonal::Sigmas(noise_vector);
}
gtsam::Matrix CovarFromDiagonalNoiseModel(
    const gtsam::SharedDiagonal& diagonal) {
  const gtsam::Vector sigmas = diagonal->sigmas();
  gtsam::Matrix result = gtsam::zeros(sigmas.size(), sigmas.size());
  for (unsigned int i = 0; i < sigmas.size(); i++) {
    result(i, i) = sigmas(i) * sigmas(i);
  }
  return result;
}
gtsam::Pose2 Pose3ToPose2(const gtsam::Pose3& p3) {
  gtsam::Vector ypr = p3.rotation().ypr();
  return gtsam::Pose2(ypr[0], gtsam::Point2(p3.x(), p3.y()));
}
gtsam::Pose3 Pose2ToPose3(const gtsam::Pose2& p2) {
  return gtsam::Pose3(gtsam::Rot3::ypr(p2.theta(), 0, 0),
                      gtsam::Point3(p2.x(), p2.y(), 0.0));
}
