#ifndef OUSTER_ROS_MOTION_CORRECTION_H_
#define OUSTER_ROS_MOTION_CORRECTION_H_

#include <kindr/minimal/quat-transformation.h>
#include "ouster_ros/point_os1.h"

namespace ouster_ros {
namespace OS1 {

typedef kindr::minimal::QuatTransformation Transformation;

template <typename Type>
using AlignedList = std::list<Type, Eigen::aligned_allocator<Type>>;

class MotionCorrection {
 public:
  MotionCorrection(const bool output_reflectivity_as_intensity = false,
                   const bool remove_old_transformations = true)
      : output_reflectivity_as_intensity_(output_reflectivity_as_intensity),
        remove_old_transformations_(remove_old_transformations) {}

  void addTransformation(
      const ros::Time& stamp,
      const kindr::minimal::QuatTransformation& transformation) {
    const uint64_t stamp_us = static_cast<uint64_t>(stamp.sec) * 1000000ul +
                              static_cast<uint64_t>(stamp.nsec) / 1000ul;

    if (!transformation_list_.empty() &&
        stamp_us < transformation_list_.back().stamp) {
      ROS_ERROR_STREAM(
          "Detected jump backwards in time, resetting transformation list");
      transformation_list_.clear();
    }

    TransformationStamped transformation_stamped;
    transformation_stamped.transformation = transformation;
    transformation_stamped.stamp = stamp_us;
    transformation_list_.push_back(transformation_stamped);

    if (transformation_list_.size() == 1) {
      it_ = transformation_list_.begin();
    }
  }

  void addPointCloudToQueue(
      const std::shared_ptr<pcl::PointCloud<PointOS1>>& pointcloud_ptr) {
    if (!pointcloud_list_.empty() &&
        pointcloud_ptr->header.stamp < pointcloud_list_.back()->header.stamp) {
      ROS_ERROR_STREAM(
          "Detected jump backwards in time, resetting pointcloud list");
    }

    pointcloud_list_.push_back(pointcloud_ptr);
  }

  bool processNextQueuedPointcloud(
      pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
    while (!pointcloud_list_.empty()) {
      const InterpolationStatus status =
          TransformPoints(*pointcloud_list_.front(), pointcloud);

      if (status == InterpolationStatus::MATCHED) {
        return true;
      }
      if ((status == InterpolationStatus::AFTER_LAST) ||
          (status == InterpolationStatus::NOT_ENOUGH_POINTS)) {
        return false;
      }
      if (status == InterpolationStatus::BEFORE_FIRST) {
        pointcloud_list_.pop_front();
      }
    }
    return false;
  }

 private:
  enum class InterpolationStatus {
    NOT_ENOUGH_POINTS,
    BEFORE_FIRST,
    MATCHED,
    AFTER_LAST
  };

  InterpolationStatus TransformPoints(
      const pcl::PointCloud<PointOS1> pointcloud_in,
      pcl::PointCloud<pcl::PointXYZI>* pointcloud_out) {
    pointcloud_out->header = pointcloud_in.header;

    ros::Time input_time;
    const uint64_t first_point_stamp =
        pointcloud_in.header.stamp + pointcloud_in.front().time_offset_us;
    const uint64_t last_point_stamp =
        pointcloud_in.header.stamp + pointcloud_in.back().time_offset_us;
    pointcloud_out->header.stamp =
        pointcloud_in.header.stamp + (pointcloud_in.front().time_offset_us +
                                      pointcloud_in.back().time_offset_us) /
                                         2;

    if (transformation_list_.size() < 2) {
      return InterpolationStatus::NOT_ENOUGH_POINTS;
    }
    if (transformation_list_.front().stamp > first_point_stamp) {
      return InterpolationStatus::BEFORE_FIRST;
    }
    if (transformation_list_.back().stamp < last_point_stamp) {
      return InterpolationStatus::AFTER_LAST;
    }

    for (const PointOS1& point : pointcloud_in) {
      const uint64_t point_stamp =
          pointcloud_in.header.stamp + point.time_offset_us;

      while (it_->stamp < point_stamp) {
        ++it_;
      }

      const TransformationStamped& transformation_after = *it_;
      const TransformationStamped& transformation_before = *std::prev(it_);

      // Interpolate between the two transformations using the exponential
      // map.
      const double t_delta_ratio =
          static_cast<double>(point_stamp - transformation_before.stamp) /
          static_cast<double>(transformation_after.stamp -
                              transformation_before.stamp);

      const kindr::minimal::QuatTransformation::Vector6 delta_vector =
          (transformation_before.transformation.inverse() *
           transformation_after.transformation)
              .log();

      const kindr::minimal::QuatTransformation delta_transforamtion =
          Transformation::exp(t_delta_ratio * delta_vector);

      const kindr::minimal::Position original_position(point.x, point.y,
                                                       point.z);
      const kindr::minimal::Position interpolated_position =
          delta_transforamtion * original_position;

      pcl::PointXYZI output_point;
      output_point.x = interpolated_position.x();
      output_point.y = interpolated_position.y();
      output_point.z = interpolated_position.z();

      if (output_reflectivity_as_intensity_) {
        output_point.intensity = point.reflectivity;
      } else {
        output_point.intensity = point.intensity;
      }
    }

    if (remove_old_transformations_) {
      while (transformation_list_.begin() != std::prev(it_)) {
        transformation_list_.pop_front();
      }
    }

    return InterpolationStatus::MATCHED;
  }

 private:
  struct TransformationStamped {
    uint64_t stamp;
    kindr::minimal::QuatTransformation transformation;
  };
  const bool output_reflectivity_as_intensity_;
  const bool remove_old_transformations_;

  AlignedList<std::shared_ptr<pcl::PointCloud<PointOS1>>> pointcloud_list_;
  AlignedList<TransformationStamped> transformation_list_;
  AlignedList<TransformationStamped>::const_iterator it_;
};

}  // namespace OS1
}  // namespace ouster_ros

#endif  // OUSTER_ROS_MOTION_CORRECTION_H_
