#pragma once

#include <Eigen/Dense>
#include <vector>

#include "icp/eigen_kdtree.h"
#include "map.h"

class Localizer2D {
 public:
  using PointType = Eigen::Vector2f;
  using ContainerType =
      std::vector<PointType, Eigen::aligned_allocator<PointType>>;

  Localizer2D();

  /**
   * @brief Set the internal map reference and constructs the KD-Tree containing
   * obstacles coordinates for fast access.
   *
   * @param map_
   */
  void setMap(std::shared_ptr<Map> map_);
  /**
   * @brief Set the current estimate for laser_in_world
   *
   * @param initial_pose_
   */
  void setInitialPose(const Eigen::Isometry2f& initial_pose_);
  /**
   * @brief Process the input scan.
   * First creates a prediction using the current laser_in_world estimate
   *
   * @param scan_
   */
  void process(const ContainerType& scan_);
  /**
   * @brief Set the parameters of the laser scanner. Used to predict
   * measurements.
   * These parameters should be taken from the incoming sensor_msgs::LaserScan
   * message
   *
   * For further documentation, refer to:
   * http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
   *
   *
   * @param range_min_
   * @param range_max_
   * @param angle_min_
   * @param angle_max_
   * @param angle_increment_
   */
  void setLaserParams(float range_min_, float range_max_, float angle_min_,
                      float angle_max_, float angle_increment_);

  /**
   * @brief Returns the current estimate of laser pose in world frame
   *
   * @return const Eigen::Isometry2f&
   */
  inline const Eigen::Isometry2f& X() const { return _laser_in_world; }

 protected:
  std::shared_ptr<Map> _map;
  Eigen::Isometry2f _laser_in_world;

  // Obstacles vector
  ContainerType _obst_vect;
  using TreeType = TreeNode_<ContainerType::iterator>;
  // Obstacle KD-Tree for fast NN-search
  std::shared_ptr<TreeType> _obst_tree_ptr;

  // Laser scanner properties
  float _range_min, _range_max;
  float _angle_min, _angle_max, _angle_increment;

  /**
   * @brief Computes the predicted scan at the current laser_in_world pose
   * estimate.
   *
   *
   * @param dest_ Output predicted scan
   */
  void getPrediction(ContainerType& dest_);
};
