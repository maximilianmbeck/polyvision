#pragma once

#include <cgal_utils.hpp>
#include <polyvision_cgal.hpp>

namespace polyvision {

class BeamDataGenerator {
public:
  // inputs
  Polygon_2 world_bounds_;
  Polygon_set_2 obstacles_;
  // Sensorbeams -> angle + direction vector
  std::vector<double> beam_angles_; // beam angles in radians -> rhs-coordinate
                                    // system -> x-axis is zero to the right
  std::vector<Vector_2>
      beam_directions_; // vectors representing the beam directions

  // intermediate results
  Polygon_set_2 world_poly_set_;

public:
  BeamDataGenerator(const Polygon_2 &world_bounds,
                    const Polygon_set_2 &obstacles,
                    const std::vector<Vector_2> &beam_directions,
                    const std::vector<double> &beam_angles);

  /**
   * @brief Get the Sensorbeam Readings At Pose object
   *
   * @param p the query point
   * @param theta in radians
   * @return std::vector<double>
   */
  std::vector<Point_2> getSensorbeamIntersectPointsAtPose(const Point_2 &p,
                                                  const double &theta);

  bool isPointInObstacles(const Point_2& p);

  bool isPointInWorld(const Point_2& p);

private:
  /**
   * @brief
   *
   * @param beam_dirs
   * @param theta in radians
   * @return std::vector<Vector_2>
   */
  std::vector<Vector_2> rotateBeamDirs(const std::vector<Vector_2> beam_dirs,
                                       const double &theta);

  std::vector<Ray_2>
  createSensorbeamsAtPoint(const Point_2 &p,
                           const std::vector<Vector_2> beam_dirs);
};

} // namespace polyvision
