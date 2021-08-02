#include <beamdata_gen.hpp>
#include <cgal_debug_utils.hpp>
#include <polyvision_cgal.hpp>
#include <vector>
namespace polyvision {

BeamDataGenerator::BeamDataGenerator(
    const Polygon_2 &world_bounds, const Polygon_set_2 &obstacles,
    const std::vector<Vector_2> &beam_directions,
    const std::vector<double> &beam_angles)
    : world_bounds_(world_bounds), obstacles_(obstacles),
      beam_angles_(beam_angles), beam_directions_(beam_directions) {
  CGALUtils cgalUtils;
  this->world_poly_set_ =
      cgalUtils.createWorldPolySet_2(this->world_bounds_, this->obstacles_);
  // CGALDebugUtils cgaldebugutils;
  // cgaldebugutils.printPolygon_set_2(this->world_poly_set_);
}

std::vector<Point_2>
BeamDataGenerator::getSensorbeamIntersectPointsAtPose(const Point_2 &p,
                                                      const double &theta) {
  CGALUtils cgalUtils;
  std::vector<Point_2> beam_intersects;

  // std::cout << "Pose: " << p << ", " << theta << std::endl;

  if (this->isPointInObstacles(p)) {
    std::cout << "ERROR: Point (" << p << ") in Obstacle!" << std::endl;
    return beam_intersects;
  }

  // calculate visibility polygon
  Polygon_2 visA;
  visA = cgalUtils.createVisibilityPolygonFromPoint(this->world_poly_set_, p);

  // CGALDebugUtils cgaldebugutils;
  // std::cout << "visibility poly: " << std::endl;
  // cgaldebugutils.printPolygon_2(visA);

  // generate sensor beams
  std::vector<Ray_2> sensorbeams = this->createSensorbeamsAtPoint(
      p, this->rotateBeamDirs(this->beam_directions_, theta));

  

  // calculate beam intersection points
  beam_intersects.reserve(sensorbeams.size());

  for (const auto &b : sensorbeams) {
    Point_2 intersect;
    cgalUtils.rayIntersectsPolygonFromInside(b, visA, intersect);
    beam_intersects.emplace_back(intersect);
  }

  return beam_intersects;
}

bool BeamDataGenerator::isPointInObstacles(const Point_2 &p) {
  Polygon_with_holes_2 polyPInside;
  if (this->obstacles_.locate(p, polyPInside)) {
    return true;
  } else {
    return false;
  }
}

bool BeamDataGenerator::isPointInWorld(const Point_2& p) {
  CGALUtils cgalUtils;
  return cgalUtils.pointInsidePolygon(p, this->world_bounds_);
}

std::vector<Vector_2>
BeamDataGenerator::rotateBeamDirs(const std::vector<Vector_2> beam_dirs,
                                  const double &theta) {
  std::vector<Vector_2> rotated_beam_dirs;
  rotated_beam_dirs.reserve(beam_dirs.size());

  Aff_Transformation_2 rotate(CGAL::ROTATION, std::sin(theta), std::cos(theta));

  for (const auto &v : beam_dirs) {
    rotated_beam_dirs.emplace_back(v.transform(rotate));
  }
  return rotated_beam_dirs;
}

std::vector<Ray_2> BeamDataGenerator::createSensorbeamsAtPoint(
    const Point_2 &p, const std::vector<Vector_2> beam_dirs) {
  std::vector<Ray_2> sensorbeams;
  sensorbeams.reserve(beam_dirs.size());

  for (const auto &v : beam_dirs) {
    sensorbeams.emplace_back(Ray_2(p, v));
  }

  return sensorbeams;
}
} // namespace polyvision