#pragma once
#include <memory>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "check_inside.hpp"
#include "visible_area.hpp"

#include <beamdata_gen.hpp>

namespace py = pybind11;

namespace polyvision {

// constexpr unsigned int precision = 6; // decimal places

class VisibleAreaPythonWrapper {
public:
  VisibleAreaPythonWrapper(const py::array_t<double> &origin,
                           const py::list &fieldsOfView,
                           const py::list &opaquePolygons);

  // Destructor
  ~VisibleAreaPythonWrapper() { delete visibleArea; }

  // Copyconstructor
  VisibleAreaPythonWrapper(const VisibleAreaPythonWrapper &other) {
    this->visibleArea = new VisibleArea(other.visibleArea->getOrigin(),
                                        other.visibleArea->getFieldsOfView(),
                                        other.visibleArea->getOpaquePolygons());
  }
  // Assignment operator
  VisibleAreaPythonWrapper &operator=(const VisibleAreaPythonWrapper &rhs) {
    if (this != &rhs) {
      this->visibleArea = new VisibleArea(rhs.visibleArea->getOrigin(),
                                          rhs.visibleArea->getFieldsOfView(),
                                          rhs.visibleArea->getOpaquePolygons());
    }
    return *this;
  }
  // Move copyconstructor
  VisibleAreaPythonWrapper(VisibleAreaPythonWrapper &&other) noexcept {
    this->visibleArea =
        other.visibleArea;       // "steal" others' data without copying
    other.visibleArea = nullptr; // delete other's pointer
  }
  // Move assignment operator
  VisibleAreaPythonWrapper &operator=(VisibleAreaPythonWrapper &&rhs) noexcept {
    if (this != &rhs) {
      std::swap(this->visibleArea, rhs.visibleArea);
      // this->visibleArea = rhs.visibleArea;
      // rhs.visibleArea = nullptr; // Wrong! content must be swapped, otherwise
      // memory leak
    }
    return *this;
  }

  void calculateVisibleArea();

  py::list getVisibleAreas() const;

  py::list getNonVisibleAreas() const;

  bool checkInside(py::array_t<double> &point) const;

  py::list getOpaquePolygons() const;

  py::list getFieldsOfView() const;

  py::array_t<double>
  getVisibilityBorder(const py::array_t<double> &line_xys) const;

  py::array_t<double> getOrigin() const;

private:
  VisibleArea *visibleArea;
};

class BeamDataGeneratorWrapper {
private:
  std::unique_ptr<BeamDataGenerator> beamDataGenerator;

public:
  BeamDataGeneratorWrapper(const py::array_t<double> &world_bound_poly,
                    const py::list &obstacles,
                    const py::array_t<double> &beam_directions);

  py::array_t<double>
  getSensorbeamIntersectPointsAtPose(const py::array_t<double> &p,
                                     const double &theta) const;

  bool isPointInObstacles(const py::array_t<double>& p) const;

  bool isPointInWorld(const py::array_t<double>& p) const;

  py::list getObstacles() const;

  py::array_t<double> getWorldBounds() const;
};

// functions
// CGAL conversion functions
Point_2 numpyArray2Point_2(const py::array_t<double> &point);

py::array_t<double> point_2_2NumpyArray(const Point_2 &point);

py::array_t<double> polygon_2_2NumpyArray(const Polygon_2 &polygon);

Polygon_2 numpyArray2Polygon_2(const py::array_t<double> &polygonArray);

py::list polygon_set_2_2ListOfNumpyArrays(const Polygon_set_2 &polygons);

Polygon_set_2 listOfNumpyArrays2Polygon_set_2(const py::list &polygonArrays);

// used in BeamDataGenerator
std::vector<Vector_2> numpyArray2VecOfVector_2(const py::array_t<double> & vecs);

py::array_t<double> point_2Vec2NumpyArray(const std::vector<Point_2>& points);

} // namespace polyvision