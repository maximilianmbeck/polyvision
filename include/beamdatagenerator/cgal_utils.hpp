#pragma once
#include "polyvision_cgal.hpp"
#include "gtest/gtest.h"
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <iostream>
#include <list>

namespace polyvision {

class CGALUtils {

public:
  std::list<Polygon_with_holes_2>
  convertPolygonset2PolygonList(const Polygon_set_2 &polygonset);
  bool segmentIntersectsSegment(const Segment_2 &segment_a,
                                const Segment_2 &segment_b, Point_2 &point);
  bool segmentIntersectsPolygon(const Polygon_set_2 &polyset,
                                const Segment_2 &line, Point_2 &point);

  bool pointInsidePolygon(const Point_2 &p, const Polygon_2 &polygon);

  bool rayIntersectsPolygonFromInside(const Ray_2 &ray,
                                      const Polygon_2 &polygon,
                                      Point_2 &intersect);

  Arrangement_2
  polygon_with_holes2arrangement_2(const Polygon_with_holes_2 &polywh);

  Polygon_set_2 createWorldPolySet_2(const Polygon_2 &world_bounds,
                                     const Polygon_set_2 &obstacles);

  Polygon_2
  createVisibilityPolygonFromPoint(const Polygon_set_2 &world_poly_set,
                                   const Point_2 &p);
};
} // namespace polyvision