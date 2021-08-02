#include <cmath>
#include <iostream>
#include <list>

#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/intersections.h>

#include <beamdata_gen.hpp>
#include <cgal_debug_utils.hpp>
#include <cgal_utils.hpp>
#include <polyvision_cgal.hpp>

#include "gtest/gtest.h"

using namespace polyvision;

TEST(CGALTest, VisibilityCalculation) {
  Polygon_2 world_bounds;
  world_bounds.push_back(Point_2(1, 1));
  world_bounds.push_back(Point_2(10, 1));
  world_bounds.push_back(Point_2(10, 10));
  world_bounds.push_back(Point_2(1, 10));

  Polygon_2 rect1;
  rect1.push_back(Point_2(4, 5));
  rect1.push_back(Point_2(7, 5));
  rect1.push_back(Point_2(7, 7));
  rect1.push_back(Point_2(4, 7));

  Polygon_2 rect2;
  rect2.push_back(Point_2(2, 8));
  rect2.push_back(Point_2(3, 8));
  rect2.push_back(Point_2(3, 9));
  rect2.push_back(Point_2(2, 9));

  Polygon_2 rect3;
  rect3.push_back(Point_2(6, 4));
  rect3.push_back(Point_2(7, 4));
  rect3.push_back(Point_2(7, 6));
  rect3.push_back(Point_2(6, 6));

  Polygon_2 rect4;
  rect4.push_back(Point_2(9, 3));
  rect4.push_back(Point_2(11, 3));
  rect4.push_back(Point_2(11, 4));
  rect4.push_back(Point_2(9, 4));

  Polygon_set_2 obstacles;
  obstacles.insert(rect1);
  obstacles.join(rect2);
  obstacles.join(rect3);
  obstacles.join(rect4);

  Point_2 p1(2, -1);
  Point_2 p2(2, 2);
  Point_2 p3(3, -3);
  Point_2 p4(3, 3);
  Point_2 p5(3.0, -3.0);
  Point_2 p6(3.0, -6.0);
  Segment_2 segment1(p1, p2);
  Segment_2 segment2(p3, p4);
  Segment_2 segment3(p5, p6);

  CGALDebugUtils cgalDebugUtils;
  CGALUtils cgalUtils;

  cgalDebugUtils.printPolygon_set_2(obstacles);
  // cgalDebugUtils.printPolygon_set_2(world_bounds);

  Polygon_set_2 world;
  world = cgalUtils.createWorldPolySet_2(world_bounds, obstacles);

  cgalDebugUtils.printPolygon_set_2(world);

  // visibility calculation
  Point_2 p(7, 2);

  Polygon_2 visA;
  visA = cgalUtils.createVisibilityPolygonFromPoint(world, p);

  cgalDebugUtils.printPolygon_2(visA);

  std::cout << "Point inside visA: " << cgalUtils.pointInsidePolygon(p, visA)
            << std::endl;

  Vector_2 v(0, -1);
  Ray_2 r(p, v);

  Point_2 res;

  cgalUtils.rayIntersectsPolygonFromInside(r, visA, res);
  std::cout << "intersect: " << res << std::endl;
}

TEST(CGALTest, Transformations) {
  const double pi = std::acos(-1);
  Vector_2 v(1, 0);

  Aff_Transformation_2 rotate(CGAL::ROTATION, std::sin(pi / 2),
                              std::cos(pi / 2));

  Vector_2 r;
  r = v.transform(rotate);

  std::cout << "(" << v << ")"
            << " rotate = "
            << "(" << r << ")" << std::endl;
}