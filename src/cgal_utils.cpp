#include "cgal_utils.hpp"
#include "polyvision_cgal.hpp"
#include "gtest/gtest.h"
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Triangular_expansion_visibility_2.h>
#include <CGAL/intersections.h>
#include <CGAL/squared_distance_2.h>

#include <iostream>
#include <list>

namespace polyvision {

// Define the used visibility class
typedef CGAL::Triangular_expansion_visibility_2<Arrangement_2> TriExpVisibility;

std::list<Polygon_with_holes_2>
CGALUtils::convertPolygonset2PolygonList(const Polygon_set_2 &polygonset) {
  std::list<Polygon_with_holes_2> polygonList;
  // insert all Polygons_with_holes from polygonset
  polygonset.polygons_with_holes(std::back_inserter(polygonList));
  return polygonList;
}

bool CGALUtils::segmentIntersectsSegment(const Segment_2 &segment_a,
                                         const Segment_2 &segment_b,
                                         Point_2 &point) {
  CGAL::cpp11::result_of<Intersect_2(Segment_2, Segment_2)>::type result =
      intersection(segment_a, segment_b);
  if (result) {
    if (const Segment_2 *s = boost::get<Segment_2>(&*result)) {
      point = s->point(0);
    } else {
      point = *boost::get<Point_2>(&*result);
    }
    return true;
  } else {
    return false;
  }
}

bool CGALUtils::segmentIntersectsPolygon(const Polygon_set_2 &polygonset,
                                         const Segment_2 &segment,
                                         Point_2 &point) {
  std::list<Polygon_with_holes_2> polygonList =
      convertPolygonset2PolygonList(polygonset);
  std::list<Polygon_with_holes_2>::const_iterator it;

  for (it = polygonList.begin(); it != polygonList.end(); it++) {
    if (!it->is_unbounded()) {

      size_t i = 0;
      Polygon_2::Edge_const_iterator eit;
      for (eit = it->outer_boundary().edges_begin();
           eit != it->outer_boundary().edges_end(); eit++) {
        Segment_2 seg(eit->point(0), eit->point(1));
        bool valid = segmentIntersectsSegment(seg, segment, point);
        if (valid) {
          return true;
        }
        i++;
      }
    }
  }
  return false;
}

bool CGALUtils::pointInsidePolygon(const Point_2 &p, const Polygon_2 &polygon) {
  bool isInside = false;
  switch (polygon.bounded_side(p)) {
  case CGAL::ON_BOUNDED_SIDE:
    isInside = true;
    break;
  case CGAL::ON_BOUNDARY:
    isInside = true;
    break;
  case CGAL::ON_UNBOUNDED_SIDE:
    isInside = false;
  }
  return isInside;
}

bool CGALUtils::rayIntersectsPolygonFromInside(const Ray_2 &ray,
                                               const Polygon_2 &polygon,
                                               Point_2 &intersect) {
  // check if ray's source inside polygon
  Point_2 raysource = ray.source();
  if (!this->pointInsidePolygon(raysource, polygon)) {
    return false;
  }
  Polygon_2::Edge_const_iterator eit;
  for (eit = polygon.edges_begin(); eit != polygon.edges_end(); eit++) {
    Segment_2 seg(eit->point(0), eit->point(1));

    // check intersection with ray
    const auto intersect_result = CGAL::intersection(ray, seg);
    if (intersect_result) {
      if (const Segment_2 *s = boost::get<Segment_2>(&*intersect_result)) {
        // intersection is a segment (should almost never happen)
        // if it happens return the point closer to raysource
        if (CGAL::squared_distance(raysource, s->source()) <
            CGAL::squared_distance(raysource, s->target())) {
          intersect = s->source();
        } else {
          intersect = s->target();
        }
      } else {
        // intersection is a point
        const Point_2 *p = boost::get<Point_2>(&*intersect_result);
        intersect = *p;
      }
      return true;
    }
  }
  return false;
}

Arrangement_2 CGALUtils::polygon_with_holes2arrangement_2(
    const Polygon_with_holes_2 &polywh) {
  Arrangement_2 arr;
  CGAL::insert_non_intersecting_curves(arr,
                                       polywh.outer_boundary().edges_begin(),
                                       polywh.outer_boundary().edges_end());
  Polygon_with_holes_2::Hole_const_iterator hit;
  for (hit = polywh.holes_begin(); hit != polywh.holes_end(); hit++) {
    CGAL::insert_non_intersecting_curves(arr, hit->edges_begin(),
                                         hit->edges_end());
  }
  return arr;
}

Polygon_set_2 CGALUtils::createWorldPolySet_2(const Polygon_2 &world_bounds,
                                              const Polygon_set_2 &obstacles) {
  Polygon_set_2 world;
  world.insert(world_bounds);
  world.difference(obstacles);
  return world;
}

Polygon_2
CGALUtils::createVisibilityPolygonFromPoint(const Polygon_set_2 &world_poly_set,
                                            const Point_2 &p) {
  //* Visibility preprocessing
  // get polygon with holes in which the point p is located
  Polygon_with_holes_2 visAEnv_poly;
  world_poly_set.locate(p, visAEnv_poly);

  // Create Arrangement_2 for visibility calculations as this is the input to
  // visibility algorithm
  Arrangement_2 visAEnv_arr;
  CGALUtils cgalUtils;
  visAEnv_arr = cgalUtils.polygon_with_holes2arrangement_2(visAEnv_poly);

  // add point to visAEnv_arr
  Arrangement_2::Vertex_handle pVertex = CGAL::insert_point(visAEnv_arr, p);

  //* Calculate visibility
  // algorithm class
  TriExpVisibility triExpVisibility(visAEnv_arr);
  // output of visibility calculations
  Arrangement_2 visAOutput;
  Face_handle fhVisAOutput;

  // check if p is isolatd inside face or if on edge
  if (pVertex->is_isolated()) {
    Face_const_handle fhVisAInput;
    fhVisAInput = pVertex->face();

    fhVisAOutput = triExpVisibility.compute_visibility(pVertex->point(),
                                                       fhVisAInput, visAOutput);
  } else {
    Arrangement_2::Halfedge_const_handle he = visAEnv_arr.halfedges_begin();
    while (he->target()->point() != pVertex->point() ||
           (he->face()->is_unbounded()))
      he++;
    // run visibility calculation
    fhVisAOutput =
        triExpVisibility.compute_visibility(pVertex->point(), he, visAOutput);
  }

  //* Visibility postprocessing
  // Create visibility polygon from visibility calculation
  Polygon_2 visibility_polygon;
  Arrangement_2::Ccb_halfedge_const_circulator curr = fhVisAOutput->outer_ccb();
  do {
    visibility_polygon.push_back(curr->target()->point());
    curr++;
  } while (curr != fhVisAOutput->outer_ccb());

  return visibility_polygon;
}

} // namespace polyvision