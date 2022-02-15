#pragma once

#include "odb/geom.h"
#include "Types.h"

#include <vector>

namespace odb {
  class Rect;
  class dbBox;
  class dbNet;
  template <class T> class dbSet;
  class dbTransform;
}

namespace Utils {

//This function returns the summation of the areas of the rectangles
//Note: this function disregards overlaps.
AreaDBU area(std::vector<odb::Rect> rects);

//Given an odb::Rect and a vector of rectangles this function return
// a vector of intersecting rectangles.
//Note: this function does not return zero area intersections.
std::vector<odb::Rect> clippingIntersection(odb::Rect rect,
                                            std::vector<odb::Rect> rects);

//Return a vector of sliced odb::Rect which does not contain overlaps.
//Notes: the rectangles might be disjoint.
//
//Example:
//Imput: two rectangles A and B
// ------------------
// |                |
// |   A    |-------------|
// |        |      B      |
// ---------|             |
//          |-------------|
//
//Output: three rectangles A, B and C.
// ---------|-------|
// |        |       |
// |   A    |   B   |-----|
// |        |       |  C  |
// ---------|       |     |
//          |-------|-----|
std::vector<odb::Rect> removeOverlaps(std::vector<odb::Rect> rects);

//Applies the transform over all dbBox.Rect elements.
//For example:
//   ^                               ^
// 4 I                             4 I  |--|
//   I                               I  |  |
// 3 |--|             Offset       3 I  |  |
//   |  |          Transform(1,1)    I  |  |
// 2 |  |               __         2 I  |  |-----|
//   |  |                /|          I  |  |     |
// 1 |  |-----|         /          1 I  |--|-----|
//   |  |     |        /             I
// 0 |--|-----|--->                0 ------------->
//   0  1  2  3  4                   0  1  2  3  4
std::vector<odb::Rect> getTransformedRects(odb::dbTransform transform,
                                           odb::dbSet<odb::dbBox> boxes);

//Given an odb::Rect and a set of possibly overlapping and/or
// disjoint rectangles, this function returns the non-overlapping
// area of the intersection of this vector of rectangles with the
// given odb::Rect.
//
//For example:
// Non-Overlapping Intersecting Area (XX) = 3
//
//   ^
// 4 |        I==I
//   |        I  I
// 3 |  |-----I  I--|
//   |  |     IXXI  |
// 2 |  |     I========I
//   |  |     IXX XX   I
// 1 |  |-----I========I
//   |
// 0 |------------------>
//   0  1  2  3  4  5  6
AreaDBU nonOverlappingIntersectingArea(odb::Rect rect,
                                       std::vector<odb::Rect> rects);

//This function receive a odb::dbSet<odb::dbBox> and return
// the rectangles from a given routing level.
std::vector<odb::Rect> rectsFromRoutingLevel(int level,
                                             odb::dbSet<odb::dbBox> boxes);

//Given an odb::dbNet this function returns an odb::Rect bouding box
// of the net.
//
//For example:
//   ^
// 5 |  _
// 4 | |2|
// 3 |      _   -\
// 2 |     |3|  -/  odb::Rect(0,0,3,3)
// 1 |_
// 0 |1|
//   |-------->
//   0 1 2 3 4
odb::Rect netBoudingBox(odb::dbNet* net);
}
