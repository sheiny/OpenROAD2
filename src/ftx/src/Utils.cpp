#include "Utils.h"
#include "odb/db.h"
#include "odb/dbTransform.h"
#include "odb/geom.h"

#include <algorithm>
#include <limits>
#include <list>

struct Range
{
  Range(int a, int b)
  {
    min = std::min(a, b);
    max = std::max(a, b);
  }

  bool hasOverlap(const Range& other)
  {
    //x1 = min, x2 = max, y1 = other.min, y2 = other.max
    //Assumes x1 <= x2 and y1 <= y2; if this assumption is not safe, the code
    //can be changed to have x1 being min(x1, x2) and x2 being max(x1, x2) and
    //similarly for the ys.
    return max >= other.min && other.max >= min;
  }

  void merge(const Range& other)
  {
    if(hasOverlap(other))
    {
      min = std::min(min, other.min);
      max = std::max(max, other.max);
    }
  }

  int min;
  int max;
};

namespace Utils {

AreaDBU
area(std::vector<odb::Rect> rects)
{
  AreaDBU area = 0;
  for(auto rect : rects)
    area += rect.area();
  return area;
}

std::vector<odb::Rect>
clippingIntersection(odb::Rect rect, std::vector<odb::Rect> rects)
{
  std::vector<odb::Rect> intersecting_rects;
  intersecting_rects.reserve(rects.size());
  for(auto r : rects)
  {
    odb::Rect result;
    rect.intersection(r, result);
    if(result.area() != 0)
      intersecting_rects.push_back(result);
  }
  return intersecting_rects;
}

void
InsertRangeY(std::list<Range>& rangesOfY, Range& rangeY)
{
  std::list<Range>::iterator iter = rangesOfY.begin();
  while(iter != rangesOfY.end())
  {
    if(rangeY.hasOverlap(*iter))
    {
      rangeY.merge(*iter);

      std::list<Range>::iterator iterCopy = iter;
      ++ iter;
      rangesOfY.erase(iterCopy);
    }
    else
      ++ iter;
  }
  rangesOfY.push_back(rangeY);
}

void
GetRangesOfY(const std::vector<odb::Rect>& rects,
             std::vector<odb::Rect>::const_iterator iterRect,
             const Range& rangeX,
             std::list<Range>& rangesOfY)
{
  for(; iterRect != rects.end(); ++ iterRect)
  {
    auto range = Range(iterRect->yMin(), iterRect->yMax());
    if(rangeX.min < iterRect->xMax() && rangeX.max > iterRect->xMin())
      InsertRangeY(rangesOfY, range);
  }
}

std::vector<odb::Rect>
removeOverlaps(std::vector<odb::Rect> rects)
{
  if(rects.empty())
    return {};
  // sort rectangles according to x-value of right edges
  std::sort(rects.begin(), rects.end());
  std::vector<int> xes;
  for(auto rect : rects)
  {
    xes.push_back(rect.xMin());
    xes.push_back(rect.xMax());
  }
  std::sort(xes.begin(), xes.end());
  xes.erase(std::unique(xes.begin(), xes.end()), xes.end());

  std::vector<int>::iterator iterX1 = xes.begin();
  std::vector<odb::Rect>::const_iterator iterRect = rects.begin();
  std::vector<odb::Rect> sliced_rects;
  for(; iterX1 != xes.end() - 1; ++ iterX1)
  {
    std::vector<int>::iterator iterX2 = std::next(iterX1);

    Range rangeX(*iterX1, *iterX2);

    while(iterRect->xMax() < *iterX1)
      ++iterRect;

    std::list<Range> rangesOfY;
    GetRangesOfY(rects, iterRect, rangeX, rangesOfY);
    for(auto iter = rangesOfY.begin(); iter != rangesOfY.end(); ++ iter)
      sliced_rects.push_back({rangeX.min, iter->min, rangeX.max, iter->max});
  }
  return sliced_rects;
}

std::vector<odb::Rect>
getTransformedRects(odb::dbTransform transform, odb::dbSet<odb::dbBox> boxes)
{
  std::vector<odb::Rect> rects;
  rects.reserve(boxes.size());
  for(auto box : boxes)
  {
    odb::Rect rect;
    box->getBox(rect);
    transform.apply(rect);
    rects.push_back(rect);
  }
  return rects;
}

AreaDBU
nonOverlappingIntersectingArea(odb::Rect rect, std::vector<odb::Rect> rects)
{
  auto intersecting_rects = clippingIntersection(rect, rects);
  auto non_overlapping_rects = removeOverlaps(intersecting_rects);
  auto result = area(non_overlapping_rects);
  return result;
}

std::vector<odb::Rect>
rectsFromRoutingLevel(int level, odb::dbSet<odb::dbBox> boxes)
{
  std::vector<odb::Rect> result;
  result.reserve(boxes.size());
  for(auto box : boxes)
    if(level == box->getTechLayer()->getRoutingLevel())
    {
      odb::Rect rect;
      box->getBox(rect);
      result.push_back(rect);
    }
  return result;
}

odb::Rect
netBoudingBox(odb::dbNet* net)
{
  odb::Rect net_bbox;
  net_bbox.set_xlo(std::numeric_limits<int>::max());
  net_bbox.set_ylo(std::numeric_limits<int>::max());
  net_bbox.set_xhi(std::numeric_limits<int>::min());
  net_bbox.set_yhi(std::numeric_limits<int>::min());

  for(auto iterm : net->getITerms())
  {
    int x,y;
    auto has_shape = iterm->getAvgXY(&x, &y);
    if(has_shape == false)
      continue;
    net_bbox.set_xlo(std::min(x, net_bbox.xMin()));
    net_bbox.set_xhi(std::max(x, net_bbox.xMax()));
    net_bbox.set_ylo(std::min(y, net_bbox.yMin()));
    net_bbox.set_yhi(std::max(y, net_bbox.yMax()));
  }
  return net_bbox;
}
}
