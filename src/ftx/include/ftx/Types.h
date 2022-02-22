#pragma once

#include <cstdint>

namespace Utils {
  typedef int DBU;
  typedef std::int64_t AreaDBU;
}

namespace ftx {
typedef int vertexIndex;
enum class DRVType {undefined = 0, adjacentCutSpacing = 1,
  sameLayerCutSpacing = 2, endOfLine = 3, floatingPatch = 4,
  minArea = 5, minWidth = 6, nonSuficientMetalOverlap = 7,
  cutShort = 8, metalShort = 9, outOfDieShort = 10, cornerSpacing = 11,
  parallelRunLength = 12};
}
