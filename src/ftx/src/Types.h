#pragma once

#include <cstdint>

namespace Utils {
  typedef int DBU;
  typedef std::int64_t AreaDBU;
}

namespace ftx {
typedef int vertexIndex;
enum class FeatureArea {cell = 0, l1Blockage = 1, l2Blockage = 2,
  l1Pin = 3, l2Pin= 4, macro = 5, macroPin = 6};
enum class FeatureCount {cell = 0, pin = 1, macro = 2, macroPin = 3,
  passingNets = 4, vOverflow = 5, vRemain = 6, vTracks = 7,
  hOverflow = 8, hRemain = 9, hTracks = 10};
enum class DRVType {undefined = 0, adjacentCutSpacing = 1,
  sameLayerCutSpacing = 2, endOfLine = 3, floatingPatch = 4,
  minArea = 5, minWidth = 6, nonSuficientMetalOverlap = 7,
  cutShort = 8, metalShort = 9, outOfDieShort = 10, cornerSpacing = 11,
  parallelRunLength = 12};
}
