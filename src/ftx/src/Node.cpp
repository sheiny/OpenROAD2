#include "ftx/Node.h"

#include <ostream>

namespace ftx {

Node::Node(vertexIndex node_id, odb::Rect rect) :
  nodeID(node_id),
  rect(rect)
{
  numCells = 0;
  numCellPins = 0;
  numMacros = 0;
  numMacroPins = 0;

  //GR features
  horizontal_capacity = 0;
  vertical_capacity = 0;
  horizontal_demand = 0;
  vertical_demand = 0;

  //CNN features
  vertical_overflow = 0;
  vertical_remain = 0;
  vertical_tracks = 0;
  horizontal_overflow = 0;
  horizontal_remain = 0;
  horizontal_tracks = 0;

  cellArea = 0;
  l1BlockageArea = 0;
  l2BlockageArea = 0;
  l1PinArea = 0;
  l2PinArea = 0;
  macroArea = 0;
  macroPinArea = 0;

  violation = false;
}

std::string
Node::printPlacementFeatures(std::string separator, bool debug)
{
  auto area = rect.area();
  std::string result;
  result += (debug?"numCells: ":"") + std::to_string(numCells) + separator;
  result += (debug?"numCellPins: ":"") + std::to_string(numCellPins) + separator;
  result += (debug?"numMacros: ":"") + std::to_string(numMacros) + separator;
  result += (debug?"numMacroPins: ":"") + std::to_string(numMacroPins) + separator;
  double horizontalOverflow = (horizontal_capacity == 0? 0 : (double)horizontal_demand/horizontal_capacity);
  result += (debug?"HorizontalOverflow: ":"") + std::to_string(horizontalOverflow) + separator;
  double verticalOverflow = (vertical_capacity == 0? 0 : (double)vertical_demand/vertical_capacity);
  result += (debug?"VerticalOverflow: ":"") + std::to_string(verticalOverflow) + separator;

  result += (debug?"TileArea: ":"") + std::to_string(area) + separator;
  result += (debug?"CellDensity: ":"") + std::to_string((double)cellArea/area) + separator;
  result += (debug?"MacroDensity: ":"") + std::to_string((double)macroArea/area) + separator;
  result += (debug?"MacroPinDensity: ":"") + std::to_string((double)macroPinArea/area) + separator;
  result += (debug?"L1BlkgDensity: ":"") + std::to_string((double)l1BlockageArea/area) + separator;
  result += (debug?"L2BlkgDensity: ":"") + std::to_string((double)l2BlockageArea/area) + separator;
  result += (debug?"L1PinDensity: ":"") + std::to_string((double)l1PinArea/area) + separator;
  result += (debug?"L2PinDensity: ":"") + std::to_string((double)l2PinArea/area) + separator;
  return result;
}

std::string
Node::printCongestion()
{
  std::string result;
  result += std::to_string(vertical_overflow)+" ";
  result += std::to_string(vertical_remain)+" ";
  result += std::to_string(vertical_tracks)+" ";
  result += std::to_string(horizontal_overflow)+" ";
  result += std::to_string(horizontal_remain)+" ";
  result += std::to_string(horizontal_tracks)+" ";
  return result;
}

std::string
Node::printDRVs()
{
  std::string result;
  std::vector<DRVType> drvTypes{DRVType::adjacentCutSpacing,
    DRVType::sameLayerCutSpacing, DRVType::endOfLine, DRVType::floatingPatch,
    DRVType::minArea, DRVType::minWidth, DRVType::nonSuficientMetalOverlap,
    DRVType::cutShort, DRVType::metalShort, DRVType::outOfDieShort,
    DRVType::cornerSpacing, DRVType::parallelRunLength};
  for(DRVType drv : drvTypes)
  {
    bool found = (drvs.find(drv) != drvs.end()) ? true : false;
    result += std::to_string(found) + ", ";
  }
  result += std::to_string(violation) + "\n";
  return result;
}
}
