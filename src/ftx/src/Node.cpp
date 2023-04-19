#include "ftx/Node.h"

#include <ostream>

namespace ftx {

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
