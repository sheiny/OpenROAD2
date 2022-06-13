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

Node::~Node(){
}

std::string
Node::debugInfo()
{
  std::string debug_info;
  std::string separator = "\n";
  debug_info+="NodeID: "+std::to_string(nodeID)+separator;
  debug_info+="NumCells: "+std::to_string(numCells)+separator;
  debug_info+="NumPins: "+std::to_string(numCellPins)+separator;
  debug_info+="NumMacros: "+std::to_string(numMacros)+separator;
  debug_info+="NumMacroPins: "+std::to_string(numMacroPins)+separator;
  debug_info+="NumMacroPins: "+std::to_string(numMacroPins)+separator;
  debug_info+="HorizontalCap: "+std::to_string(horizontal_capacity)+separator;
  debug_info+="VerticalCap: "+std::to_string(vertical_capacity)+separator;
  debug_info+="HorizontalDemand: "+std::to_string(horizontal_demand)+separator;
  debug_info+="VerticalDemand: "+std::to_string(vertical_demand)+separator;
  debug_info+="HorizontalOverflow: "+std::to_string((double)horizontal_demand/horizontal_capacity)+separator;
  debug_info+="VerticalOverflow: "+std::to_string((double)vertical_demand/vertical_capacity)+separator;
  return debug_info;
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
Node::printPlacementFeatures()
{
  auto area = rect.area();
  std::string result;
  result += std::to_string(numCells) + ", ";
  result += std::to_string(numCellPins) + ", ";
  result += std::to_string(numMacros) + ", ";
  result += std::to_string(numMacroPins) + ", ";
  result += std::to_string((double)horizontal_demand/horizontal_capacity) + ", ";
  result += std::to_string((double)vertical_demand/vertical_capacity) + ", ";

  result += std::to_string(area) + ", ";
  result += std::to_string((double)cellArea/area) + ", ";
  result += std::to_string((double)macroArea/area) + ", ";
  result += std::to_string((double)macroPinArea/area) + ", ";
  result += std::to_string((double)l1BlockageArea/area) + ", ";
  result += std::to_string((double)l2BlockageArea/area) + ", ";
  result += std::to_string((double)l1PinArea/area) + ", ";
  result += std::to_string((double)l2PinArea/area) + ", ";
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
