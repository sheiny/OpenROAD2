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
  numPassingNets = 0;

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
  debug_info+="NumPassingNets: "+std::to_string(numPassingNets)+separator;
  return debug_info;
}

std::ostream&
operator<<(std::ostream& os, Node& node)
{
  auto area = node.rect.area();
  os<<std::to_string(node.nodeID)<<", "
    <<std::to_string(node.numCells)<<", "
    <<std::to_string(node.numCellPins)<<", "
    <<std::to_string(node.numMacros)<<", "
    <<std::to_string(node.numMacroPins)<<", "
    <<std::to_string(node.numPassingNets)<<", "

    <<std::to_string(node.vertical_overflow)<<", "
    <<std::to_string(node.vertical_remain)<<", "
    <<std::to_string(node.vertical_tracks)<<", "
    <<std::to_string(node.horizontal_overflow)<<", "
    <<std::to_string(node.horizontal_remain)<<", "
    <<std::to_string(node.horizontal_tracks)<<", "

    <<std::to_string(area)<<", "
    <<std::to_string((double)node.cellArea/area)<<", "
    <<std::to_string((double)node.macroArea/area)<<", "
    <<std::to_string((double)node.macroPinArea/area)<<", "
    <<std::to_string((double)node.l1BlockageArea/area)<<", "
    <<std::to_string((double)node.l2BlockageArea/area)<<", "
    <<std::to_string((double)node.l1PinArea/area)<<", "
    <<std::to_string((double)node.l2PinArea/area)<<", ";
  std::unordered_set<DRVType> drvs = node.drvs;
  std::vector<DRVType> drvTypes{DRVType::adjacentCutSpacing,
    DRVType::sameLayerCutSpacing, DRVType::endOfLine, DRVType::floatingPatch,
    DRVType::minArea, DRVType::minWidth, DRVType::nonSuficientMetalOverlap,
    DRVType::cutShort, DRVType::metalShort, DRVType::outOfDieShort,
    DRVType::cornerSpacing};
  for(DRVType drv : drvTypes)
  {
    bool found = (drvs.find(drv) != drvs.end()) ? true : false;
    os<<found<<", ";
  }
  os<<(drvs.find(DRVType::parallelRunLength) != drvs.end()) ? true : false;

  return os;
}
}
