#include "ftx/Node.h"

#include <ostream>

namespace ftx {

Node::Node(vertexIndex node_id, odb::Rect rect) :
  nodeID_(node_id),
  rect_(rect)
{
  numCells_ = 0;
  numCellPins_ = 0;
  numMacros_ = 0;
  numMacroPins_ = 0;
  numPassingNets_ = 0;

  vertical_overflow_ = 0;
  vertical_remain_ = 0;
  vertical_tracks_ = 0;
  horizontal_overflow_ = 0;
  horizontal_remain_ = 0;
  horizontal_tracks_ = 0;

  cellArea_ = 0;
  l1BlockageArea_ = 0;
  l2BlockageArea_ = 0;
  l1PinArea_ = 0;
  l2PinArea_ = 0;
  macroArea_ = 0;
  macroPinArea_ = 0;

  violation_ = false;
}

Node::~Node(){
}

void
Node::insertFeatureCount(FeatureCount feature, int amount)
{
  if(feature == FeatureCount::cell)
    numCells_ += amount;
  else if(feature == FeatureCount::pin)
    numCellPins_ += amount;
  else if(feature == FeatureCount::macro)
    numMacros_ += amount;
  else if(feature == FeatureCount::macroPin)
    numMacroPins_ += amount;
  else if(feature == FeatureCount::passingNets)
    numPassingNets_ += amount;
  else if(feature == FeatureCount::vOverflow)
    vertical_overflow_ += amount;
  else if(feature == FeatureCount::vRemain)
    vertical_remain_ += amount;
  else if(feature == FeatureCount::vTracks)
    vertical_tracks_ += amount;
  else if(feature == FeatureCount::hOverflow)
    horizontal_overflow_ += amount;
  else if(feature == FeatureCount::hRemain)
    horizontal_remain_ += amount;
  else if(feature == FeatureCount::hTracks)
    horizontal_tracks_ += amount;
}

void
Node::insertFeatureArea(FeatureArea feature, Utils::AreaDBU area)
{
  if(feature == FeatureArea::cell)
    cellArea_ += area;
  else if(feature == FeatureArea::l1Blockage)
    l1BlockageArea_ += area;
  else if(feature == FeatureArea::l2Blockage)
    l2BlockageArea_ += area;
  else if(feature == FeatureArea::l1Pin)
    l1PinArea_ += area;
  else if(feature == FeatureArea::l2Pin)
    l2PinArea_ += area;
  else if(feature == FeatureArea::macro)
    macroArea_ += area;
  else if(feature == FeatureArea::macroPin)
    macroPinArea_ += area;
}

void
Node::setViolation(bool violation)
{
  violation_ = violation;
}

void
Node::insertDRVType(DRVType drv)
{
  drvs.insert(drv);
}

bool
Node::hasViolation()
{
  return violation_;
}

std::unordered_set<DRVType>
Node::getDRVs()
{
  return drvs;
}

int
Node::getFeatureCount(FeatureCount feature)
{
  if(feature == FeatureCount::cell)
    return numCells_;
  else if(feature == FeatureCount::pin)
    return numCellPins_;
  else if(feature == FeatureCount::macro)
    return numMacros_;
  else if(feature == FeatureCount::macroPin)
    return numMacroPins_;
  else if(feature == FeatureCount::passingNets)
    return numPassingNets_;
  else if(feature == FeatureCount::vOverflow)
    return vertical_overflow_;
  else if(feature == FeatureCount::vRemain)
    return vertical_remain_;
  else if(feature == FeatureCount::vTracks)
    return vertical_tracks_;
  else if(feature == FeatureCount::hOverflow)
    return horizontal_overflow_;
  else if(feature == FeatureCount::hRemain)
    return horizontal_remain_;
  else if(feature == FeatureCount::hTracks)
    return horizontal_tracks_;
  return 0;
}

Utils::AreaDBU
Node::getFeatureArea(FeatureArea feature)
{
  if(feature == FeatureArea::cell)
    return cellArea_;
  else if(feature == FeatureArea::l1Blockage)
    return l1BlockageArea_;
  else if(feature == FeatureArea::l2Blockage)
    return l2BlockageArea_;
  else if(feature == FeatureArea::l1Pin)
    return l1PinArea_;
  else if(feature == FeatureArea::l2Pin)
    return l2PinArea_;
  else if(feature == FeatureArea::macro)
    return macroArea_;
  else if(feature == FeatureArea::macroPin)
    return macroPinArea_;
  return 0;
}

odb::Rect
Node::rect()
{
  return rect_;
}

vertexIndex
Node::id()
{
  return nodeID_;
}

std::ostream&
operator<<(std::ostream& os, Node& node)
{
  auto area = node.rect_.area();
  os<<std::to_string(node.nodeID_)<<", "
    <<std::to_string(node.numCells_)<<", "
    <<std::to_string(node.numCellPins_)<<", "
    <<std::to_string(node.numMacros_)<<", "
    <<std::to_string(node.numMacroPins_)<<", "
    <<std::to_string(node.numPassingNets_)<<", "

    <<std::to_string(node.vertical_overflow_)<<", "
    <<std::to_string(node.vertical_remain_)<<", "
    <<std::to_string(node.vertical_tracks_)<<", "
    <<std::to_string(node.horizontal_overflow_)<<", "
    <<std::to_string(node.horizontal_remain_)<<", "
    <<std::to_string(node.horizontal_tracks_)<<", "

    <<std::to_string(area)<<", "
    <<std::to_string((double)node.cellArea_/area)<<", "
    <<std::to_string((double)node.macroArea_/area)<<", "
    <<std::to_string((double)node.macroPinArea_/area)<<", "
    <<std::to_string((double)node.l1BlockageArea_/area)<<", "
    <<std::to_string((double)node.l2BlockageArea_/area)<<", "
    <<std::to_string((double)node.l1PinArea_/area)<<", "
    <<std::to_string((double)node.l2PinArea_/area)<<", ";
  std::unordered_set<DRVType> drvs = node.getDRVs();
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
