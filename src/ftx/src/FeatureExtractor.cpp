#include "ftx/FeatureExtractor.h"
#include "gui/gui.h"
#include "CongestParser.h"
#include "GridGraph.h"
#include "Utils.h"
#include "RptParser.h"
#include "odb/db.h"
#include "odb/dbTransform.h"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <unordered_set>

namespace ftx {

FeatureExtractor::FeatureExtractor() :
  db_(nullptr),
  gridGraph_(nullptr)
{
}

FeatureExtractor::~FeatureExtractor()
{
  clear();
}

void
FeatureExtractor::initGraph(odb::dbDatabase* db)
{
  db_ = db;
  gridGraph_ = new ftx::GridGraph();
  auto block = db_->getChip()->getBlock();
  auto row = block->getRows().begin();
  auto site = row->getSite();
  auto height = site->getHeight();
  auto tile_dim = height * 3;
  gridGraph_->initGridUsingDimensions(db_, tile_dim, tile_dim);
}

void
FeatureExtractor::initGraphFromDef(odb::dbDatabase* db)
{
  std::cout<<"FeatureExtractor::initGraphFromDef called."<<std::endl;
  db_ = db;
  gridGraph_ = new ftx::GridGraph();
  gridGraph_->initGridFromDEFGCells(db_);
}

void
FeatureExtractor::readRPT(std::string file_path,
                          odb::dbLib* lib,
                          bool triton)
{
  std::cout<<"FeatureExtractor::readRPT called."<<std::endl;
  ftx::RPTParser parser;
  std::vector<std::pair<odb::Rect, DRVType>> drvs;
  drvs = parser.parseDRVs(file_path, lib);
  for(auto drv : drvs)
  {
    auto nodes = gridGraph_->intersectingNodes(drv.first);
    for(auto node_ptr : nodes)
    {
      auto intersection = node_ptr->rect().intersect(drv.first);
      if(intersection.area() > 0)
      {
        node_ptr->setViolation(true);
        node_ptr->insertDRVType(drv.second);
      }
    }
  }
}

void
FeatureExtractor::readCongestion(std::string file_path)
{
  std::cout<<"FeatureExtractor::readCongestion called."<<std::endl;
  std::ifstream file(file_path);
  return readCongestion(file);
}

void
FeatureExtractor::readCongestion(std::istream & isstream)
{
  ftx::CongestParser parser;
  auto congestions = parser.parseCongestion(isstream);
  for(auto c : congestions)
  {
    auto nodes = gridGraph_->intersectingNodes(c.rect);
    int i = 0;
    ftx::Node * n;
    for(auto node_ptr : nodes)
    {
      auto intersection = node_ptr ->rect().intersect(c.rect);
      if(intersection.area() > 0)
      {
        n = node_ptr;
        i += 1;
      }
    }
    if(i == 1)// There is a misalignment between congest and GCells
    {
      n->insertFeatureCount(FeatureCount::vOverflow, c.v_overflow);
      n->insertFeatureCount(FeatureCount::vRemain, c.v_remain);
      n->insertFeatureCount(FeatureCount::vTracks, c.v_tracks);
      n->insertFeatureCount(FeatureCount::hOverflow, c.h_overflow);
      n->insertFeatureCount(FeatureCount::hRemain, c.h_remain);
      n->insertFeatureCount(FeatureCount::hTracks, c.h_tracks);
    }
  }
}

void
FeatureExtractor::init(odb::dbDatabase* db, GridGraph* graph)
{
  db_ = db;
  gridGraph_ = graph;
}

void
FeatureExtractor::clear()
{
  db_ = nullptr;
  gridGraph_ = nullptr;
}

void
FeatureExtractor::extractFeatures()
{
  std::cout<<"FeatureExtractor::extractFeatures called."<<std::endl;
  auto block = db_->getChip()->getBlock();
  for(auto inst : block->getInsts())
    extractInstFeatures(inst);
  for(auto net : block->getNets())
    extractPassingNets(net);
}

double
FeatureExtractor::calculateDensity()
{
  Utils::AreaDBU total_area = 0.0;
  for(auto node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
  {
    auto node_ptr = gridGraph_->node(node_id);
    auto macro_area = node_ptr->getFeatureArea(FeatureArea::macro);
    auto cell_area = node_ptr->getFeatureArea(FeatureArea::cell);
    total_area += (macro_area+cell_area);
  }
  odb::Rect dieRect;
  db_->getChip()->getBlock()->getDieArea(dieRect);
  auto die_area = dieRect.area();
  return (double)total_area/die_area;
}

void
FeatureExtractor::calculateABU()
{
  odb::Rect rows_bbox;
  db_->getChip()->getBlock()->getCoreArea(rows_bbox);
  std::vector<double> densities;
  for(auto node_ptr : gridGraph_->intersectingNodes(rows_bbox))
  {
    auto node_area = node_ptr->rect().area();
    auto macro_area = node_ptr->getFeatureArea(FeatureArea::macro);
    if(macro_area == node_area)
      continue;

    auto cell_area = node_ptr->getFeatureArea(FeatureArea::cell);
    double density = (macro_area+cell_area)/node_area;
    densities.push_back(density);
  }
  std::sort(densities.begin(), densities.end());

  std::cout << std::setprecision(2);
  std::cout<<"['"<<db_->getChip()->getBlock()->getName()
    <<"', '"<<calculateDensity();

  auto thresholds = {0.01, 0.02, 0.05, 0.1, 0.2, 0.5};
  for(auto threshold : thresholds)
  {
    int i = 0;
    double density = 0;
    int limit = densities.size()*threshold;
    for(auto density_it = densities.rbegin();
        density_it != densities.rend();
        density_it++)
    {
      density += *density_it;
      i++;
      if(i == limit)
        break;
    }
    density /= i;
    //std::cout<<"Threshold: "<<threshold<<" density: "<<density<<std::endl;
    std::cout<<"', '"<<density;
  }
  std::cout<<"']"<<std::endl;
}

void
FeatureExtractor::writeCSV(std::string file_path)
{
  std::cout<<"FeatureExtractor::writeCSV called."<<std::endl;
  std::ofstream ofs(file_path, std::ofstream::out);
  //count placement features
  ofs<<"NodeID,"
     <<"#Cells,"
     <<"#CellPins,"
     <<"#Macros,"
     <<"#MacroPins,"
     <<"#PassingNets,"
  //count global routing features
     <<"#VerticalOverflow,"
     <<"#VerticalRemain,"
     <<"#VerticalTracks,"
     <<"#HorizontalOverflow,"
     <<"#HorizontalRemain,"
     <<"#HorizontalTracks,"
  //area features
     <<"TileArea,"
     <<"CellDensity,"
     <<"MacroDensity,"
     <<"MacroPinDensity,"
     <<"Layer1BlkgDensity,"
     <<"Layer2BlkgDensity,"
     <<"Layer1PinDensity,"
     <<"Layer2PinDensity,"
  //drv types
     <<"AdjacentCutSpacing,"
     <<"SameLayerCutSpacing,"
     <<"EndOfLine,"
     <<"FloatingPatch,"
     <<"MinArea,"
     <<"MinWidth,"
     <<"NonSuficientMetalOverlap,"
     <<"CutShort,"
     <<"MetalShort,"
     <<"OutOfDieShort,"
     <<"CornerSpacing,"
     <<"ParallelRunLength,"
  //neighbor area features
     <<"NeighborTileArea,"
     <<"NeighborCellArea,"
     <<"NeighborL1PinArea,"
     <<"NeighborL2PinArea,"
     <<"NeighborL1BlkArea,"
     <<"NeighborL2BlkArea,"
     <<"NeighborMacroArea,"
     <<"NeighborMacroPinArea,"
  //count placement neighbor features
     <<"#NeighborCells,"
     <<"#NeighborCellPins,"
     <<"#NeighborMacros,"
     <<"#NeighborMacroPins,"
     <<"#NeighborPassingNets,"
  //count global routing neighbor features
     <<"#NeighborVerticalOverflow,"
     <<"#NeighborVerticalRemain,"
     <<"#NeighborVerticalTracks,"
     <<"#NeighborHorizontalOverflow,"
     <<"#NeighborHorizontalRemain,"
     <<"#NeighborHorizontalTracks,"
  //count global routing neighbor features
     <<"HasDetailedRoutingViolation"<<std::endl;

  odb::Rect rows_bbox;
  db_->getChip()->getBlock()->getCoreArea(rows_bbox);
  for(auto node_ptr : gridGraph_->intersectingNodes(rows_bbox))
  {
    ofs<<*node_ptr;
    ofs<<", "<<gridGraph_->neighborhoodFeatures(node_ptr);
    ofs<<", "<<node_ptr->hasViolation()<<std::endl;
  }

  ofs.close();
}

bool
isValid(ftx::Node* node,
        odb::Rect rows_bbox)
{
  if(node == nullptr)
    return false;
  auto node_rect = node->rect();
  return node_rect.intersects(rows_bbox);
}

class DRVRenderer : public gui::Renderer
{
public:
  DRVRenderer(odb::dbDatabase* db,
              ftx::GridGraph * gridGraph) :
    db_(db),
    gridGraph_(gridGraph)
  {
  }

  virtual void drawObjects(gui::Painter& /* painter */) override;

private:
  ftx::GridGraph * gridGraph_;
  odb::dbDatabase* db_;
};

void
DRVRenderer::drawObjects(gui::Painter &painter)
{
  if (gridGraph_ && db_)
  {
    odb::Rect rows_bbox;
    db_->getChip()->getBlock()->getCoreArea(rows_bbox);
    for (auto node_ptr : gridGraph_->intersectingNodes(rows_bbox))
    {
      if (node_ptr->hasViolation())
      {
        odb::Rect node_rect = node_ptr->rect();
        painter.setBrush(gui::Painter::dark_red);
        painter.drawRect(node_rect);
      }
    }
  }
}

void
FeatureExtractor::drawDRVs()
{
  if (drvRenderer_ == nullptr)
  {
    gui::Gui* gui = gui::Gui::get();
    drvRenderer_ = std::make_unique<DRVRenderer>(db_, gridGraph_);
    gui->registerRenderer(drvRenderer_.get());
    gui->redraw();
  }
}

class NodePainter : public gui::Renderer
{
public:
  NodePainter(ftx::GridGraph * gridGraph,
              std::vector<int> nodeIds) :
    gridGraph_(gridGraph),
    nodeIds_(nodeIds)
  {
  }

  virtual void drawObjects(gui::Painter& /* painter */) override;

private:
  const std::vector<int> nodeIds_;
  ftx::GridGraph * gridGraph_;
};

void
NodePainter::drawObjects(gui::Painter &painter)
{
  if (gridGraph_)
  {
    for (auto node_id : nodeIds_)
    {
      auto node_ptr = gridGraph_->node(node_id);
      odb::Rect node_rect = node_ptr->rect();
      painter.setBrush(gui::Painter::dark_blue);
      painter.drawRect(node_rect);
    }
  }
}

void FeatureExtractor::paintNodes(std::string file_path)
{
  std::ifstream infile(file_path);
  std::vector<int> nodeIds;
  int nodeId;
  while (infile >> nodeId)
  {
    nodeIds.emplace_back(nodeId);
  }

  if (nodePainter_ == nullptr)
  {
    gui::Gui* gui = gui::Gui::get();
    nodePainter_ = std::make_unique<NodePainter>(gridGraph_, nodeIds);
    gui->registerRenderer(nodePainter_.get());
    gui->redraw();
  }
}

void
FeatureExtractor::extractInstFeatures(odb::dbInst* inst)
{
  auto box = inst->getBBox();
  odb::Rect bbox_inst;
  box->getBox(bbox_inst);
  auto nodes = gridGraph_->intersectingNodes(bbox_inst);
  odb::dbTransform transform;
  inst->getTransform(transform);
  auto master = inst->getMaster();

  for(auto node : nodes)
  {
    auto node_rect = node->rect();
    if(inst->isCore())
      extractCellFeatures(inst, bbox_inst, master, node, transform);
    else
      extractMacroFeatures(inst, master, node, transform);
  }
}

void
FeatureExtractor::extractCellFeatures(odb::dbInst* inst, odb::Rect bbox_inst,
                                      odb::dbMaster* master, Node* node,
                                      odb::dbTransform transform)
{
  auto node_rect = node->rect();
  odb::Rect intersection;
  node_rect.intersection(bbox_inst, intersection);
  auto area = intersection.area();
  node->insertFeatureArea(FeatureArea::cell, area);
  if(area > 0)
    node->insertFeatureCount(FeatureCount::cell, 1);

  extractCellBlockages(master, node, transform);
  extractCellPins(master, node, transform);
}

void
FeatureExtractor::extractCellBlockages(odb::dbMaster* master, Node* node,
                                       odb::dbTransform transform)
{
  auto node_rect = node->rect();
  auto boxes = master->getObstructions();
  auto l1Rects = Utils::rectsFromRoutingLevel(1, boxes);
  auto l2Rects = Utils::rectsFromRoutingLevel(2, boxes);
  for(auto& rect : l1Rects)
    transform.apply(rect);
  for(auto& rect : l2Rects)
    transform.apply(rect);
  auto areal1 = Utils::nonOverlappingIntersectingArea(node_rect, l1Rects);
  auto areal2 = Utils::nonOverlappingIntersectingArea(node_rect, l2Rects);
  node->insertFeatureArea(FeatureArea::l1Blockage, areal1);
  node->insertFeatureArea(FeatureArea::l2Blockage, areal2);
}

void
FeatureExtractor::extractCellPins(odb::dbMaster* master, Node* node,
                                  odb::dbTransform transform)
{
  auto node_rect = node->rect();
  for(auto term : master->getMTerms())
    for(auto pin : term->getMPins())
    {
      auto boxes = pin->getGeometry();
      auto l1Rects = Utils::rectsFromRoutingLevel(1, boxes);
      auto l2Rects = Utils::rectsFromRoutingLevel(2, boxes);
      for(auto& rect : l1Rects)
        transform.apply(rect);
      for(auto& rect : l2Rects)
        transform.apply(rect);
      auto areal1 = Utils::nonOverlappingIntersectingArea(node_rect,
                                                          l1Rects);
      auto areal2 = Utils::nonOverlappingIntersectingArea(node_rect,
                                                          l2Rects);
      node->insertFeatureArea(FeatureArea::l1Pin, areal1);
      node->insertFeatureArea(FeatureArea::l2Pin, areal2);
      if(areal1 + areal2 > 0)
        node->insertFeatureCount(FeatureCount::pin, 1);
    }
}

void
FeatureExtractor::extractMacroFeatures(odb::dbInst* inst,
                                       odb::dbMaster* master,
                                       Node* node,
                                       odb::dbTransform transform)
{
  auto node_rect = node->rect();
  auto obs = master->getObstructions();
  auto obs_rects = Utils::getTransformedRects(transform, obs);
  auto area = Utils::nonOverlappingIntersectingArea(node_rect, obs_rects);
  node->insertFeatureArea(FeatureArea::macro, area);
  if(area > 0)
    node->insertFeatureCount(FeatureCount::macro, 1);

  extractMacroPins(master, node, transform);
}

void
FeatureExtractor::extractMacroPins(odb::dbMaster* master, Node* node,
                                   odb::dbTransform transform)
{
  auto node_rect = node->rect();
  for(auto term : master->getMTerms())
    for(auto pin : term->getMPins())
    {
      auto boxes = pin->getGeometry();
      auto block_rects = Utils::getTransformedRects(transform, boxes);
      auto area = Utils::nonOverlappingIntersectingArea(node_rect,
                                                        block_rects);
      node->insertFeatureArea(FeatureArea::macroPin, area);
      if(area > 0)
        node->insertFeatureCount(FeatureCount::macroPin, 1);
    }
}

void
FeatureExtractor::extractPassingNets(odb::dbNet* net)
{
  auto net_bbox = Utils::netBoudingBox(net);
  auto nodes = gridGraph_->intersectingNodes(net_bbox);
  for(auto node : nodes)
    node->insertFeatureCount(FeatureCount::passingNets, 1);
}

}
