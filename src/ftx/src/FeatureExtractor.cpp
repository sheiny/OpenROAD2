#include "ftx/FeatureExtractor.h"
#include "gui/gui.h"
#include "ftx/CongestParser.h"
#include "ftx/GridGraph.h"
#include "ftx/Utils.h"
#include "ftx/RptParser.h"
#include "ord/OpenRoad.hh"
#include "odb/db.h"
#include "odb/dbTransform.h"
#include "stt/SteinerTreeBuilder.h"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <unordered_set>


struct TrackGrid
{
  int step, offset;
  bool horizontal;

  void init(int _step, int _offset, bool _horizontal)
  {
    step = _step;
    offset = _offset;
    horizontal = _horizontal;
  }
};

struct Segment
{
  Segment(int xs, int xe, int ys, int ye):
    x1(xs),
    x2(xe),
    y1(ys),
    y2(ye)
  {};
  int x1, x2, y1, y2;
};

std::vector<Segment>
extractSegments(const stt::Tree &tree, bool horizontal)
{
  std::vector<Segment> result;
  for(int i = 0; i < tree.branchCount(); ++i)
  {
    const stt::Branch& branch = tree.branch[i];
    if(i == branch.n)
      continue;
    const int x1 = branch.x;
    const int y1 = branch.y;
    const stt::Branch& neighbor = tree.branch[branch.n];
    const int x2 = neighbor.x;
    const int y2 = neighbor.y;

    if(horizontal)
    {
      result.push_back({x1, x2, y1, y1});
      result.push_back({x1, x2, y2, y2});
    }
    else
    {
      result.push_back({x1, x1, y1, y2});
      result.push_back({x2, x2, y1, y2});
    }
  }
  return result;
}

namespace ftx {

FeatureExtractor::FeatureExtractor()
{
  db_ = ord::OpenRoad::openRoad()->getDb();
  logger_ = ord::OpenRoad::openRoad()->getLogger();
  stt_ = ord::OpenRoad::openRoad()->getSteinerTreeBuilder();
}

FeatureExtractor::~FeatureExtractor()
{
  //clear();
}

void
FeatureExtractor::saveLocations(std::string file_path)
{
  std::ofstream ofs(file_path, std::ofstream::out);
  auto block = db_->getChip()->getBlock();
  for(auto inst : block->getInsts())
  {
    std::string name = inst->getName();
    int xloc, yloc;
    inst->getLocation(xloc, yloc);
    ofs<<name<<" "<<xloc<<" "<<yloc<<std::endl;
  }
  ofs.close();
}

void
FeatureExtractor::loadLocations(std::string file_path)
{
  std::ifstream file(file_path);
  std::string line;
  std::string delimiter = " ";
  std::unordered_map<std::string, std::pair<int,int>> locations;
  while(std::getline(file, line))
  {
    size_t pos = 0;
    pos = line.find(delimiter);
    std::string instName = line.substr(0, pos);
    line.erase(0, pos + delimiter.length());
    pos = line.find(delimiter);
    int xloc = std::stoi(line.substr(0, pos));
    line.erase(0, pos + delimiter.length());
    int yloc = std::stoi(line);

    locations[instName] = std::make_pair(xloc, yloc);
  }
  //get row height
  //calculate average displacement
  auto block = db_->getChip()->getBlock();
  auto row = block->getRows().begin();
  auto site = row->getSite();
  int height = site->getHeight();
  long totalXdispl = 0;
  long totalYdispl = 0;
  int numInstFound = 0;
  for(auto inst : block->getInsts())
  {
    std::string name = inst->getName();
    if(locations.find(name) == locations.end())
      continue;
    else
    {
      int xloc, yloc;
      inst->getLocation(xloc, yloc);
      ++numInstFound;
      std::pair<int, int> savedLocation = locations[name];
      totalXdispl += std::abs(xloc - savedLocation.first);
      totalYdispl += std::abs(yloc - savedLocation.second);
    }
  }
  double avgDipl = ((double)totalXdispl + (double)totalYdispl)/numInstFound;
  std::cout<<"The total displacement in placement units is: "<<avgDipl<<" for: "<<numInstFound<<" instances."<<std::endl;
}

void
FeatureExtractor::initGraph(int sizeInRowHeights)
{
  std::cout<<"FeatureExtractor::initGraph called."<<std::endl;
  gridGraph_ = new ftx::GridGraph();
  auto block = db_->getChip()->getBlock();
  auto row = block->getRows().begin();
  auto site = row->getSite();
  auto height = site->getHeight();
  auto tile_dim = height * sizeInRowHeights;
  gridGraph_->initGridUsingDimensions(db_, tile_dim, tile_dim);
}

void
FeatureExtractor::initGraphFromDef()
{
  std::cout<<"FeatureExtractor::initGraphFromDef called."<<std::endl;
  gridGraph_ = new ftx::GridGraph();
  gridGraph_->initGridFromDEFGCells(db_);
}

void
FeatureExtractor::readRPT(std::string file_path,
                          odb::dbLib* lib)
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
      auto intersection = node_ptr->rect.intersect(drv.first);
      if(intersection.area() > 0)
      {
        node_ptr->violation = true;
        node_ptr->drvs.insert(drv.second);
      }
    }
  }

  int num_violating_nodes = 0;
  for(auto node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
  {
    auto node_ptr = gridGraph_->node(node_id);
    if (node_ptr->violation)
    {
      ++num_violating_nodes;
    }
  }
  std::cout<<"Nodes with DRVs: "<<num_violating_nodes<<" of "<<gridGraph_->sizeNodes()<<", on Metal1/Metal2/Metal3 Layers."<<std::endl;
}

void
FeatureExtractor::readCongestion(std::string file_path)
{
  std::cout<<"FeatureExtractor::readCongestion called."<<std::endl;
  std::ifstream file(file_path);
  readCongestion(file);
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
      auto intersection = node_ptr->rect.intersect(c.rect);
      if(intersection.area() > 0)
      {
        n = node_ptr;
        i += 1;
      }
    }
    if(i == 1)// There is a misalignment between congest and GCells
    {
      n->vertical_overflow += c.v_overflow;
      n->vertical_remain += c.v_remain;
      n->vertical_tracks += c.v_tracks;
      n->horizontal_overflow += c.h_overflow;
      n->horizontal_remain += c.h_remain;
      n->horizontal_tracks += c.h_tracks;
    }
  }
}

void
FeatureExtractor::initGraphFromCongestion(std::string file_path)
{
  std::cout<<"FeatureExtractor::initGraphFromCongestion called."<<std::endl;
  gridGraph_ = new ftx::GridGraph();
  std::vector<Utils::DBU> xTicks, yTicks;
  ftx::CongestParser parser;
  // Create grid
  auto congestions = parser.parseCongestion(file_path);
  for(auto congestion : congestions)
  {
    odb::Rect rect = congestion.rect;
    xTicks.push_back(rect.xMin());
    xTicks.push_back(rect.xMax());
    yTicks.push_back(rect.yMin());
    yTicks.push_back(rect.yMax());
  }
  gridGraph_->initGridFromCoords(xTicks, yTicks);

  // Populate grid values
  for(auto congestion : congestions)
  {
    odb::Rect rect = congestion.rect;
    int xMid = (rect.xMin()+rect.xMax())/2;
    int yMid = (rect.yMin()+rect.yMax())/2;
    odb::Point center = odb::Point(xMid, yMid);
    ftx::Node* node = gridGraph_->intersectingNode(center);
    if(node == nullptr)
      throw std::out_of_range("There is a gcell without any grid node overlap!!!");
    node->vertical_overflow = congestion.v_overflow;
    node->vertical_remain = congestion.v_remain;
    node->vertical_tracks = congestion.v_tracks;
    node->horizontal_overflow = congestion.h_overflow;
    node->horizontal_remain = congestion.h_remain;
    node->horizontal_tracks = congestion.h_tracks;
  }
}

void
FeatureExtractor::clear()
{
  db_ = nullptr;
  stt_ = nullptr;
  if (gridGraph_ != nullptr)
    delete gridGraph_;
}

void
FeatureExtractor::extractFeatures()
{
  std::cout<<"FeatureExtractor::extractFeatures called."<<std::endl;
  auto block = db_->getChip()->getBlock();
  // Placement features
  for(auto inst : block->getInsts())
    extractInstFeatures(inst);

  // Routing features
  block->setDrivingItermsforNets();
  initRoutingCapacity();
  for(auto net : block->getNets())
  {
    extractRoutingFeatures(net);
  }
}

void
FeatureExtractor::writeCNNInputFile(std::string path,
                                    std::string circuitName,
                                    std::string fileName,
                                    auto container,
                                    int neighborhoodSize)
{
  std::ofstream outViolating(path+"/temp.txt");
  int validNeighborNodes = 0;
  for(auto node : container)
  {
    std::string neighborhood;
    bool validNeighborhood = gridGraph_->neighborhoodCongestion(node, neighborhoodSize, neighborhood);
    if(validNeighborhood)
    {
      outViolating<<neighborhood<<std::endl;//write file
      ++validNeighborNodes;
    }
  }
  outViolating.close();
  // it is very important to include in the name of the file the matrix
  // shape it should be X_23_23_6
  std::rename(std::string(path+"/temp.txt").c_str(),
              std::string(path+"/"+circuitName+"_"+fileName+"_"+
              std::to_string(validNeighborNodes)+"_"+
              std::to_string(neighborhoodSize*2+1)+"_"+
              std::to_string(neighborhoodSize*2+1)+"_6.txt").c_str());
}

void
FeatureExtractor::extractCNNFeatures(std::string outputPath,
                                     std::string circuitName,
                                     int neighborhoodSize)
{
  std::vector<ftx::Node*> violatingNodes;
  std::unordered_set<ftx::Node*> surroundingViolNodes;
  std::vector<ftx::Node*> nonViolatingNodes;
  //Get violating nodes and their surrounded by violating neighbors.
  for(auto node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
  {
    ftx::Node *node_ptr = gridGraph_->node(node_id);
    if(node_ptr->violation)
    {
      std::vector<ftx::Node*> neighborhood = gridGraph_->neighborhood(node_ptr, 1);
      if(neighborhood.empty())//invalid neighborhood padding is required
        continue;
      violatingNodes.push_back(node_ptr);
      for(ftx::Node* neighbor : neighborhood)
      {
        if(neighbor->violation == false)
          surroundingViolNodes.insert(neighbor);
      }
    }
  }
  //Get the free violation nodes (excluding the surrounding ones)
  for(auto node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
  {
    ftx::Node *node_ptr = gridGraph_->node(node_id);
    if(node_ptr->violation == false &&
       surroundingViolNodes.find(node_ptr) == surroundingViolNodes.end())
    {
      nonViolatingNodes.push_back(node_ptr);
    }
  }
  writeCNNInputFile(outputPath, circuitName, "violatingNodes", violatingNodes, neighborhoodSize);
  writeCNNInputFile(outputPath, circuitName, "surroundingViolNodes", surroundingViolNodes, neighborhoodSize);
  writeCNNInputFile(outputPath, circuitName, "nonViolatingNodes", nonViolatingNodes, neighborhoodSize);
}

double
FeatureExtractor::calculateDensity()
{
  Utils::AreaDBU total_area = 0.0;
  for(auto node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
  {
    ftx::Node *node_ptr = gridGraph_->node(node_id);
    Utils::AreaDBU macro_area = node_ptr->macroArea;
    Utils::AreaDBU cell_area = node_ptr->cellArea;
    total_area += (macro_area+cell_area);
  }
  odb::Rect dieRect = db_->getChip()->getBlock()->getDieArea();
  auto die_area = dieRect.area();
  return (double)total_area/die_area;
}

void
FeatureExtractor::calculateABU()
{
  std::vector<double> densities;
  for(auto node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
  {
    ftx::Node *node_ptr = gridGraph_->node(node_id);
    auto node_area = node_ptr->rect.area();
    Utils::AreaDBU macro_area = node_ptr->macroArea;
    if(macro_area == node_area)
      continue;

    Utils::AreaDBU cell_area = node_ptr->cellArea;
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
FeatureExtractor::writeCSV(std::string file_path, int distance)
{
  int neighborhoodSize = (distance*2+1)*(distance*2+1);
  std::cout<<"FeatureExtractor::writeCSV called."<<std::endl;
  std::ofstream ofs(file_path, std::ofstream::out);
  ofs<<"NodeID,";

  //window features
  std::vector<std::string> placementFeatures = {"#Cells,", "#CellPins,", "#Macros,",
    "#MacroPins,", "HorizontalOverflow,", "VerticalOverflow,", "TileArea,",
    "CellDensity,", "MacroDensity,", "MacroPinDensity,", "Layer1BlkgDensity,",
    "Layer2BlkgDensity,", "Layer1PinDensity,", "Layer2PinDensity,"};
  for(int i = 0; i < neighborhoodSize; i++)
  {
    for(std::string feature : placementFeatures)
    {
      ofs<<std::to_string(i)+"_"+feature;
    }
  }
  //drv types
  ofs<<"AdjacentCutSpacing,"
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
  //count global routing neighbor features
     <<"HasDetailedRoutingViolation"<<std::endl;
  for(auto node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
  {
    ftx::Node *node_ptr = gridGraph_->node(node_id);
    std::vector<Node*> neighborhood = gridGraph_->neighborhood(node_ptr, distance);
    if(neighborhood.empty())//invalide neighborhood padding is required
      continue;
    //print node ID
    ofs<<std::to_string(node_ptr->nodeID)<<", ";
    for(Node* node : neighborhood)//this follows the major order
      ofs<<node->printPlacementFeatures();
    //print node info
    //print neighborhood folowing row major order print node infos
    //print node violation info
    ofs<<node_ptr->printDRVs();
  }

  ofs.close();
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
    for(auto node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
    {
      auto node_ptr = gridGraph_->node(node_id);
      if (node_ptr->violation)
      {
        odb::Rect node_rect = node_ptr->rect;
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

class GridRender : public gui::Renderer
{
public:
  GridRender(odb::dbDatabase* db,
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
GridRender::drawObjects(gui::Painter &painter)
{
  if (gridGraph_ && db_)
  {
    for(auto node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
    {
      ftx::Node *node_ptr = gridGraph_->node(node_id);
      odb::Rect node_rect = node_ptr->rect;
      painter.setBrush(gui::Painter::transparent);
      painter.setPen(gui::Painter::yellow, false, 50);
      painter.drawRect(node_rect);
    }
  }
}

void
FeatureExtractor::drawGrid()
{
  if (drvRenderer_ == nullptr)
  {
    gui::Gui* gui = gui::Gui::get();
    gridRenderer_ = std::make_unique<GridRender>(db_, gridGraph_);
    gui->registerRenderer(gridRenderer_.get());
    gui->redraw();
  }
}

void
FeatureExtractor::initRoutingCapacity()
{
  odb::dbBlock *block = db_->getChip()->getBlock();
  odb::dbSet<odb::dbTrackGrid> grids = block->getTrackGrids();
  std::vector<TrackGrid> layerToGrid;
  for(auto trackGrid : grids)
  {
    odb::dbTechLayer *techLayer = trackGrid->getTechLayer();
    odb::dbTechLayerDir layerDirection = techLayer->getDirection();
    if(layerDirection.getValue() == odb::dbTechLayerDir::Value::NONE)
      continue;

    if(trackGrid->getNumGridPatternsX() != 1 || trackGrid->getNumGridPatternsY() != 1)
      throw std::out_of_range("There are more than one grid track pattern for a same routing layer.");

    std::vector<int> grid;
    bool horizontal;
    if(layerDirection.getValue() == odb::dbTechLayerDir::Value::HORIZONTAL)
    {
      trackGrid->getGridY(grid);
      horizontal = true;
    }
    else if(layerDirection.getValue() == odb::dbTechLayerDir::Value::VERTICAL)
    {
      trackGrid->getGridX(grid);
      horizontal = false;
    }
    const int step = *std::next(grid.begin()) - grid.front();
    const int offset = grid.front();
    TrackGrid t_grid;
    t_grid.init(step, offset, horizontal);
    layerToGrid.push_back(t_grid);
  }
  for(int node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
  {
    int h_capacity = 0, v_capacity = 0;
    ftx::Node *node = gridGraph_->node(node_id);
    odb::Rect rect =  node->rect;
    for(auto grid : layerToGrid)
    {
      int edge_length = 0, edge_origin = 0;
      if(grid.horizontal)
      {
        edge_origin = rect.yMin();
        edge_length = rect.dy();
      }
      else
      {
        edge_origin = rect.xMin();
        edge_length = rect.dx();
      }
      const bool aligned = (edge_origin-grid.offset)%grid.step == 0;
      const int capacity = (edge_length/grid.step) + aligned;

      if(grid.horizontal)
        h_capacity += capacity;
      else
        v_capacity += capacity;
    }
    node->horizontal_capacity = h_capacity;
    node->vertical_capacity = v_capacity;
  }
}

class NodePainter : public gui::Renderer
{
public:
  NodePainter(ftx::GridGraph * gridGraph,
              std::unordered_set<int> nodeIds) :
    gridGraph_(gridGraph),
    nodeIds_(nodeIds)
  {
  }

  void addNode(int id)
  {
    nodeIds_.insert(id);
  };

  virtual void drawObjects(gui::Painter& /* painter */) override;

private:
  std::unordered_set<int> nodeIds_;
  ftx::GridGraph * gridGraph_;
};

void
NodePainter::drawObjects(gui::Painter &painter)
{
  if (gridGraph_)
  {
    for (auto node_id : nodeIds_)
    {
      ftx::Node *node_ptr = gridGraph_->node(node_id);
      odb::Rect node_rect = node_ptr->rect;
      painter.setBrush(gui::Painter::dark_blue);
      painter.drawRect(node_rect);
    }
  }
}

void FeatureExtractor::paintNodes(std::string file_path)
{
  std::ifstream infile(file_path);
  std::unordered_set<int> nodeIds;
  int nodeId;
  while (infile >> nodeId)
  {
    nodeIds.insert(nodeId);
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
FeatureExtractor::paintNode(unsigned int id)
{
  gui::Gui* gui = gui::Gui::get();
  if (nodePainter_ == nullptr)
  {
    std::unordered_set<int> nodeIds;
    nodePainter_ = std::make_unique<NodePainter>(gridGraph_, nodeIds);
    gui->registerRenderer(nodePainter_.get());
  }
  nodePainter_->addNode(id);
  gui->redraw();
}

void
FeatureExtractor::printNodeDebugInfo(unsigned int id)
{
  ftx::Node *node_ptr = gridGraph_->node(id);
  std::cout<<"PrintNodeDebugInfo for node id: "<<id<<std::endl
           <<node_ptr->debugInfo()<<std::endl;
}

void
FeatureExtractor::extractInstFeatures(odb::dbInst* inst)
{
  odb::dbBox *box = inst->getBBox();
  odb::Rect bbox_inst = box->getBox();
  std::vector<ftx::Node*> nodes = gridGraph_->intersectingNodes(bbox_inst);
  odb::dbTransform transform;
  inst->getTransform(transform);
  odb::dbMaster *master = inst->getMaster();

  for(auto node : nodes)
  {
    odb::Rect node_rect = node->rect;
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
  odb::Rect node_rect = node->rect;
  odb::Rect intersection;
  node_rect.intersection(bbox_inst, intersection);
  auto area = intersection.area();
  node->cellArea += area;
  if(area > 0)
  {
    node->numCells += 1;

    extractCellBlockages(master, node, transform);
    extractCellPins(master, node, transform);
  }
}

void
FeatureExtractor::extractCellBlockages(odb::dbMaster* master, Node* node,
                                       odb::dbTransform transform)
{
  odb::Rect node_rect = node->rect;
  odb::dbSet<odb::dbBox> boxes = master->getObstructions();
  std::vector<odb::Rect> l1Rects = Utils::rectsFromRoutingLevel(1, boxes);
  std::vector<odb::Rect> l2Rects = Utils::rectsFromRoutingLevel(2, boxes);
  for(auto& rect : l1Rects)
    transform.apply(rect);
  for(auto& rect : l2Rects)
    transform.apply(rect);
  Utils::AreaDBU areal1 = Utils::nonOverlappingIntersectingArea(node_rect, l1Rects);
  Utils::AreaDBU areal2 = Utils::nonOverlappingIntersectingArea(node_rect, l2Rects);
  node->l1BlockageArea += areal1;
  node->l2BlockageArea += areal2;
}

void
FeatureExtractor::extractCellPins(odb::dbMaster* master, Node* node,
                                  odb::dbTransform transform)
{
  odb::Rect node_rect = node->rect;
  for(auto term : master->getMTerms())
  {
    if(term->getName() == "VSS" || term->getName() == "VDD")
      continue;
    for(auto pin : term->getMPins())
    {
      odb::dbSet<odb::dbBox> boxes = pin->getGeometry();
      std::vector<odb::Rect> l1Rects = Utils::rectsFromRoutingLevel(1, boxes);
      std::vector<odb::Rect> l2Rects = Utils::rectsFromRoutingLevel(2, boxes);
      for(auto& rect : l1Rects)
        transform.apply(rect);
      for(auto& rect : l2Rects)
        transform.apply(rect);
      Utils::AreaDBU areal1 = Utils::nonOverlappingIntersectingArea(node_rect, l1Rects);
      Utils::AreaDBU areal2 = Utils::nonOverlappingIntersectingArea(node_rect, l2Rects);
      node->l1PinArea += areal1;
      node->l2PinArea += areal2;
      if(areal1 + areal2 > 0)
        node->numCellPins += 1;
    }
  }
}

void
FeatureExtractor::extractMacroFeatures(odb::dbInst* inst,
                                       odb::dbMaster* master,
                                       Node* node,
                                       odb::dbTransform transform)
{
  odb::Rect node_rect = node->rect;
  odb::dbSet<odb::dbBox> obs = master->getObstructions();
  std::vector<odb::Rect> obs_rects = Utils::getTransformedRects(transform, obs);
  Utils::AreaDBU area = Utils::nonOverlappingIntersectingArea(node_rect, obs_rects);
  node->macroArea += area;
  if(area > 0)
  {
    node->numMacros += 1;
    extractMacroPins(master, node, transform);
  }
}

void
FeatureExtractor::extractMacroPins(odb::dbMaster* master, Node* node,
                                   odb::dbTransform transform)
{
  odb::Rect node_rect = node->rect;
  for(auto term : master->getMTerms())
  {
    if(term->getName() == "VSS" || term->getName() == "VDD")
      continue;
    for(auto pin : term->getMPins())
    {
      odb::dbSet<odb::dbBox> boxes = pin->getGeometry();
      std::vector<odb::Rect> block_rects = Utils::getTransformedRects(transform, boxes);
      Utils::AreaDBU area = Utils::nonOverlappingIntersectingArea(node_rect, block_rects);
      node->macroPinArea += area;
      if(area > 0)
        node->numMacroPins += 1;
    }
  }
}

void
FeatureExtractor::extractRoutingFeatures(odb::dbNet* net)
{
  //skip PG Nets (double check for clock?)
  if ((net->getSigType() == odb::dbSigType::GROUND)
      || (net->getSigType() == odb::dbSigType::POWER))
    return;

  const int driverID = net->getDrivingITerm();
  if(driverID == 0 || driverID == -1)
    return; //throw std::logic_error("Error, net without a driver (should we skip it?).");

  // Get pin coords and driver
  std::vector<int> xcoords, ycoords;
  int rootIndex;
  for(auto dbITerm : net->getITerms())
  {
    int x, y;
    const bool pinExist = dbITerm->getAvgXY(&x, &y);
    if(pinExist)
    {
      if(driverID == dbITerm->getId())
      {
        rootIndex = xcoords.size();
      }
      xcoords.push_back(x);
      ycoords.push_back(y);
    }
  }
  // Build Steiner Tree
  const stt::Tree tree = stt_->makeSteinerTree(xcoords, ycoords, rootIndex);

  // Get routing segments
  const std::vector<Segment> horizontalSegments = extractSegments(tree, true);
  const std::vector<Segment> verticalSegments = extractSegments(tree, false);

  // Query intersecting nodes
  std::unordered_set<Node*> horizontalNodes, verticalNodes;
  for(auto seg : horizontalSegments)
  {
    const std::vector<Node*> nodes = gridGraph_->intersectingNodes({seg.x1, seg.y1, seg.x2, seg.y2});
    horizontalNodes.insert(nodes.begin(), nodes.end());
  }
  for(auto seg : verticalSegments)
  {
    const std::vector<Node*> nodes = gridGraph_->intersectingNodes({seg.x1, seg.y1, seg.x2, seg.y2});
    verticalNodes.insert(nodes.begin(), nodes.end());
  }
  // Increase demand of those nodes
  for(auto node : horizontalNodes)
    node->horizontal_demand += 1;
  for(auto node : verticalNodes)
    node->vertical_demand += 1;
}

}
