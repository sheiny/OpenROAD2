#include "ftx/FeatureExtractor.h"
#include "gui/gui.h"
#include "ftx/GridGraph.h"
#include "ftx/Utils.h"
#include "ftx/RptParser.h"
#include "ord/OpenRoad.hh"
#include "odb/db.h"
#include "odb/dbTransform.h"
#include "stt/SteinerTreeBuilder.h"

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <unordered_set>

#if 0
//Code for test numberOfTracksBetween
std::vector<int> elements = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
std::vector<std::tuple<int, int, int>> tests = {{80, 95, 1}, {85, 95, 1}, {5, 35, 3},
  {5, 30, 3}, {0, 100, 10}, {0, 110, 10}, {99, 110, 1}, {5, 7, 0}, {89, 110, 2}, {89, 90, 1}};
for(auto t : tests)
{
  int result = numberOfTracksBetween(elements, std::get<0>(t), std::get<1>(t));
  if(result != std::get<2>(t))
  {
    std::cout<<"Test fail for lb= "<<std::get<0>(t)<<" up= "<<std::get<1>(t)<<std::endl;
    std::cout<<"dist= "<<result<<" should be "<<std::get<2>(t)<<std::endl;
  }
}
#endif
int
numberOfTracksBetween(const std::vector<int> &elements, int lowerBound, int upperBound)
{
    auto itlb = std::lower_bound(elements.begin(), elements.end(), lowerBound);
    auto itub = itlb;
    int distance = 0;
    while(*itub <= upperBound && itub != elements.end())
    {
      itub++;
      distance++;
    }
    if(*itlb == lowerBound)
      distance--;
    return distance;
}

struct TrackGrid
{
  int routing_level;
  std::vector<int> xTicks, yTicks;
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

struct GuideSegment
{
  int layerIndex;
  odb::Rect rect;
  bool horizontal;
};

struct RoutingGuide
{
  std::string netName;
  std::vector<GuideSegment> guides;
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

    //Consider both options for each segment
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

FeatureExtractor::~FeatureExtractor() = default;

void
FeatureExtractor::initGraph(int sizeInRowHeights)
{
  std::cout<<"FeatureExtractor::initGraph called."<<std::endl;
  gridGraph_ = std::make_unique<ftx::GridGraph>();
  auto block = db_->getChip()->getBlock();
  auto row = block->getRows().begin();
  auto site = row->getSite();
  auto height = site->getHeight();
  auto tile_dim = height * sizeInRowHeights;
  gridGraph_->initGridUsingDimensions(db_, tile_dim, tile_dim);
  initRoutingCapacity();
}

void
FeatureExtractor::initGraphFromDef()
{
  std::cout<<"FeatureExtractor::initGraphFromDef called."<<std::endl;
  gridGraph_ = std::make_unique<ftx::GridGraph>();
  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::dbGCellGrid* grid = block->getGCellGrid();
  std::vector<Utils::DBU> xTicks, yTicks;
  grid->getGridX(xTicks);
  grid->getGridY(yTicks);
  gridGraph_->initGridFromCoords(xTicks, yTicks);
  initRoutingCapacity();
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
  std::cout<<"Nodes with DRVs: "<<num_violating_nodes<<" of "<<gridGraph_->sizeNodes()<<" total nodes."<<std::endl;
}

void
FeatureExtractor::extractFeatures()
{
  std::cout<<"FeatureExtractor::extractFeatures called."<<std::endl;
  auto block = db_->getChip()->getBlock();
  // Placement features
  for(auto inst : block->getInsts())
  {
    std::string instName = inst->getName();
    odb::dbMaster *master = inst->getMaster();
    std::string masterName = master->getName();
    if(master->isFiller() ||
       boost::icontains(instName, "fill") ||
       boost::icontains(masterName, "fill"))
      continue;
    extractInstFeatures(inst);
  }

  // Routing features
  block->setDrivingItermsforNets();
  for(auto net : block->getNets())
    extractRoutingFeatures(net);
}

void
FeatureExtractor::extractCNNFeatures()
{
  auto block = db_->getChip()->getBlock();
  for(auto inst : block->getInsts())
  {
		if(inst->isCore() == false)
			continue;
    std::string instName = inst->getName();
    odb::dbMaster *master = inst->getMaster();
    std::string masterName = master->getName();
    if(master->isFiller() ||
       boost::icontains(instName, "fill") ||
       boost::icontains(masterName, "fill"))
      continue;

		odb::Rect bbox_inst = inst->getBBox()->getBox();
		std::vector<ftx::Node*> nodes = gridGraph_->intersectingNodes(bbox_inst);
		odb::dbTransform transform;
		inst->getTransform(transform);
		for(auto node : nodes)
		{
			odb::Rect node_rect = node->rect;
			odb::Rect intersection;
			node_rect.intersection(bbox_inst, intersection);
			auto area = intersection.area();
			if(area == 0)
        continue;

			for(auto term : master->getMTerms())
			{
				if(term->getName() == "VSS" || term->getName() == "VDD")
					continue;
				for(auto pin : term->getMPins())
				{
					odb::Rect pinIntersection;
					odb::dbSet<odb::dbBox> boxes = pin->getGeometry();
          for(auto box : boxes)
					{
						odb::Rect pinRect = box->getBox();
						transform.apply(pinRect);
			      node_rect.intersection(pinRect, pinIntersection);
						if(pinIntersection.area() > 0)
              break;
					}
          if(pinIntersection.area() > 0)
						node->numPins += 1;
				}
			}
		}
  }
	for(int node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
	{
		ftx::Node *node = gridGraph_->node(node_id);
    std::vector<Node*> neighbors = gridGraph_->neighborhood(node, 1);
    for(auto neighbor : neighbors)
		  node->numNeighborPins+=neighbor->numPins;
		node->numNeighborPins-=node->numPins;
	}
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
  distance = std::max(distance, 1); //otherwise surround nodes would be empty
  int neighborhoodSize = (distance*2+1)*(distance*2+1);
  std::string header = "NodeID,";
  std::vector<std::string> features = {"#Cells,", "#CellPins,", "#Macros,",
    "#MacroPins,", "HorizontalOverflow,", "VerticalOverflow,", "TileArea,",
    "CellDensity,", "MacroDensity,", "MacroPinDensity,", "Layer1BlkgDensity,",
    "Layer2BlkgDensity,", "Layer1PinDensity,", "Layer2PinDensity,"};
  for(int i = 0; i < neighborhoodSize; i++)
    for(std::string feature : features)
      header += std::to_string(i)+"_"+feature;
  //drv types
  header += "AdjacentCutSpacing,SameLayerCutSpacing,EndOfLine,FloatingPatch,";
  header += "MinArea,MinWidth,NonSuficientMetalOverlap,CutShort,MetalShort,";
  header += "OutOfDieShort,CornerSpacing,ParallelRunLength,HasDetailedRoutingViolation";
  //count global routing neighbor features
  std::cout<<"FeatureExtractor::writeCSV called."<<std::endl
           <<"Writing nodes with violation into "+file_path+"_viol.csv"<<std::endl;
  std::ofstream ofsViol(file_path+"_viol.csv", std::ofstream::out);
  ofsViol<<header<<std::endl;

  std::unordered_set<Node*> surroundingNodes;
  for(auto node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
  {
    Node *node_ptr = gridGraph_->node(node_id);
    if(node_ptr->violation == false)//skip nonViol nodes
      continue;

    std::vector<Node*> neighborhood = gridGraph_->neighborhood(node_ptr, distance);
    if(neighborhood.empty())//invalid neighborhood padding is required
      continue;

    ofsViol<<std::to_string(node_ptr->nodeID)<<", ";
    for(Node* neighbor : neighborhood)//this follows the major order
    {
      ofsViol<<neighbor->printPlacementFeatures(", ", false);
      if(neighbor->violation == false)
        surroundingNodes.insert(neighbor);
    }
    //print node info
    //print neighborhood folowing row major order print node infos
    //print node violation info
    ofsViol<<node_ptr->printDRVs();
  }
  ofsViol.close();


  std::cout<<"Writing nodes surrounded by violations into "+file_path+"_surround.csv"<<std::endl;
  std::ofstream ofsSurround(file_path+"_surround.csv", std::ofstream::out);
  ofsSurround<<header<<std::endl;
  for(Node* node : surroundingNodes)
  {
    std::vector<Node*> neighborhood = gridGraph_->neighborhood(node, distance);
    if(neighborhood.empty())//invalid neighborhood padding is required
      continue;

    ofsSurround<<std::to_string(node->nodeID)<<", ";
    for(Node* neighbor : neighborhood)//this follows the major order
      ofsSurround<<neighbor->printPlacementFeatures(", ", false);
    //print node info
    //print neighborhood folowing row major order print node infos
    //print node violation info
    ofsSurround<<node->printDRVs();
  }
  ofsSurround.close();


  std::cout<<"Writing non violating nodes into "+file_path+"_nonViol.csv"<<std::endl;
  std::ofstream ofsNonViol(file_path+"_nonViol.csv", std::ofstream::out);
  ofsNonViol<<header<<std::endl;
  for(auto node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
  {
    Node *node_ptr = gridGraph_->node(node_id);
    if(node_ptr->violation == true)//skip viol nodes
      continue;

    if(surroundingNodes.find(node_ptr) != surroundingNodes.end())//skip surround nodes
      continue;

    std::vector<Node*> neighborhood = gridGraph_->neighborhood(node_ptr, distance);
    if(neighborhood.empty())//invalid neighborhood padding is required
      continue;

    ofsNonViol<<std::to_string(node_ptr->nodeID)<<", ";
    for(Node* neighbor : neighborhood)//this follows the major order
      ofsNonViol<<neighbor->printPlacementFeatures(", ", false);
    //print node info
    //print neighborhood folowing row major order print node infos
    //print node violation info
    ofsNonViol<<node_ptr->printDRVs();
  }
  ofsNonViol.close();
}

std::string
FeatureExtractor::nodeHyperImage(Node*node, const std::vector<Node*>& neighbors)
{
  std::string result;
	for(auto neighbor : neighbors)
		result+=std::to_string(neighbor->numPins)+",";
	for(auto neighbor : neighbors)
		result+=std::to_string(neighbor->numNeighborPins)+",";
  int numLayers = node->v_cap.size();

  //horizontal overflow
  for(int l = 0; l<numLayers; ++l)
    for(auto neighbor : neighbors)
		{
			double hOverflow = double(neighbor->h_demand.at(l))/neighbor->h_cap.at(l);
			std::ostringstream ost;// Create an output string stream
			ost << std::fixed;// Set Fixed -Point Notation
			ost << std::setprecision(2);// Set precision to 2 digits
			ost << hOverflow;//Add double to stream
			ost.str();// Get string from output string stream
			result+=ost.str()+",";
		}

  //vertical overflow
  for(int l = 0; l<numLayers; ++l)
    for(auto neighbor : neighbors)
		{
			double vOverflow = double(neighbor->v_demand.at(l))/neighbor->v_cap.at(l);
			std::ostringstream ost;// Create an output string stream
			ost << std::fixed;// Set Fixed -Point Notation
			ost << std::setprecision(2);// Set precision to 2 digits
			ost << vOverflow;//Add double to stream
			ost.str();// Get string from output string stream
			result+=ost.str()+",";
		}
  if(node->drvs.find(DRVType::metalShort) != node->drvs.end())
	  result+="1";
	else
	  result+="0";
  return result;
}

void
FeatureExtractor::writeCNNCSV(std::string file_path, int distance)
{
  distance = std::max(distance, 16); //otherwise surround nodes would be empty
  std::ofstream ofsviol(file_path+".csv", std::ofstream::out);
  for(auto node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
  {
    Node *node_ptr = gridGraph_->node(node_id);
    std::vector<Node*> neighborhood = gridGraph_->neighborhood(node_ptr, distance);
    if(neighborhood.empty())//invalid neighborhood padding is required
      continue;
    ofsviol<<nodeHyperImage(node_ptr, neighborhood)<<std::endl;
  }
  ofsviol.close();
}

void
FeatureExtractor::writeCNNCSVs(std::string file_path, int distance)
{
  distance = std::max(distance, 16); //otherwise surround nodes would be empty

  std::unordered_set<Node*> surroundingNodes;
  std::vector<Node*> violatingNodes;
  for(auto node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
  {
    Node *node_ptr = gridGraph_->node(node_id);
    if(node_ptr->drvs.find(DRVType::metalShort) == node_ptr->drvs.end())//skip nonShortViol nodes
      continue;

    std::vector<Node*> neighborhood = gridGraph_->neighborhood(node_ptr, distance);
    if(neighborhood.empty())//invalid neighborhood padding is required
      continue;

    for(Node* neighbor : neighborhood)//this follows the major order
      if(neighbor->violation == false && !gridGraph_->neighborhood(neighbor, distance).empty())
        surroundingNodes.insert(neighbor);
    violatingNodes.push_back(node_ptr);
  }

  std::vector<Node*> nonViolatingNodes;
  for(auto node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
  {
    Node *node_ptr = gridGraph_->node(node_id);
    if(node_ptr->violation == true)//skip viol nodes
      continue;
    if(surroundingNodes.find(node_ptr) != surroundingNodes.end())//skip surround nodes
      continue;

    if(gridGraph_->neighborhood(node_ptr, distance).empty())//invalid neighborhood padding is required
      continue;
    nonViolatingNodes.push_back(node_ptr);
  }

  std::ofstream ofsviol(file_path+"_viol.csv", std::ofstream::out);
  for(auto node : violatingNodes)
    ofsviol<<nodeHyperImage(node, gridGraph_->neighborhood(node, distance))<<std::endl;
  ofsviol.close();

  std::ofstream ofsNonviol(file_path+"_nonViol.csv", std::ofstream::out);
  for(auto node : nonViolatingNodes)
    ofsNonviol<<nodeHyperImage(node, gridGraph_->neighborhood(node, distance))<<std::endl;
  ofsNonviol.close();

  std::ofstream ofsSurround(file_path+"_surround.csv", std::ofstream::out);
  for(auto node : surroundingNodes)
    ofsSurround<<nodeHyperImage(node, gridGraph_->neighborhood(node, distance))<<std::endl;
  ofsSurround.close();
}

class RectRender : public gui::Renderer
{
public:
  RectRender(odb::dbDatabase* db) :
    db_(db)
  {
  }

  void addRect(odb::Rect r){ rects_.push_back(r); };

  virtual void drawObjects(gui::Painter& /* painter */) override;

private:
  odb::dbDatabase* db_;
  std::vector<odb::Rect> rects_;
};

void
RectRender::drawObjects(gui::Painter &painter)
{
  for(auto rect : rects_)
	{
		painter.setBrush(gui::Painter::dark_red);
		painter.drawRect(rect);
	}
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
      if (node_ptr->drvs.find(DRVType::metalShort) != node_ptr->drvs.end())
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
    drvRenderer_ = std::make_unique<DRVRenderer>(db_, gridGraph_.get());
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
  if (gridRenderer_ == nullptr)
  {
    gui::Gui* gui = gui::Gui::get();
    gridRenderer_ = std::make_unique<GridRender>(db_, gridGraph_.get());
    gui->registerRenderer(gridRenderer_.get());
    gui->redraw();
  }
}

void
FeatureExtractor::initRoutingCapacity()
{
  odb::dbBlock *block = db_->getChip()->getBlock();
  std::vector<TrackGrid> trackGrids;
  odb::dbSet<odb::dbTrackGrid> grids = block->getTrackGrids();
  for(auto trackGrid : grids)
  {
    odb::dbTechLayer *techLayer = trackGrid->getTechLayer();
    TrackGrid t_grid;
    t_grid.routing_level = techLayer->getRoutingLevel();
    if(t_grid.routing_level == 0)
      continue;
    odb::dbTechLayerDir layerDirection = techLayer->getDirection();
    if(layerDirection.getValue() == odb::dbTechLayerDir::Value::NONE)
      continue;
    trackGrid->getGridX(t_grid.xTicks);
    trackGrid->getGridY(t_grid.yTicks);
    trackGrids.push_back(t_grid);
  }
  int numRoutingLayers = trackGrids.size();
  for(int node_id = 0; node_id != gridGraph_->sizeNodes(); node_id++)
  {
    ftx::Node *node = gridGraph_->node(node_id);
    node->h_cap.resize(numRoutingLayers);
    node->v_cap.resize(numRoutingLayers);
    node->h_demand.resize(numRoutingLayers);
    node->v_demand.resize(numRoutingLayers);
    odb::Rect rect =  node->rect;
    for(auto grid : trackGrids)
    {
      node->v_cap.at(grid.routing_level-1) = numberOfTracksBetween(grid.xTicks, rect.xMin(), rect.xMax());
      node->h_cap.at(grid.routing_level-1) = numberOfTracksBetween(grid.yTicks, rect.yMin(), rect.yMax());
		}
    for(int l=0;l<numRoutingLayers;++l)
		{
      node->horizontal_capacity+=node->h_cap.at(l);
      node->vertical_capacity+=node->v_cap.at(l);
		}
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
    nodePainter_ = std::make_unique<NodePainter>(gridGraph_.get(), nodeIds);
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
    nodePainter_ = std::make_unique<NodePainter>(gridGraph_.get(), nodeIds);
    gui->registerRenderer(nodePainter_.get());
  }
  nodePainter_->addNode(id);
  gui->redraw();
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
    return;//throw std::logic_error("Error, net without a driver: "+net->getName());

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

void
FeatureExtractor::readGuide(std::string file_path)
{
  std::ifstream iss;
  iss.open(file_path);
  std::string line;
  std::vector<RoutingGuide> guides;
  int dbus_per_micron = db_->getLibs().begin()->getDbUnitsPerMicron();
  while(std::getline(iss, line))
  {
    RoutingGuide rguide;
    if(line.find("routeGuideNet") != std::string::npos)
    {
      std::string netName = line.substr(line.find(" ")+1, line.length());
      rguide.netName = netName;
      while(std::getline(iss, line) && line.find("wire") != std::string::npos)
      {
        std::istringstream values(line);
        std::string value;
        double x1, y1, x2, y2;
        values >> value;
        values >> x1;
        values >> y1;
        values >> x2;
        values >> y2;
        values >> value;
        bool horizontal = value == "H";
        values >> value;
        values >> value;
        values >> value;
        int layer = std::stoi(value);
        GuideSegment gsegment;
        gsegment.horizontal = horizontal;
        gsegment.rect = odb::Rect(x1*dbus_per_micron, y1*dbus_per_micron, x2*dbus_per_micron, y2*dbus_per_micron);
        if(gsegment.rect.area() > 0)
					throw std::out_of_range("There is a guide rectangle that is not from eGR (area greater than zero).");
        gsegment.layerIndex = layer;
        rguide.guides.push_back(gsegment);
      }
    }
    guides.push_back(rguide);
  }

  int numLayers = gridGraph_->node(0)->h_demand.size();
  for(auto rguide : guides)
  {
    std::vector<std::unordered_set<Node*>> horizontalNodes;
    horizontalNodes.resize(numLayers);
    for(auto guideSegment : rguide.guides)
    {
      if(!guideSegment.horizontal)
        continue;
      std::vector<Node*> nodes = gridGraph_->intersectingNodes(guideSegment.rect);
      for(auto node : nodes)
      {
				if(guideSegment.rect.yMin() == node->rect.yMin())
          continue;//touching horizontal botton edge
        horizontalNodes.at(guideSegment.layerIndex-1).insert(node);
      }
    }

    std::vector<std::unordered_set<Node*>> verticalNodes;
    verticalNodes.resize(numLayers);
    for(auto guideSegment : rguide.guides)
		{
      if(guideSegment.horizontal)
        continue;
      std::vector<Node*> nodes = gridGraph_->intersectingNodes(guideSegment.rect);
      for(auto node : nodes)
      {
			  if(guideSegment.rect.xMin() == node->rect.xMin())
				  continue;//touching vertical left edge
        verticalNodes.at(guideSegment.layerIndex-1).insert(node);
      }
		}

    for(int i = 0; i<horizontalNodes.size(); i++)
      for(auto node : horizontalNodes.at(i))
        node->h_demand.at(i)++;

    for(int i = 0; i<verticalNodes.size(); i++)
      for(auto node : verticalNodes.at(i))
        node->v_demand.at(i)++;
  }
}

}
