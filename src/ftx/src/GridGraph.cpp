#include "ftx/GridGraph.h"

#include "odb/db.h"

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/graph/grid_graph.hpp>

// Shortcuts for Boost namespace.
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// Define a 2D cartesian point using geometry box of DBUs.
typedef bg::model::point<Utils::DBU, 2, bg::cs::cartesian> point_t;
typedef bg::model::box<point_t> box_t;

// Define RTree type of DBU box using R-Star algorithm.
typedef std::pair<box_t, ftx::vertexIndex> treeElement;
typedef bgi::rtree<treeElement, bgi::rstar<16>> RTree;

// Define a 2D Grid Graph with no wrapping.
typedef boost::grid_graph<2> Graph;

namespace ftx {

enum Axis { x = 0, y = 1 };

GridGraph::GridGraph() :
  db_(nullptr),
  graph_(nullptr),
  rTree_(nullptr)
{
}

GridGraph::~GridGraph()
{
  xGrid_.clear();
  yGrid_.clear();
  vertex2Node_.clear();
  if( graph_ != nullptr)
    delete (Graph*) graph_;
  if( rTree_ != nullptr)
    delete (RTree*) rTree_;
}

void
GridGraph::initGridFromDEFGCells(odb::dbDatabase* db)
{
  db_ = db;
  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::dbGCellGrid* grid = block->getGCellGrid();
  grid->getGridX(xGrid_);
  grid->getGridY(yGrid_);
  buildGraph();
}

void
GridGraph::initGridFromCoords(odb::dbDatabase* db,
                              std::vector<Utils::DBU> xTicks,
                              std::vector<Utils::DBU> yTicks)
{
  db_ = db;
  //sorted grid without duplicates.
  xGrid_ = xTicks;
  std::sort(xGrid_.begin(), xGrid_.end());
  xGrid_.erase(std::unique(xGrid_.begin(), xGrid_.end()), xGrid_.end());
  yGrid_ = yTicks;
  std::sort(yGrid_.begin(), yGrid_.end());
  yGrid_.erase(std::unique(yGrid_.begin(), yGrid_.end()), yGrid_.end());
  buildGraph();
}

void
GridGraph::initGridUsingDimensions(odb::dbDatabase* db,
                                   Utils::DBU width,
                                   Utils::DBU height)
{
  db_ = db;
  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::Rect core_area;
  block->getCoreArea(core_area);
  std::vector<Utils::DBU> xTicks, yTicks;

  auto x_steps = 1 + ((core_area.dx()-1)/width);// if core_area.dx() != 0
  xTicks.reserve(x_steps);
  for(auto i = 0; i <= x_steps; i++)
    xTicks.push_back(std::min(core_area.xMin()+i*width, core_area.xMax()));

  auto y_steps = 1 + ((core_area.dy()-1)/height);// if core_area.dy() != 0
  yTicks.reserve(y_steps);
  for(auto i = 0; i <= y_steps; i++)
    yTicks.push_back(std::min(core_area.yMin()+i*height, core_area.yMax()));

  xGrid_ = xTicks;
  yGrid_ = yTicks;
  buildGraph();
}

Node*
GridGraph::intersectingNode(odb::Point point)
{
  RTree* rTree = (RTree*) rTree_;
  point_t query_point(point.x(), point.y());
  std::vector<treeElement> vertices_found;
  rTree->query(bgi::intersects(query_point),
               std::back_inserter(vertices_found));
  if(vertices_found.empty())
    return nullptr;
  return &vertex2Node_.at(vertices_found.front().second);
}

std::vector<Node*>
GridGraph::intersectingNodes(odb::Rect rect)
{
  RTree* rTree = (RTree*) rTree_;
  std::vector<Node*> result;
  box_t query_box({rect.xMin(), rect.yMin()},
                  {rect.xMax(), rect.yMax()});
  std::vector<treeElement> vertices_found;
  rTree->query(bgi::intersects(query_box),
               std::back_inserter(vertices_found));
  result.reserve(vertices_found.size());
  for(auto tree_element : vertices_found)
    result.push_back(&vertex2Node_.at(tree_element.second));
  return result;
}

std::vector<Node*>
GridGraph::neighbors(vertexIndex node_index)
{
  std::vector<Node*> result;
  result.reserve(4);
  auto up_ptr = upNode(node_index);
  if(up_ptr != nullptr)
    result.push_back(up_ptr);
  auto down_ptr = downNode(node_index);
  if(down_ptr != nullptr)
    result.push_back(down_ptr);
  auto left_ptr = leftNode(node_index);
  if(left_ptr != nullptr)
    result.push_back(left_ptr);
  auto right_ptr = rightNode(node_index);
  if(right_ptr != nullptr)
    result.push_back(right_ptr);
  return result;
}

std::vector<Node*>
GridGraph::neighborhood(vertexIndex node_index)
{
  auto result = neighbors(node_index);
  auto up_ptr = upNode(node_index);
  if(up_ptr != nullptr)
  {
    auto left = leftNode(up_ptr->nodeID);
    if(left != nullptr)
      result.push_back(left);
    auto right = rightNode(up_ptr->nodeID);
    if(right != nullptr)
      result.push_back(right);
  }
  auto down_ptr = downNode(node_index);
  if(down_ptr != nullptr)
  {
    auto left = leftNode(down_ptr->nodeID);
    if(left != nullptr)
      result.push_back(left);
    auto right = rightNode(down_ptr->nodeID);
    if(right != nullptr)
      result.push_back(right);
  }
  return result;
}

std::string
GridGraph::neighborhoodFeatures(Node* node)
{
  auto neighborhoodNodes = neighborhood(node->nodeID);
  Utils::AreaDBU totalTileArea = 0;
  Utils::AreaDBU totalCellArea = 0;
  Utils::AreaDBU totalL1PinArea = 0;
  Utils::AreaDBU totalL2PinArea = 0;
  Utils::AreaDBU totalL1BlkgArea = 0;
  Utils::AreaDBU totalL2BlkgArea = 0;
  Utils::AreaDBU totalMacroArea = 0;
  Utils::AreaDBU totalMacroPinArea = 0;

  int totalNumCells = 0;
  int totalNumCellPins = 0;
  int totalNumMacros = 0;
  int totalNumMacroPins = 0;
  int totalNumPassingNets = 0;
  int totalVerticalOverflow = 0;
  int totalVerticalRemain = 0;
  int totalVerticalTracks = 0;
  int totalHorizontalOverflow = 0;
  int totalHorizontalRemain = 0;
  int totalHorizontalTracks = 0;

  for(auto node : neighborhoodNodes)
  {
    totalTileArea += node->rect.area();
    totalCellArea += node->cellArea;
    totalL1PinArea += node->l1PinArea;
    totalL2PinArea += node->l2PinArea;
    totalL1BlkgArea += node->l1BlockageArea;
    totalL2BlkgArea += node->l2BlockageArea;
    totalMacroArea += node->macroArea;
    totalMacroPinArea += node->macroPinArea;

    totalNumCells += node->numCells;
    totalNumCellPins += node->numCellPins;
    totalNumMacros += node->numMacros;
    totalNumMacroPins += node->numMacroPins;
    totalNumPassingNets += node->numPassingNets;
    totalVerticalOverflow += node->vertical_overflow;
    totalVerticalRemain += node->vertical_remain;
    totalVerticalTracks += node->vertical_tracks;
    totalHorizontalOverflow += node->horizontal_overflow;
    totalHorizontalRemain += node->horizontal_remain;
    totalHorizontalTracks += node->horizontal_tracks;
  }

  std::string result = "";
  result += std::to_string(totalTileArea) + ", "
         + std::to_string((double)totalCellArea/totalTileArea) + ", "
         + std::to_string((double)totalL1PinArea/totalTileArea) + ", "
         + std::to_string((double)totalL2PinArea/totalTileArea) + ", "
         + std::to_string((double)totalL1BlkgArea/totalTileArea) + ", "
         + std::to_string((double)totalL2BlkgArea/totalTileArea) + ", "
         + std::to_string((double)totalMacroArea/totalTileArea) + ", "
         + std::to_string((double)totalMacroPinArea/totalTileArea) + ", "
         + std::to_string(totalNumCells) + ", "
         + std::to_string(totalNumCellPins) + ", "
         + std::to_string(totalNumMacros) + ", "
         + std::to_string(totalNumMacroPins) + ", "
         + std::to_string(totalNumPassingNets) + ", "
         + std::to_string(totalVerticalOverflow) + ", "
         + std::to_string(totalVerticalRemain) + ", "
         + std::to_string(totalVerticalTracks) + ", "
         + std::to_string(totalHorizontalOverflow) + ", "
         + std::to_string(totalHorizontalRemain) + ", "
         + std::to_string(totalHorizontalTracks);
  return result;
}

Node*
GridGraph::upNode(vertexIndex node_index)
{
  return adjacentNode(node_index, Axis::y, false);
}

Node*
GridGraph::downNode(vertexIndex node_index)
{
  return adjacentNode(node_index, Axis::y, true);
}


Node*
GridGraph::leftNode(vertexIndex node_index)
{
  return adjacentNode(node_index, Axis::x, false);
}

Node*
GridGraph::rightNode(vertexIndex node_index)
{
  return adjacentNode(node_index, Axis::x, true);
}

Node*
GridGraph::adjacentNode(vertexIndex node_index,
                       unsigned int axis,
                       bool next)
{
  Graph *graph = (Graph*) graph_;
  auto v = vertex(node_index, *graph);
  auto adj = next ? graph->next(v, axis) : graph->previous(v, axis);
  auto adj_id = get(boost::vertex_index, *graph, adj);
  return adj_id == node_index ? nullptr : &vertex2Node_.at(adj_id);
}

Node*
GridGraph::node(vertexIndex node_index)
{
  return &vertex2Node_.at(node_index);
}

long unsigned int
GridGraph::XLength() const
{
  Graph *graph = (Graph*) graph_;
  return graph->length(Axis::x);
}

long unsigned int
GridGraph::YLength() const
{
  Graph *graph = (Graph*) graph_;
  return graph->length(Axis::y);
}

unsigned int
GridGraph::sizeNodes() const
{
  return vertex2Node_.size();
}

void
GridGraph::buildGraph()
{
  boost::array<std::size_t, 2> lengths = {{xGrid_.size()-1, yGrid_.size()-1}};
  boost::array<bool, 2> wrapped = {{false, false}};
  rTree_ = (void*) (new RTree);
  graph_ = new Graph(lengths, wrapped);
  unsigned int num_nodes = (xGrid_.size()-1)*(yGrid_.size()-1);
  vertex2Node_.reserve(num_nodes);
  RTree* rtree = (RTree*) rTree_;
  Utils::DBU prev_y = *yGrid_.rbegin();
  Utils::DBU prev_x = *xGrid_.begin();
  auto x_length = xGrid_.size()-1;
  for(auto y_it = std::next(yGrid_.rbegin()); y_it != yGrid_.rend(); y_it++)
  {
    for(auto x_it = std::next(xGrid_.begin()); x_it != xGrid_.end(); x_it++)
    {
      auto x_id = std::distance(xGrid_.begin(), x_it)-1;
      auto y_id = std::distance(yGrid_.rbegin(), y_it)-1;
      auto node_id = y_id * x_length + x_id;
      odb::Rect node_box(prev_x, *y_it, *x_it, prev_y);
      Node node(node_id, node_box);
      vertex2Node_.insert({node_id, node});
      box_t gcell_box({prev_x, *y_it}, {*x_it, prev_y});
      rtree->insert({gcell_box, node_id});
      prev_x = *x_it;
    }
    prev_x = *xGrid_.begin();
    prev_y = *y_it;
  }
}

}
