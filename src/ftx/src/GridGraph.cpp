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
GridGraph::initGridFromCoords(std::vector<Utils::DBU> xTicks,
                              std::vector<Utils::DBU> yTicks)
{
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
GridGraph::neighborhood(Node* node, int distance)
{
  // First check if we have a valid neighborhood (try to walk
  // distance times on each direction)
  // Up
  ftx::Node* upNeighbor = node;
  for(int i = 0; i < distance; ++i)
  {
    upNeighbor = upNode(upNeighbor->nodeID);
    if (upNeighbor == nullptr)
      return {};
  }
  // Down
  ftx::Node* downNeighbor = node;
  for(int i = 0; i < distance; ++i)
  {
    downNeighbor = downNode(downNeighbor->nodeID);
    if (downNeighbor == nullptr)
      return {};
  }
  // Left
  ftx::Node* leftNeighbor = node;
  for(int i = 0; i < distance; ++i)
  {
    leftNeighbor = leftNode(leftNeighbor->nodeID);
    if (leftNeighbor == nullptr)
      return {};
  }
  // Right
  ftx::Node* rightNeighbor = node;
  for(int i = 0; i < distance; ++i)
  {
    rightNeighbor = rightNode(rightNeighbor->nodeID);
    if (rightNeighbor == nullptr)
      return {};
  }
  //Then move to the starting point to follow row-major order
  // when printing congestion attributes
  ftx::Node *upLeft = upNeighbor;
  for(int i = 0; i < distance; ++i)
    upLeft = leftNode(upLeft->nodeID);

  std::vector<Node*> result;
  result.reserve((distance*2+1)*(distance*2+1));
  int numJumps = distance*2;
  ftx::Node *leftMostNode = upLeft;
  //for columns
  for(int i = 0; i <= numJumps; ++i)
  {
    //for rows
    ftx::Node *currentNode = leftMostNode;
    for(int j = 0; j <= numJumps; ++j)
    {
      result.push_back(currentNode);
      currentNode = rightNode(currentNode->nodeID);
    }
    leftMostNode = downNode(leftMostNode->nodeID);
  }
  return result;
}

bool
GridGraph::neighborhoodCongestion(Node* node, int size, std::string &result)
{
  // First check if we have a valid neighborhood (try to walk
  // size times on each direction)
  // Up
  ftx::Node* upNeighbor = node;
  for(int i = 0; i < size; ++i)
  {
    upNeighbor = upNode(upNeighbor->nodeID);
    if (upNeighbor == nullptr)
      return false;
  }
  // Down
  ftx::Node* downNeighbor = node;
  for(int i = 0; i < size; ++i)
  {
    downNeighbor = downNode(downNeighbor->nodeID);
    if (downNeighbor == nullptr)
      return false;
  }
  // Left
  ftx::Node* leftNeighbor = node;
  for(int i = 0; i < size; ++i)
  {
    leftNeighbor = leftNode(leftNeighbor->nodeID);
    if (leftNeighbor == nullptr)
      return false;
  }
  // Right
  ftx::Node* rightNeighbor = node;
  for(int i = 0; i < size; ++i)
  {
    rightNeighbor = rightNode(rightNeighbor->nodeID);
    if (rightNeighbor == nullptr)
      return false;
  }
  //Then move to the starting point to follow row-major order
  // when printing congestion attributes
  ftx::Node *upLeft = upNeighbor;
  for(int i = 0; i < size; ++i)
    upLeft = leftNode(upLeft->nodeID);

  int numJumps = size*2;
  ftx::Node *leftMostNode = upLeft;
  //for columns
  for(int i = 0; i <= numJumps; ++i)
  {
    //  for rows
    ftx::Node *currentNode = leftMostNode;
    for(int j = 0; j <= numJumps; ++j)
    {
      result += currentNode->printCongestion();
      currentNode = rightNode(currentNode->nodeID);
    }
    leftMostNode = downNode(leftMostNode->nodeID);
  }
  result.pop_back();// remove last character (space)
  return true;
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
