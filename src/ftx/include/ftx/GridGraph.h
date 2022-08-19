#pragma once

#include "Node.h"
#include "Types.h"
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/graph/grid_graph.hpp>
#include <memory>
#include <unordered_map>
#include <vector>

namespace odb {
  class dbDatabase;
}

// Shortcuts for Boost namespace.
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

namespace ftx {

class GridGraph
{
  private:
  // Define a 2D cartesian point using geometry box of DBUs.
  typedef bg::model::point<Utils::DBU, 2, bg::cs::cartesian> point_t;
  typedef bg::model::box<point_t> box_t;
  // Define RTree type of DBU box using R-Star algorithm.
  typedef std::pair<box_t, ftx::vertexIndex> treeElement;
  typedef bgi::rtree<treeElement, bgi::rstar<16>> RTree;
  // Define a 2D Grid Graph with no wrapping.
  typedef boost::grid_graph<2> Graph;
  public:
    //Builder functions
    void initGridFromCoords(std::vector<Utils::DBU> xTicks,
                            std::vector<Utils::DBU> yTicks);
    void initGridUsingDimensions(odb::dbDatabase* db,
                                 Utils::DBU width,
                                 Utils::DBU height);

    //This function returns a nullptr when there is no intersecting node.
    Node* intersectingNode(odb::Point point);

    //WARNING: This function returns nodes with zero intersection area.
    std::vector<Node*> intersectingNodes(odb::Rect rect);

    //Given a node index, this function returns the four horizontal
    // and vertical adjacent nodes.
    std::vector<Node*> neighbors(vertexIndex node_index);

    //Given a node index, this function returns the distance adjacent nodes.
    //Ex: distance 1 = eight adjacent nodes
    //Width = distance*2+1 and Height = distance*2+1
    std::vector<Node*> neighborhood(Node* node, int distance);

    Node* upNode(vertexIndex node_index);
    Node* downNode(vertexIndex node_index);
    Node* leftNode(vertexIndex node_index);
    Node* rightNode(vertexIndex node_index);

    //Getters
    Node* node(vertexIndex node_index);
    long unsigned int XLength() const;
    long unsigned int YLength() const;
    unsigned int sizeNodes() const;

  private:
    void buildGraph();
    Node* adjacentNode(vertexIndex node_index,
                       unsigned int axis,
                       bool next);

    odb::dbDatabase* db_;
    std::vector<Utils::DBU> xGrid_;
    std::vector<Utils::DBU> yGrid_;
    //Do not modify this map outside the buildGraph function
    //Otherwise, all Node references might be invalidated.
    std::unordered_map<vertexIndex, Node> vertex2Node_;
    std::unique_ptr<Graph> graph_;
    std::unique_ptr<RTree> rTree_;
};

}
