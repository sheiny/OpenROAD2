#pragma once

#include "Node.h"
#include "Types.h"
#include <vector>
#include <unordered_map>

namespace odb {
  class dbDatabase;
}

namespace ftx {

class GridGraph
{
  public:
    GridGraph();
    ~GridGraph();

    //Builder functions
    void initGridFromDEFGCells(odb::dbDatabase* db);
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

    //Given a NeighborhoodSize it will extract the neighborhood window
    //This window is calculated using Width = size*2+1 and Height = size*2+1
    bool neighborhoodCongestion(Node* node, int size, std::string &result);

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
    void* graph_;
    void* rTree_;
};

}
