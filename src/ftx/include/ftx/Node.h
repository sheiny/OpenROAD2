#pragma once

#include "Types.h"
#include "odb/geom.h"

#include <iosfwd>
#include <unordered_set>

namespace ftx {

struct Node {
    Node(vertexIndex node_id, odb::Rect rect) :
      nodeID(node_id),
      rect(rect)
    {
    }

    std::string printPlacementFeatures(std::string separator, bool debug);

    std::string printDRVs();

    //Graph attributes
    const vertexIndex nodeID;
    const odb::Rect rect;
    //Placement Features
    int numCells = 0;
    int numCellPins = 0;
    int numMacros = 0;
    int numMacroPins = 0;
    Utils::AreaDBU cellArea = 0;
    Utils::AreaDBU l1BlockageArea = 0;
    Utils::AreaDBU l2BlockageArea = 0;
    Utils::AreaDBU l1PinArea = 0;
    Utils::AreaDBU l2PinArea = 0;
    Utils::AreaDBU macroArea = 0;
    Utils::AreaDBU macroPinArea = 0;
    //Global Routing Features
    int horizontal_capacity = 0;
    int vertical_capacity = 0;
    int horizontal_demand = 0;
    int vertical_demand = 0;
    //labels
    bool violation = false;
    int numPins = 0;
    int numNeighborPins = 0;
    std::vector<int> h_cap, v_cap, h_demand, v_demand;
    std::unordered_set<DRVType> drvs;
};
}
