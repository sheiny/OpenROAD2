#pragma once

#include "Types.h"
#include "odb/geom.h"

#include <iosfwd>
#include <unordered_set>

namespace ftx {

struct Node {
    Node(vertexIndex node_id, odb::Rect rect);

    std::string printPlacementFeatures(std::string separator, bool debug);

    std::string printDRVs();

    //Graph attributes
    vertexIndex nodeID;
    odb::Rect rect;
    //Placement Features
    int numCells, numCellPins, numMacros, numMacroPins;
    Utils::AreaDBU cellArea, l1BlockageArea, l2BlockageArea,
                   l1PinArea, l2PinArea, macroArea, macroPinArea;
    //Global Routing Features
    int horizontal_capacity, vertical_capacity,
        horizontal_demand, vertical_demand;
    //labels
    bool violation;
    int numPins, numNeighborPins;
    std::vector<int> h_cap, v_cap, h_demand, v_demand;
    std::unordered_set<DRVType> drvs;
};
}
