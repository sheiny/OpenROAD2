#pragma once

#include "Types.h"
#include "odb/geom.h"

#include <iosfwd>
#include <unordered_set>

namespace ftx {

struct Node {
    Node(vertexIndex node_id, odb::Rect rect);

    std::string printPlacementFeatures(std::string separator, bool debug);

    std::string printCongestion();

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
    //CNN features
    int vertical_overflow, vertical_remain, vertical_tracks,
        horizontal_overflow, horizontal_remain, horizontal_tracks;
    //labels
    bool violation;
    std::unordered_set<DRVType> drvs;
};
}
