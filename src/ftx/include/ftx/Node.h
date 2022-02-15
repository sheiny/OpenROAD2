#pragma once

#include "Types.h"
#include "odb/geom.h"

#include <iosfwd>
#include <unordered_set>

namespace ftx {

class Node {
  public:
    Node(vertexIndex node_id, odb::Rect rect);
    ~Node();

    //Modifiers
    void insertFeatureCount(FeatureCount feature, int amount);
    void insertFeatureArea(FeatureArea feature, Utils::AreaDBU area);
    void setViolation(bool violation);
    void insertDRVType(DRVType drv);

    //Getters
    bool hasViolation();
    std::unordered_set<DRVType> getDRVs();
    int getFeatureCount(FeatureCount feature);
    Utils::AreaDBU getFeatureArea(FeatureArea feature);
    odb::Rect rect();
    vertexIndex id();

    friend std::ostream& operator<<(std::ostream& os, Node& node);
  private:
    //Placement Features
    int numCells_, numCellPins_, numMacros_, numMacroPins_,
        numPassingNets_;
    Utils::AreaDBU cellArea_, l1BlockageArea_, l2BlockageArea_,
                   l1PinArea_, l2PinArea_, macroArea_, macroPinArea_;
    //Global Routing Features
    int vertical_overflow_, vertical_remain_, vertical_tracks_,
        horizontal_overflow_, horizontal_remain_, horizontal_tracks_;
    bool violation_;
    std::unordered_set<DRVType> drvs;
    vertexIndex nodeID_;
    odb::Rect rect_;
};
}
