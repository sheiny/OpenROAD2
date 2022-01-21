#pragma once

#include "Types.h"
#include <string>
#include <iosfwd>

#include <memory>

namespace odb {
  class dbDatabase;
  class dbInst;
  class dbLib;
  class dbMaster;
  class dbNet;
  class dbTransform;
  class Rect;
}

namespace ftx {
class DRVRenderer;
class GridGraph;
class GridRender;
class Node;
class NodePainter;

class FeatureExtractor {
  public:
    FeatureExtractor();
    ~FeatureExtractor();

    void initGraph(odb::dbDatabase* db);

    void initGraphFromDef(odb::dbDatabase* db);

    void readRPT(std::string file_path,
                 odb::dbLib* lib,
                 bool triton=false);

    void readCongestion(std::string file_path);
    void readCongestion(std::istream & isstream);

    void init(odb::dbDatabase* db, GridGraph* graph);
    void clear();
    void extractFeatures();

    double calculateDensity();

    //Ignore nodes completely overlapped by macros!
    void calculateABU();

    void writeCSV(std::string file_path);

    void drawDRVs();

    void drawGrid();

    void paintNodes(std::string file_path);
  private:
    void extractInstFeatures(odb::dbInst* inst);
    void extractCellFeatures(odb::dbInst* inst, odb::Rect bbox_inst,
                             odb::dbMaster* master, Node* node,
                             odb::dbTransform transform);
    void extractCellBlockages(odb::dbMaster* master, Node* node,
                              odb::dbTransform transform);
    void extractCellPins(odb::dbMaster* master, Node* node,
                         odb::dbTransform transform);
    void extractMacroFeatures(odb::dbInst* inst, odb::dbMaster* master,
                              Node* node, odb::dbTransform transform);
    void extractMacroPins(odb::dbMaster* master, Node* node,
                          odb::dbTransform transform);
    void extractPassingNets(odb::dbNet* net);

    odb::dbDatabase* db_;
    ftx::GridGraph * gridGraph_;
    std::unique_ptr<DRVRenderer> drvRenderer_;
    std::unique_ptr<GridRender> gridRenderer_;
    std::unique_ptr<NodePainter> nodePainter_;
  };
}

