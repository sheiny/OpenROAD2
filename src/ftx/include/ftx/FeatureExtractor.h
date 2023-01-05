#pragma once

#include "Types.h"
#include <iosfwd>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace odb {
  class dbDatabase;
  class dbInst;
  class dbLib;
  class dbMaster;
  class dbNet;
  class dbTransform;
  class Rect;
}

namespace stt {
  class SteinerTreeBuilder;
}

namespace utl {
  class Logger;
}

namespace ftx {
class DRVRenderer;
class GridGraph;
class GridRender;
struct Node;
class NodePainter;
class RectRender;

class FeatureExtractor {
  public:
    FeatureExtractor();
    ~FeatureExtractor();

    //Init GridGraph using a multiple of RowHeight (considers core area only).
    void initGraph(int sizeInRowHeights = 3);

    //Init GridGraph using DEF GCells description.
    void initGraphFromDef();

    void saveLocations(std::string file_path);

    void loadLocations(std::string file_path);

    void readRPT(std::string file_path,
                 odb::dbLib* lib);

    void readGuide(std::string file_path);

    void extractFeatures();

    void extractCNNFeatures();

    double calculateDensity();

    //Ignore nodes completely overlapped by macros!
    void calculateABU();

    void writeCSV(std::string file_path, int distance);

    void writeCNNCSVs(std::string file_path, int distance);

    void writeCNNCSV(std::string file_path, int distance);

    void drawDRVs();

    void drawGrid();

    void paintNodes(std::string file_path);

    void paintNode(unsigned int id);

  private:
    std::string nodeHyperImage(Node* node, const std::vector<Node*>& neighbors);

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
    //Consider routing cap from Metal1 up to maxRoutingLevel
    void initRoutingCapacity();
    void extractRoutingFeatures(odb::dbNet* net);

    odb::dbDatabase *db_;
    std::unique_ptr<GridGraph> gridGraph_;
    stt::SteinerTreeBuilder *stt_;
    utl::Logger *logger_;
    std::unique_ptr<DRVRenderer> drvRenderer_;
    std::unique_ptr<GridRender> gridRenderer_;
    std::unique_ptr<NodePainter> nodePainter_;
    std::unique_ptr<RectRender> rectRender_;
  };
}

