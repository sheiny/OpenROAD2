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

class FeatureExtractor {
  public:
    FeatureExtractor();
    ~FeatureExtractor();

    //Init GridGraph using a multiple of RowHeight (considers core area only).
    void initGraph(int sizeInRowHeights = 3);

    //Init GridGraph using DEF GCells description.
    void initGraphFromDef();

    //Init GridGraph using GR solution and its congestion information.
    void initGraphFromCongestion(std::string file_path);

    void saveLocations(std::string file_path);

    void loadLocations(std::string file_path);

    void readRPT(std::string file_path,
                 odb::dbLib* lib);

    void readCongestion(std::string file_path);
    void readCongestion(std::istream & isstream);

    void extractFeatures();

    void extractCNNFeatures(std::string outputPath,
                            std::string circuitName,
                            int neighborhoodSize=11);

    double calculateDensity();

    //Ignore nodes completely overlapped by macros!
    void calculateABU();

    void writeCSV(std::string file_path, int distance);

    void drawDRVs();

    void drawGrid();

    void paintNodes(std::string file_path);

    void paintNode(unsigned int id);

    void printNodeDebugInfo(unsigned int id);
  private:
    void writeCNNInputFile(std::string path,
                           std::string circuitName,
                           std::string fileName,
                           auto container,
                           int neighborhoodSize);

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
    void initRoutingCapacity();
    void extractRoutingFeatures(odb::dbNet* net);

    odb::dbDatabase *db_;
    std::unique_ptr<GridGraph> gridGraph_;
    stt::SteinerTreeBuilder *stt_;
    utl::Logger *logger_;
    std::unique_ptr<DRVRenderer> drvRenderer_;
    std::unique_ptr<GridRender> gridRenderer_;
    std::unique_ptr<NodePainter> nodePainter_;
  };
}

