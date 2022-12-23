%{
#include "odb/db.h"
#include "ord/OpenRoad.hh"
#include "ftx/FeatureExtractor.h"

namespace ord {
// Defined in OpenRoad.i
ftx::FeatureExtractor* getFeatureExtractor();
// Defined in OpenRoad.i
OpenRoad *getOpenRoad();
}  // namespace ord

using ord::getFeatureExtractor;
using ftx::FeatureExtractor;

%}

%inline %{

namespace ftx {

void
save_locations(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->saveLocations(fileName);
}

void
load_locations(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->loadLocations(fileName);
}

void
init_graph(int sizeInRowHeights = 3)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->initGraph(sizeInRowHeights);
}

void
init_graph_from_GCells()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->initGraphFromDef();
}

void
run()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->extractFeatures();
}

void
runCNN()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->extractCNNFeatures();
}

void
write_csv(char* fileName, int distance=1)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->writeCSV(fileName, distance);
}

void
write_cnn_csv(char* fileName, int distance=16)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->writeCNNCSV(fileName, distance);
}

void
read_rpt(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  ord::OpenRoad *openroad = ord::getOpenRoad();
  auto db_ptr = openroad->getDb();
  auto lib_ptr = db_ptr->getLibs().begin();
  featureExt->readRPT(fileName, *lib_ptr);
}

void
read_guide(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->readGuide(fileName);
}

void
calculate_ABU()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->calculateABU();
}

void
Draw_DRVs()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->drawDRVs();
}

void
Draw_Grid()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->drawGrid();
}

void
Paint_Nodes(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->paintNodes(fileName);
}

void
Paint_Node(int id)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->paintNode(id);
}

} // namespace

%} // inline
