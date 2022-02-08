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
init_graph()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->initGraph();
}

void
init_graph_from_GCells()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->initGraphFromDef();
}

void
read_congestion(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->readCongestion(fileName);
}

void
run()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->extractFeatures();
}

void
write_csv(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->writeCSV(fileName);
}

void
read_rpt(char* fileName, bool triton)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  ord::OpenRoad *openroad = ord::getOpenRoad();
  auto db_ptr = openroad->getDb();
  auto lib_ptr = db_ptr->getLibs().begin();
  featureExt->readRPT(fileName, *lib_ptr, triton);
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

} // namespace

%} // inline
