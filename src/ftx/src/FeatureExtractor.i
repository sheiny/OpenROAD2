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

%include "../../Exception.i"

%inline %{

namespace ftx {

void
feature_extract_init_cmd()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  ord::OpenRoad *openroad = ord::getOpenRoad();
  featureExt->init(openroad->getDb(), nullptr);
}

void
feature_extract_init_graph_cmd()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  ord::OpenRoad *openroad = ord::getOpenRoad();
  featureExt->initGraph(openroad->getDb());
}

void
feature_extract_init_graph_from_GCells_cmd()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  ord::OpenRoad *openroad = ord::getOpenRoad();
  featureExt->initGraphFromDef(openroad->getDb());
}

void
feature_extract_read_congestion_cmd(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->readCongestion(fileName);
}

void
feature_extract_run_cmd()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->extractFeatures();
}

void
feature_extract_write_csv_cmd(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->writeCSV(fileName);
}

void
feature_extract_read_rpt_cmd(char* fileName, bool triton)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  ord::OpenRoad *openroad = ord::getOpenRoad();
  auto db_ptr = openroad->getDb();
  auto lib_ptr = db_ptr->getLibs().begin();
  featureExt->readRPT(fileName, *lib_ptr, triton);
}

void
feature_extract_calculate_ABU_cmd()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->calculateABU();
}

void
feature_extract_Draw_DRVs_cmd()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->drawDRVs();
}

void
feature_extract_Paint_Nodes_cmd(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->paintNodes(fileName);
}

} // namespace

%} // inline
