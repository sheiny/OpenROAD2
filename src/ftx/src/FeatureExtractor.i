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
saveLocations(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->saveLocations(fileName);
}

void
loadLocations(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->loadLocations(fileName);
}

void
initGraph(int sizeInRowHeights = 3)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->initGraph(sizeInRowHeights);
}

void
initGraphFromDef(int paddingSize=0)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->initGraphFromDef(paddingSize);
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
writeCSV(char* fileName, int distance=1)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->writeCSV(fileName, distance);
}

void
writeCNNCSVFiles(char* fileName, int distance=16)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->writeCNNCSVs(fileName, distance);
}

void
writeCNNCSV(char* fileName, int distance=16)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->writeCNNCSV(fileName, distance);
}

void
readRPT(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  ord::OpenRoad *openroad = ord::getOpenRoad();
  auto db_ptr = openroad->getDb();
  auto lib_ptr = db_ptr->getLibs().begin();
  featureExt->readRPT(fileName, *lib_ptr);
}

void
readGuide(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->readGuide(fileName);
}

void
DrawDRVs()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->drawDRVs();
}

void
DrawGrid()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->drawGrid();
}

void
PaintNodes(char* fileName)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->paintNodes(fileName);
}

void
PaintNode(int id)
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->paintNode(id);
}

void
SanityCheck()
{
  FeatureExtractor* featureExt = getFeatureExtractor();
  featureExt->sanityCheck();
}

} // namespace

%} // inline
