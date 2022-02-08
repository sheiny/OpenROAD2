#pragma once

namespace ftx{
class FeatureExtractor;
}

namespace odb{
class dbDatabase;
}

namespace ord {

class OpenRoad;

ftx::FeatureExtractor *makeFeatureExtractor(odb::dbDatabase *db);

void initFeatureExtractor(OpenRoad *openroad);

void deleteFeatureExtractor(ftx::FeatureExtractor *feature_extractor);

}  // namespace ord
