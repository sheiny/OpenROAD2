#pragma once

namespace ftx{
class FeatureExtractor;
}

namespace ord {

class OpenRoad;

ftx::FeatureExtractor *makeFeatureExtractor();

void initFeatureExtractor(OpenRoad *openroad);

void deleteFeatureExtractor(ftx::FeatureExtractor *feature_extractor);

}  // namespace ord
