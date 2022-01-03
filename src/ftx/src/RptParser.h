#pragma once

#include "Types.h"

#include <vector>
#include <string>
#include <iosfwd>

namespace odb {
  class Rect;
  class dbLib;
}

namespace ftx {
class RPTParser{
  public:
    std::vector<std::pair<odb::Rect, DRVType>>
    parseDRVs(std::string file_path, odb::dbLib * lib);

    std::vector<std::pair<odb::Rect, DRVType>>
    parseDRVs(std::istream & isstream, odb::dbLib * lib);

    std::vector<odb::Rect> parseTritonDRVs(std::string file_path,
                                                      odb::dbLib * lib);
    std::vector<odb::Rect> parseTritonDRVs(std::istream & isstream,
                                                      odb::dbLib * lib);
};
}
