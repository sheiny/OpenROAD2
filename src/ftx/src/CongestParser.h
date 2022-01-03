#pragma once

#include "odb/geom.h"

#include <iosfwd>
#include <string>

namespace odb {
  class Rect;
}

namespace ftx {
struct Congestion {
  odb::Rect rect;
  int v_overflow, v_remain, v_tracks,
      h_overflow, h_remain, h_tracks;
  Congestion(odb::Rect r, int v_o, int v_r, int v_t,
                          int h_o, int h_r, int h_t);
};

class CongestParser{
  public:
    std::vector<Congestion> parseCongestion(std::string file_path);
    std::vector<Congestion> parseCongestion(std::istream & isstream);
};
}
