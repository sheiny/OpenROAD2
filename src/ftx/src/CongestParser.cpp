#include "CongestParser.h"

#include <vector>
#include <fstream>
#include <istream>
#include <sstream>

namespace ftx {

Congestion::Congestion(odb::Rect r, int v_o, int v_r, int v_t,
                       int h_o, int h_r, int h_t):
  rect{r}, v_overflow{v_o}, v_remain{v_r}, v_tracks{v_t},
  h_overflow{h_o}, h_remain{h_r}, h_tracks{h_t}
{
}

std::vector<Congestion>
CongestParser::parseCongestion(std::string file_path)
{
  std::ifstream file(file_path);
  return parseCongestion(file);
}

std::vector<Congestion>
CongestParser::parseCongestion(std::istream & isstream)
{
  std::vector<Congestion> result;
  std::string line;
  for(auto i = 0; i <= 10; ++i)//skip comments
    std::getline(isstream, line);
  while(std::getline(isstream, line))
  {
    std::string word;
    std::istringstream iss{line};
    while(iss >> word)
    {
      int x1 = std::stoi(word.substr(1, word.size()-2));
      iss >> word;
      int y1 = std::stoi(word.substr(0, word.size()-1));
      iss >> word;
      int x2 = std::stoi(word.substr(1, word.size()-2));
      iss >> word;
      int y2 = std::stoi(word.substr(0, word.size()-1));
      iss >> word;
      iss >> word;
      auto v_o = std::stoi(word.substr(0, word.find('/')));
      word.erase(0, word.find('/')+1);
      auto v_r = std::stoi(word.substr(0, word.find('/')));
      word.erase(0, word.find('/')+1);
      auto v_t = std::stoi(word);
      iss >> word;
      iss >> word;
      auto h_o = std::stoi(word.substr(0, word.find('/')));
      word.erase(0, word.find('/')+1);
      auto h_r = std::stoi(word.substr(0, word.find('/')));
      word.erase(0, word.find('/')+1);
      auto h_t = std::stoi(word);
      odb::Rect rect{x1, y1, x2, y2};
      result.push_back({rect, v_o, v_r, v_t, h_o, h_r, h_t});
    }
  }
  return result;
}
}
