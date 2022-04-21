#include "ftx/RptParser.h"
#include "odb/db.h"
#include "odb/geom.h"

#include <fstream>
#include <sstream>
#include <istream>

namespace ftx {

std::vector<std::pair<odb::Rect, DRVType>>
RPTParser::parseDRVs(std::string file_path, odb::dbLib * lib)
{
  std::ifstream file(file_path);
  return parseDRVs(file, lib);
}

std::vector<std::pair<odb::Rect, DRVType>>
RPTParser::parseDRVs(std::istream & isstream, odb::dbLib * lib)
{
  std::string line;
  std::vector<std::pair<odb::Rect, DRVType>> result;
  int dbus_per_micron = lib->getDbUnitsPerMicron();
  int x1, x2, y1, y2;
  while (std::getline(isstream, line))
  {
    if(line.find("#") != std::string::npos)
      line = line.substr(0, line.find("#"));
    if(line.find("( Metal1 )") == std::string::npos &&
       line.find("( Metal2 )") == std::string::npos &&
       line.find("( Metal3 )") == std::string::npos &&
       line.find("( metal1 )") == std::string::npos &&
       line.find("( metal2 )") == std::string::npos &&
       line.find("( metal3 )") == std::string::npos)
      continue;

    std::string drvStr = line.substr(0, line.find(")")+1);
    DRVType drvType = DRVType::undefined;
    if(drvStr == "CUTSPACING: ( Adjacent Cut Spacing )")
      drvType = DRVType::adjacentCutSpacing;
    else if(drvStr == "CUTSPACING: ( Same Layer Cut Spacing )")
      drvType = DRVType::sameLayerCutSpacing;
    else if(drvStr == "EndOfLine: ( EndOfLine Spacing )")
      drvType = DRVType::endOfLine;
    else if(drvStr == "Geometric: ( Floating Patch )")
      drvType = DRVType::floatingPatch;
    else if(drvStr == "MAR: ( Minimum Area )")
      drvType = DRVType::minArea;
    else if(drvStr == "MINWIDTH: ( Minimum Width )")
      drvType = DRVType::minWidth;
    else if(drvStr == "NSMETAL: ( Non-sufficient Metal Overlap )")
      drvType = DRVType::nonSuficientMetalOverlap;
    else if(drvStr == "SHORT: ( Cut Short )")
      drvType = DRVType::cutShort;
    else if(drvStr == "SHORT: ( Metal Short )")
      drvType = DRVType::metalShort;
    else if(drvStr == "SHORT: ( Out Of Die )")
      drvType = DRVType::outOfDieShort;
    else if(drvStr == "SPACING: ( Corner Spacing )")
      drvType = DRVType::cornerSpacing;
    else if(drvStr == "SPACING: ( ParallelRunLength Spacing )")
      drvType = DRVType::parallelRunLength;
    else
      continue;

    //Bounds : ( x1, y1 ) ( x2, y2 )
    std::getline(isstream, line);
    std::istringstream iss(line);
    std::string word;
    iss >> word;
    iss >> word;
    iss >> word;
    iss >> word;
    x1 = std::stod(word) * dbus_per_micron;
    iss >> word;
    y1 = std::stod(word) * dbus_per_micron;
    iss >> word;
    iss >> word;
    iss >> word;
    x2 = std::stod(word) * dbus_per_micron;
    iss >> word;
    y2 = std::stod(word) * dbus_per_micron;
    result.push_back({{x1, y1, x2, y2}, drvType});
  }
  return result;
}

std::vector<odb::Rect>
RPTParser::parseTritonDRVs(std::string file_path, odb::dbLib * lib)
{
  std::ifstream file(file_path);
  return parseTritonDRVs(file, lib);
}

std::vector<odb::Rect>
RPTParser::parseTritonDRVs(std::istream & isstream,
                           odb::dbLib * lib)
{
  std::string line;
  std::vector<odb::Rect> result;
  int dbus_per_micron = lib->getDbUnitsPerMicron();
  int x1, x2, y1, y2;
  while (std::getline(isstream, line))
  {
    std::istringstream iss(line);
    std::string word;
    iss >> word;
    iss >> word;
    iss >> word;
    if(word == "Short")
    {
      std::getline(isstream, line);
      std::getline(isstream, line);
      std::istringstream iss(line);
      iss >> word;
      iss >> word;
      iss >> word;
      iss >> word;
      x1 = std::stod(word) * dbus_per_micron;
      iss >> word;
      y1 = std::stod(word) * dbus_per_micron;
      iss >> word;
      iss >> word;
      iss >> word;
      iss >> word;
      x2 = std::stod(word) * dbus_per_micron;
      iss >> word;
      y2 = std::stod(word) * dbus_per_micron;
      result.push_back({x1, y1, x2, y2});
    }
    else
    {
      std::getline(isstream, line);
      std::getline(isstream, line);
    }
  }
  return result;
}
}
