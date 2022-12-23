include("openroad")

project(ftx)

swig_lib(NAME      ftx
         NAMESPACE ftx
         I_FILE    src/FeatureExtractor.i
         SCRIPTS   src/FeatureExtractor.tcl
)

target_sources(ftx
  PRIVATE
    src/FeatureExtractor.cpp
    src/GridGraph.cpp
    src/MakeFeatureExtractor.cpp
    src/Node.cpp
    src/RptParser.cpp
    src/Utils.cpp
)

target_include_directories(ftx
  PUBLIC
    include
)

target_link_libraries(ftx
    gui
    OpenSTA
    stt
    utl
    Boost::boost
)

messages(
  TARGET ftx
)
