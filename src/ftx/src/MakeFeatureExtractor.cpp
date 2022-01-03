#include "ftx/MakeFeatureExtractor.h"
#include "ftx/FeatureExtractor.h"
#include "ord/OpenRoad.hh"
#include "sta/StaMain.hh"

namespace sta {
// Tcl files encoded into strings.
extern const char* ftx_tcl_inits[];
}  // namespace sta

//Rule: FeatureExtractor class -> Featureextractor_Init
//So, the module name in .i have to equal the Class name,
//although .i is considered case insensitive.
extern "C" {
extern int Ftx_Init(Tcl_Interp* interp);
}

namespace ord {

ftx::FeatureExtractor * makeFeatureExtractor()
{
  return new ftx::FeatureExtractor;
}

//This funcion will bind the calls between .tcl and .i files
void
initFeatureExtractor(OpenRoad *openroad)
{
  Tcl_Interp* tcl_interp = openroad->tclInterp();
  // Define swig TCL commands.
  Ftx_Init(tcl_interp);
  sta::evalTclInit(tcl_interp, sta::ftx_tcl_inits);
}

void
deleteFeatureExtractor(ftx::FeatureExtractor *feature_extractor)
{
  delete feature_extractor;
}

}
