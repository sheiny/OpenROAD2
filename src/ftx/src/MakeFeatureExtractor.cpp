#include "ftx/MakeFeatureExtractor.h"
#include "ftx/FeatureExtractor.h"
#include "ord/OpenRoad.hh"
#include "sta/StaMain.hh"

namespace sta {
// Tcl files encoded into strings.
extern const char* ftx_tcl_inits[];
}  // namespace sta

// TODO: I believe that the name that goes here is the project/namespace/swig_lib name
// where the first letter is Capital folowed by "_Init"
extern "C" {
extern int Ftx_Init(Tcl_Interp *interp);
}

namespace ord {

ftx::FeatureExtractor *makeFeatureExtractor(odb::dbDatabase *db)
{
  return new ftx::FeatureExtractor{db};
}

//This funcion will bind the calls between .tcl and .i files
void
initFeatureExtractor(OpenRoad *openroad)
{
  Tcl_Interp *tcl_interp = openroad->tclInterp();
  // Define swig TCL commands function from Line 14
  Ftx_Init(tcl_interp);
  sta::evalTclInit(tcl_interp, sta::ftx_tcl_inits);
}

void
deleteFeatureExtractor(ftx::FeatureExtractor *feature_extractor)
{
  delete feature_extractor;
}

}
