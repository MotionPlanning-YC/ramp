#ifndef REFLEXXES_DATA
#define REFLEXXES_DATA

#include <Reflexxes/TypeIIRML/ReflexxesAPI.h>
#include <Reflexxes/TypeIIRML/RMLPositionFlags.h>
#include <Reflexxes/TypeIIRML/RMLPositionInputParameters.h>
#include <Reflexxes/TypeIIRML/RMLPositionOutputParameters.h>

struct ReflexxesData {
  ReflexxesAPI *rml;
  RMLPositionInputParameters *inputParameters;          
  RMLPositionOutputParameters *outputParameters;
  RMLPositionFlags flags;
  unsigned int NUMBER_OF_DOFS;
  int resultValue;
};

#endif
