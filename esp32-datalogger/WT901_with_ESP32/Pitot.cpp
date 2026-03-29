
#include "Pitot.h"

TwoWire* p_Pitot = nullptr;

void setupPitot(TwoWire* ptr){
  p_Pitot = ptr;
}