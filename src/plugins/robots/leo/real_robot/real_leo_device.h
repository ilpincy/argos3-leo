#ifndef REAL_LEO_DEVICE_H
#define REAL_LEO_DEVICE_H

#include <argos3/core/utility/datatypes/datatypes.h>

using namespace argos;

class CRealLeoDevice {

public:

   virtual ~CRealLeoDevice() {}

   virtual void Do(Real f_elapsed_time) = 0;

};

#endif // REAL_LEO_DEVICE_H
