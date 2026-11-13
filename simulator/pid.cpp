// Compile upstream PID_v1.cpp unchanged under the adapter's internal class name.
#define SIM_COMPILING_PID
#define PID NativePID
#include "vendor/PID_v1.cpp"
