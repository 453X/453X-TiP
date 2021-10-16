#include "main.h"

namespace drive
{
    void init();
    void opcontrol();
}

namespace auton
{
    void redLeft();
}

namespace pid
{
    void delaySeconds(double seconds);
    void calibrate();
    void inertialReset();
    void stop();
    void stop (double seconds);
    void resetDriveEncoders();
    double avgDriveEncoders();
    void forwardPD(int units);
}