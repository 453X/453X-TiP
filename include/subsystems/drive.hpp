#include "main.h"

namespace drive
{
    void init();
    void opcontrol();
}

namespace auton
{
    void redLeft();
    void clawOpen(bool b);
}

namespace pid
{
    void delaySeconds(double seconds);

    void calibrate();
    void inertialReset();

    void resetDriveEncoders();
    double avgDriveEncoders();

    void stop();
    void stop(double seconds);
    void move(int power);
    void turn(double power);

    void forwardPD(int units);
    void rotateDegreesPD(double deg);

    // test 
    void testRotate();

}