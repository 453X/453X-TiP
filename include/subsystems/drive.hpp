#include "main.h"

namespace drive
{
    void init();
    void opcontrol();

    void move(int power);
    void turn(double power);
}

namespace auton
{
    void redRight();
    void claw_open(bool b);
    void backLift_down();
    void backLift_up();
    void roller_on();
    void roller_off();
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