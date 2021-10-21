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
    void redLeft();
}

namespace pid
{
    void delaySeconds(double seconds);
    
    void calibrate();
    void inertialReset();

    void resetDriveEncoders();
    double avgDriveEncoders();

    void stop();
    void stop (double seconds);
    




    void drive(int units);
    void rotateDegreesPD(double deg);

}