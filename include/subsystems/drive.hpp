#include "main.h"

namespace drive
{
    void init();
    void opcontrol();

    void drive(int power);
    void drive(int degrees, int power);
    void turn(double power);
    void turn(double degrees, int power);
}

namespace auton
{
    void aut();
    void skills();
    void redRight();
    void redRight2();
    void rightOneGoal();
    void leftRing();

    void claw_open(bool b);
    void deploy_claw_open(bool b);
    void backLift_down();
    void backLift_up();
    void frontLift_up(bool up);
    void frontLift_up_higher(bool up);
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

    void resetMotorEncoders();
    double avgMotorEncoders();

    void stop();
    void stop(double seconds);

    void drivePID(int units);
    void turnPID(double deg);

    void drivePIDwithClaw(int units);

    // test
    void testRotate();

}