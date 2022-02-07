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
    void rightOneGoal2();
    void leftGoal();
    void leftRing();
    void singleAWP();

    void claw_open(bool b);
    void claw_open(bool b, double d);
    void backLift_down();
    void backLift_low();
    void backLift_up();
    void backLift_up_higher();
    void frontLift_up(bool up);
    void frontLift_up_higher(bool up);
    void frontLift_down();
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
    void drivePID(int maxPower, int units);
    void turnPID(double deg);
    void distancePID(int, bool direction);

    double correctionDegrees(double heading, double setPoint);

    void drivePIDwithClaw(int units);
    void driveTurnAssist(int units, int power);

}