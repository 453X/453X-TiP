#include "main.h"

Motor LF(-1), LB(-2), RF(3), RB(4), liftBack(5), roller(-6), lift(7), claw(-8);

ADIButton frontLimit('G');
ADIButton backLimit('H');

MotorGroup left({LF, LB});
MotorGroup right({RF, RB});

ADIEncoder leftEncoder('A', 'B', true);
ADIEncoder rightEncoder('C', 'D', false);

IMU inertial(11);
DistanceSensor dist1(14);
DistanceSensor dist2(13);

Timer timer;

bool driveInitDone = false;

// Chassis Controller - lets us drive the robot around with open- or closed-loop control
auto driveOdom = ChassisControllerBuilder()
                     //  .withMotors(-1,3)
                     .withMotors(
                         {-1, -2}, // Left motors are 1 & 2
                         {3, 4})   // Right motors are 3 & 4

                     // Blue gearset, external ratio of (36.0 / 84.0), 4 inch wheel diameter, 35.4 cm wheel track

                     .withDimensions({AbstractMotor::gearset::blue, (84.0 / 36.0)}, {{4_in, 35.4_cm}, imev5BlueTPR})

                     // track        = distance between the center of the 2 tracking wheels
                     // wheel track  = distance between the center of the 2 wheels on either side

                     .withSensors(leftEncoder, rightEncoder)
                     .build();

namespace drive
{

    void init()
    {
        LF.setBrakeMode(AbstractMotor::brakeMode::coast);
        LB.setBrakeMode(AbstractMotor::brakeMode::coast);
        RF.setBrakeMode(AbstractMotor::brakeMode::coast);
        RB.setBrakeMode(AbstractMotor::brakeMode::coast);

        claw.setBrakeMode(AbstractMotor::brakeMode::hold);
        claw.tarePosition();
        claw.setEncoderUnits(AbstractMotor::encoderUnits::degrees);

        roller.setBrakeMode(AbstractMotor::brakeMode::coast);

        liftBack.setBrakeMode(AbstractMotor::brakeMode::brake);

        left.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
        right.setEncoderUnits(AbstractMotor::encoderUnits::degrees);

        pid::resetDriveEncoders();
        driveInitDone = true;
    }

    void opcontrol()
    {

        Controller master;

        // Arcade drive with the left stick.
        driveOdom->getModel()->arcade(master.getAnalog(ControllerAnalog::rightY),
                                      master.getAnalog(ControllerAnalog::leftX));
    }

    void turn(double power)
    {
        left.moveVelocity(power);
        right.moveVelocity(-power);
    }

    void turn(int degrees, int power)
    {
        left.moveRelative(degrees, power);
        right.moveRelative(degrees, power);
    }

    void drive(int power)
    {
        left.moveVelocity(power);
        right.moveVelocity(power);
    }

    void drive(int degrees, int power)
    {
        pid::resetDriveEncoders();
        while (fabs(pid::avgDriveEncoders()) < degrees)
        {
            drive(power);
        }
        pid::stop();
    }

}

namespace auton
{
    void skills()
    {

        // initial
        pros::lcd::initialize();
        pid::resetDriveEncoders();
        claw.tarePosition();
        lift.tarePosition();

        // pid::turnPID(90);
        // pid::delaySeconds(3);
        // pid::turnPID(0);
        // pid::delaySeconds(3);
        // pid::turnPID(90);
        // pid::delaySeconds(20);
        
        backLift_down();
        pid::delaySeconds(0.8);
        pid::drivePID(-400);
        backLift_low();
        pid::delaySeconds(2);
        pid::turnPID(0);
        pid::delaySeconds(0.5);

        // pid::distancePID(1000, false);
        pid::drivePID(-3100);
        // pid::delaySeconds(100);

        // drive::drive(3500, -300);
        // pid::drivePID(-3500);

        pid::delaySeconds(0.3);

        backLift_down();
        pid::delaySeconds(0.5);
        // pid::turnPID(27);
        pid::turnPID(30);
        pid::drivePID(1300);
        pid::drivePID(200, 300);
        claw_open(false);
        pid::delaySeconds(2.0);
        frontLift_up_higher(true);
        pid::turnPID(37);
        pid::drivePID(2100);
        pid::delaySeconds(0.5);
        pid::turnPID(0);
        // frontLift_up(false);
        pid::delaySeconds(0.5);
        pid::drivePID(200, 150);
        auton::claw_open(true);
        // backLift_down();

        pid::delaySeconds(0.5);
        pid::turnPID(0);

        //back up to tall neutral goal

        pid::drivePID(-1300);
        pid::delaySeconds(0.3);
        pid::turnPID(-40);
        // backLift_low();

        //push tall goal to red home zone
        // pid::distancePID(800, false);
        pid::drivePID(-2800);
        pid::delaySeconds(0.5);

        frontLift_down();
        backLift_down();
        pid::delaySeconds(0.3);
        pid::drivePID(600);
        pid::turnPID(90);
        pid::delaySeconds(0.2);
        
        // pid::drivePID(1100);
        pid::distancePID(600, true);

        auton::claw_open(false);
        pid::delaySeconds(1.3);

        pid::turnPID(100);
        pid::delaySeconds(5);
        pid::drivePID(-1600);
        pid::turnPID(210);
        pid::drivePID(-2800);
        pid::drivePID(500);

        pid::delaySeconds(5);
        pid::turnPID(270);
        pid::delaySeconds(3);
        frontLift_up_higher(true);
        pid::drivePID(2200);
        pid::turnPID(0);
        // drive::drive(120, 100);

        pid::drivePID(1100);
        claw_open(true);


    }

    void leftRing()
    {
        pros::lcd::initialize();
        pid::resetDriveEncoders();
        claw.tarePosition();

        auton::backLift_down();
        drive::drive(500, -150);
        pid::delaySeconds(1.0);
        backLift_up();

        pid::delaySeconds(1.0);

        frontLift_up(true);

        auton::roller_on();

        drive::drive(1500, 50);
        pid::stop();

        pid::delaySeconds(1.0);

        backLift_down();
    }

    void rightOneGoal()
    {
        pid::inertialReset();

        pros::lcd::initialize();
        pid::resetDriveEncoders();
        claw.tarePosition();
        lift.tarePosition();

        // move and grab
        pid::drivePID(1750);

        // drive::drive(1700, 600);

        claw_open(false);
        pid::delaySeconds(1.5);

        // lift yellow goal
        frontLift_up(true);
        pid::delaySeconds(0.5);

        // back
        pid::drivePID(-1100);

        // release yellow goal
        pid::turnPID(-90);
        drive::drive(100, 100);
        frontLift_up(false);
        claw_open(true);
        pid::delaySeconds(0.1);

        // back and lift red goal
        backLift_down();
        pid::drivePID(-550);
        drive::drive(-300, 100);
        backLift_up();
        pid::delaySeconds(0.5);
        frontLift_up(true);
        pid::drivePID(50);
        auton::roller_on();
        pid::turnPID(180);
        pid::delaySeconds(0.4);
        drive::drive(700, 70);
        pid::delaySeconds(0.5);
        backLift_down();
        drive::drive(100, 200);
        // drive::drive(500, -200);
        // pid::delaySeconds(1.0);
        // drive::drive(500, 100);
        // drive::drive(-200, 1000);

        pid::stop();
    }

    void leftGoal()
    {
        pros::lcd::initialize();
        pid::resetDriveEncoders();
        claw.tarePosition();
        lift.tarePosition();

        // move and grab
        pid::drivePIDwithClaw(1900);
        // claw_open(false);
        pid::delaySeconds(0.8);

        // lift yellow goal
        frontLift_up(true);
        pid::delaySeconds(0.5);

        // back
        pid::drivePID(-1200);

        // release yellow goal
        pid::turnPID(180);
        drive::drive(100, 100);
        frontLift_up(false);
        claw_open(true);
        pid::delaySeconds(0.1);
    }

    void redRight()
    {
        // pros::lcd::initialize();
        // pid::resetDriveEncoders();

        pros::lcd::initialize();
        pid::resetDriveEncoders();
        claw.tarePosition();

        // move and grab
        pid::drivePID(1750);
        claw_open(false);
        pid::delaySeconds(1.5);

        // lift yellow goal
        frontLift_up(true);
        pid::delaySeconds(0.5);

        // back
        pid::drivePID(-1000);

        // release yellow goal
        pid::turnPID(-90);
        frontLift_up(false);
        claw_open(true);
        pid::delaySeconds(0.1);

        // back and lift red goal
        backLift_down();
        pid::drivePID(-500);
        backLift_up();
        frontLift_up(true);
        pid::drivePID(50);
        auton::roller_on();
        pid::turnPID(180);
        pid::drivePID(500);
        pid::drivePID(-500);

        pid::stop();
    }

    void redRight2()
    {
        pros::lcd::initialize();
        pid::resetDriveEncoders();

        backLift_down();
        pid::stop(0.5);
        pid::drivePID(-1000);
        backLift_up();
        frontLift_up(true);
        roller_on();
        pid::drivePID(800);
        pid::drivePID(-800);
    }

    void redBack()
    {
    }

    void deploy_claw_open(bool open)
    {
        if (open)
        {
            int err = claw.moveAbsolute(0, 100);
            pros::lcd::print(6, "claw  open>> %5.2f  err:%d", claw.getPosition(), err);
        }
        else
        {
            int err = claw.moveAbsolute(300, 100);
            // int err = claw.moveVoltage(2000);
            pros::lcd::print(7, "claw close>> %5.2f  err:%d", claw.getPosition(), err);
        }
    }

    void claw_open(bool open)
    {
        if (open)
        {
            int err = claw.moveAbsolute(200, 100);
            pros::lcd::print(6, "claw  open>> %5.2f  err:%d", claw.getPosition(), err);
        }
        else
        {
            int err = claw.moveAbsolute(900, 100);
            // int err = claw.moveVoltage(2000);
            pros::lcd::print(7, "claw close>> %5.2f  err:%d", claw.getPosition(), err);
        }
    }

    void claw_open(bool open, int power)
    {
        if (open)
        {
            int err = claw.moveAbsolute(200, power);
            pros::lcd::print(6, "claw  open>> %5.2f  err:%d", claw.getPosition(), err);
        }
        else
        {
            int err = claw.moveAbsolute(800, power);
            // int err = claw.moveVoltage(2000);
            pros::lcd::print(7, "claw close>> %5.2f  err:%d", claw.getPosition(), err);
        }
    }

    void backLift_down()
    {
        while (!frontLimit.isPressed())
        {
            liftBack.moveVoltage(12000);
        }
        liftBack.moveVelocity(0);
        liftBack.tarePosition();
    }

    void backLift_low()
    {
        liftBack.moveRelative(-600, 100);
    }
    void backLift_up()
    {
        liftBack.moveRelative(-1400, 100);
    }

    void frontLift_up(bool up)
    {
        int pos = 900;
        if (up)
        {
            lift.moveAbsolute(pos, 127);
        }
        else
        {
            lift.moveAbsolute(0, 127);
        }
    }

    void frontLift_up_higher(bool up)
    {
        int pos = 2600;
        if (up)
        {
            lift.moveAbsolute(pos, 127);
        }
        else
        {
            lift.moveAbsolute(0, 127);
        }
    }

    void frontLift_down()
    {
        lift.moveAbsolute(0, 127);
    }

    void roller_on()
    {
        roller.moveVoltage(12000);
    }
    void roller_off()
    {
        roller.moveVelocity(0);
    }

}

namespace pid
{
    void delaySeconds(double seconds) { pros::delay(seconds * 1000); }

    void calibrate()
    {
        pros::lcd::initialize();
        pros::lcd::print(0, "calibrate : initialize (done)  ");

        int err = inertial.reset();
        pros::lcd::print(0, "calibrate : reset (done)  %d", err);
        int n = 0;
        while (inertial.isCalibrating())
        {
            pros::lcd::print(0, "calibrate : is_calibrating : %d", n++);

            pros::delay(20);
        }
        pros::lcd::print(0, "calibrate : is_calibrating (done)  ");

        delaySeconds(0.5);
    }

    void inertialReset()
    {
        inertial.reset();
    }

    void stop()
    {
        left.moveVelocity(0);
        right.moveVelocity(0);
    }

    void stop(double seconds)
    {
        left.moveVelocity(0);
        right.moveVelocity(0);
        pros::delay(seconds * 1000);
    }

    void resetDriveEncoders()
    {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    void resetMotorEncoders()
    {
        left.tarePosition();
        right.tarePosition();
    }

    double avgDriveEncoders() { return (fabs(leftEncoder.get()) + fabs(rightEncoder.get())) / 2; }

    double avgMotorEncoders()
    {
        return (fabs(left.getPosition()) + fabs(right.getPosition())) / 2;
    }

    void drivePIDwithClaw(int units)
    {
        int setpoint = 1000;
        auton::claw_open(true);
        if (units > setpoint)
        {
            //pid::driveTurnAssist(units - setpoint, 600);
            auton::claw_open(false);
            drivePID(setpoint);
        }
        else
        {
            drivePID(units);
            auton::claw_open(false);
        }
    }

    void drivePID(int units)
    { // power in positive, units in positive or negative
        resetDriveEncoders();
        int direction = abs(units) / units;
        double rotation = inertial.get();
        int power = 0;
        int tune = 0;
        int setPoint = abs(units);

        double kP = 0.8;
        double kI = 0.01;
        double kD = 0.35;

        double kP_angular = 3.0;
        bool turnRight;

        double errorSum = 0;
        int fr = 0;
        int fl = 0;

        int initHeading = inertial.get();

        while (avgDriveEncoders() < abs(units))
        {
            double tolerance = 0.5;
            double error = setPoint - avgDriveEncoders();
            double angularError = initHeading - inertial.get();

            if (error < 100)
                errorSum += error;
            double prevError = 0;
            double derivative;

            if (angularError > 0)
            {
                if (angularError > 180)
                {
                    // left
                    angularError = 360 - angularError;
                    turnRight = false;
                }
                else
                {
                    // right
                    turnRight = true;
                }
            }
            else
            {
                if (angularError < -180)
                {
                    // right
                    angularError = 360 + angularError;
                    turnRight = true;
                }
                else
                {
                    // left
                    angularError = angularError * -1;
                    turnRight = false;
                }
            }

            if (direction * (error * kP + derivative * kD + errorSum * kI) >= 500)
            {
                power = 500;
            }
            else if(direction * (error * kP + derivative * kD + errorSum * kI) <= -500)
            {
                power = -500;
            }
            else
            {
                power = direction * (error * kP + derivative * kD + errorSum * kI);
            }

            if (turnRight)
            {
                tune = angularError * kP_angular;
            }
            else
            {
                tune = angularError * kP_angular * -1;
            }

            if (tune > 100)
            {
                tune = 100;
            }
            else if (tune < -100)
            {
                tune = -100;
            }

            // pros::lcd::print(0, "Get encoder  >> %f\n",
            // fabs(driveLF.get_position()));
            pros::lcd::print(0, "rotation  >> %5.2f", inertial.get());
            pros::lcd::print(1, "encoder value  >> %5.2f", avgDriveEncoders());
            pros::lcd::print(2, "error   >> %5.2f", error);
            pros::lcd::print(3, "fl,fr >> %3d , %3d", fl, fr);
            pros::lcd::print(4, "tune >> %3d", tune);

            derivative = error - prevError;
            prevError = error;

            //===============================================

            float rRate = 1.0f;
            float lRate = 1.0f;

            left.moveVelocity(power * lRate + tune);
            right.moveVelocity(power * rRate - tune);

            pros::delay(10);
        }
        stop(0);
    }

    void drivePID(int maxPower, int units)
    { // power in positive, units in positive or negative
        resetDriveEncoders();
        int direction = abs(units) / units;
        double rotation = inertial.get();
        int power = 0;
        int tune = 0;
        int setPoint = abs(units);

        double kP = 0.8;
        double kI = 0.01;
        double kD = 0.35;

        double kP_angular = 3.0;
        bool turnRight;

        double errorSum = 0;
        int fr = 0;
        int fl = 0;

        int initHeading = inertial.get();

        while (avgDriveEncoders() < abs(units))
        {
            double tolerance = 0.5;
            double error = setPoint - avgDriveEncoders();
            double angularError = initHeading - inertial.get();

            if (error < 100)
                errorSum += error;
            double prevError = 0;
            double derivative;

            if (angularError > 0)
            {
                if (angularError > 180)
                {
                    // left
                    angularError = 360 - angularError;
                    turnRight = false;
                }
                else
                {
                    // right
                    turnRight = true;
                }
            }
            else
            {
                if (angularError < -180)
                {
                    // right
                    angularError = 360 + angularError;
                    turnRight = true;
                }
                else
                {
                    // left
                    angularError = angularError * -1;
                    turnRight = false;
                }
            }

            if (direction * (error * kP + derivative * kD + errorSum * kI) >= maxPower)
            {
                power = maxPower;
            }
            else
            {
                power = direction * (error * kP + derivative * kD + errorSum * kI);
            }

            if (turnRight)
            {
                tune = angularError * kP_angular;
            }
            else
            {
                tune = angularError * kP_angular * -1;
            }

            if (tune > 100)
            {
                tune = 100;
            }

            // pros::lcd::print(0, "Get encoder  >> %f\n",
            // fabs(driveLF.get_position()));
            pros::lcd::print(0, "rotation  >> %5.2f", inertial.get());
            pros::lcd::print(1, "encoder value  >> %5.2f", avgDriveEncoders());
            pros::lcd::print(2, "error   >> %5.2f", error);
            pros::lcd::print(3, "fl,fr >> %3d , %3d", fl, fr);

            derivative = error - prevError;
            prevError = error;

            //===============================================

            float rRate = 1.0f;
            float lRate = 1.0f;

            left.moveVelocity(power * lRate + tune);
            right.moveVelocity(power * rRate - tune);

            pros::delay(10);
        }
        stop(0);
    }

    void turnPID(double deg)
    {
        double tolerance = 0.5;
        double bias = 0;
        double minPow = 50;

        double derivative;
        double prevError = 0;

        double kP = 5.0;
        double kD = 2.0;

        while (true)
        {
            double heading = inertial.get();
            if (heading < -360 || heading > 360)
            {
                continue;
            }
            bool turnRight = false;
            double error = deg - heading;
            while(error>360){
                error-=360;
            }
            pros::lcd::print(0, "heading  >> %5.2f", heading);
            pros::lcd::print(1, "target   >> %5.2f", deg);
            pros::lcd::print(2, "orignal error  >> %5.2f", error);

            if (error > 0)
            {
                if (error > 180)
                {
                    // left
                    error = 360 - error;
                    turnRight = false;
                }
                else
                {
                    // right
                    turnRight = true;
                }
            }
            else
            {
                if (error < -180)
                {
                    // right
                    error = 360 + error;
                    turnRight = true;
                }
                else
                {
                    // left
                    error = error * -1;
                    turnRight = false;
                }
            }

            // error += bias;
            if (turnRight)
                pros::lcd::print(3, "error  >> %5.2f   RIGHT", error);
            else
                pros::lcd::print(3, "error  >> %5.2f   LEFT", error);

            pros::lcd::print(4, "tole  >> %5.2f", tolerance);

            if (error > tolerance)
            {
                double pow = error * kP + derivative * kD;
                if (pow < minPow)
                {
                    pow = minPow;
                }
                if (turnRight == false)
                {
                    // turn A(error) Left
                    drive::turn(pow * -1);
                }
                else
                {
                    // turn B(error) right
                    drive::turn(pow);
                }
                pros::lcd::print(5, "TURN POW >> %5.2f", pow);
            }
            else
            {
                // timer.placeMark();

                // while ((fabs(error) < tolerance && timer.getDtFromMark() >= 1_s))
                // {
                // }

                drive::turn(0);
                break;
            }
            derivative = error - prevError;
            prevError = error;
            pros::delay(5);

            drive::drive(0);
        }
    }

    void distancePID(int setPoint, bool direction)
    {
        double kP = 5.0;
        double kP_angular = 1.0;

        int initHeading = inertial.get();

        double error;
        double angularError;
        double tune;

        double tol = 1.0;
        double power = 0;
        
        bool turnRight;

        double sDist = dist1.get() - setPoint;

        if (direction)
        {
            error = dist1.get() - setPoint;

            while (dist1.get() >setPoint + tol)
            {

                angularError = inertial.get() - initHeading;

                if (angularError > 0)
                {
                    if (angularError > 180)
                    {
                        // left
                        angularError = 360 - angularError;
                        turnRight = false;
                    }
                    else
                    {
                        // right
                        turnRight = true;
                    }
                }
                else
                {
                    if (angularError < -180)
                    {
                        // right
                        angularError = 360 + angularError;
                        turnRight = true;
                    }
                    else
                    {
                        // left
                        angularError = angularError * -1;
                        turnRight = false;
                    }
                }

                if (turnRight)
                {
                    tune = angularError * kP_angular;
                }
                else
                {
                    tune = angularError * kP_angular * -1;
                }

                if (!(dist1.get()<=2400 && dist1.get() > 10))
                {
                    error = 2400;
                }

                power = error * 500 / sDist * kP;

                left.moveVelocity(power + tune);
                right.moveVelocity(power - tune);

                pros::lcd::print(1, "dist1 >> %5.2f", dist1.get());
                pros::lcd::print(5, "TURN dist2 >> %5.2f", dist2.get());
            }
        }
        else
        {
            error = dist2.get() - setPoint;
            double dist = dist2.get();

            while (dist2.get() > setPoint + tol)
            {

                angularError = inertial.get() - initHeading;

                if (angularError > 0)
            {
                if (angularError > 180)
                {
                    // left
                    angularError = 360 - angularError;
                    turnRight = false;
                }
                else
                {
                    // right
                    turnRight = true;
                }
            }
            else
            {
                if (angularError < -180)
                {
                    // right
                    angularError = 360 + angularError;
                    turnRight = true;
                }
                else
                {
                    // left
                    angularError = angularError * -1;
                    turnRight = false;
                }
            }

                if (turnRight)
                {
                    tune = angularError * kP_angular;
                }
                else
                {
                    tune = angularError * kP_angular * -1;
                }

                if (!(dist2.get()<2400&& dist2.get()>10))
                {
                    error = 2400;
                }

                power = error * 600 / sDist * kP;

                left.moveVelocity(-power);
                right.moveVelocity(-power);
                pros::lcd::print(1, "dist1 >> %5.2f", dist1.get());
                pros::lcd::print(5, "TURN dist2 >> %5.2f", dist2.get());
            }
        }

        stop();
    }

}

void driveTurnAssist(int units, int power)
{
    pid::resetDriveEncoders();

    int direction = abs(units) / units;
    double rotation = inertial.get();
    int setPoint = abs(units);
    
    double initHeading = inertial.get();
    double angularError;
        
    double kP_angular = 3.0;
    int tune = 30;
    double tolerance = 3.0;

    while (pid::avgDriveEncoders() < abs(units))
    {
        
        angularError = pid::correctionDegrees(inertial.get(), initHeading);

        //===============================================

            tune = angularError * kP_angular;

            if(tune > 100)
            {
                tune = 100;
            }
        
            left.moveVelocity(power + tune);
            right.moveVelocity(power - tune);
           

        pros::delay(10);
    }
    pid::stop(0);
}

double correctionDegrees(double heading, double setPoint)
{
    double angularError = heading - setPoint;
    bool turnRight;

    if (angularError > 0)
            {
                if (angularError > 180)
                {
                    // left
                    angularError = 360 - angularError;
                    turnRight = false;
                }
                else
                {
                    // right
                    turnRight = true;
                }
            }
            else
            {
                if (angularError < -180)
                {
                    // right
                    angularError = 360 + angularError;
                    turnRight = true;
                }
                else
                {
                    // left
                    angularError = angularError * -1;
                    turnRight = false;
                }
            }

            if(turnRight)
            {
                return angularError;
            }
            else
            {
                return angularError * -1;
            }

}