#include "main.h"
#include "autoSelect/selection.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
    selector::init();
    drive::init();
    mogoLift::init();
    parallelLift::init();
    rollers::init();
    pid::inertialReset();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
    // selector::auton == 1 : Red Front
    // selector::auton == 2 : Red Back
    // selector::auton == 3 : Do Nothing
    // selector::auton == -1 : Blue Front
    // selector::auton == -2 : Blue Back
    // selector::auton == -3 : Do Nothing
    // selector::auton == 0 : Skills
    //

    // test
    // pid::delaySeconds(3);
    // auton::redRight();
    // auton::rightOneGoal();
    //auton::leftRing();
    
        if (selector::auton == 0)
        {
            auton::skills();
        }

        else if (selector::auton == 1)
        { // run auton for Front Red
             auton::rightOneGoal();
            //auton::skills();
        }

        else if (selector::auton == 2)
        {
            auton::leftRing();

        }

        else if (selector::auton == 3)
        {
            auton::leftGoal();
        }

        else if (selector::auton == -1)
        {
            auton::rightOneGoal();
        }

        else if (selector::auton == -2)
        {
            auton::leftRing();
        }

        else if (selector::auton == -3)
        {
            auton::leftGoal();
        }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
    drive::init();
    mogoLift::init();
    parallelLift::init();
    rollers::init();

    while (true)
    {
        drive::opcontrol();
        mogoLift::opcontrol();
        parallelLift::opcontrol();
        rollers::opcontrol();

        // Wait and give up the time we don't need to other tasks.
        // Additionally, joystick values, motor telemetry, etc. all updates every 10 ms.
        pros::delay(10);
    }
}
