/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      8/24/2026, 4:32:44 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller Controller1;

inertial imu = inertial(PORT21);

motor left_front = motor(PORT14, ratio6_1, true);
motor left_back = motor(PORT13, ratio6_1, false);

motor right_front = motor(PORT17, ratio6_1, false);
motor right_back = motor(PORT16, ratio6_1, true);

motor left_lift = motor(PORT2, ratio6_1, true);
motor right_lift = motor(PORT11, ratio6_1, true);

motor claw = motor(PORT20, ratio6_1, true);

/*---------------------------------------------------------------------------*/
/*                              Tunable Constants                            */
/*---------------------------------------------------------------------------*/

constexpr int    JOYSTICK_DEADBAND   = 5;      // pct, ignore small stick drift
constexpr float  TURN_SENSITIVITY_K  = 1.0f;   // scales turn contribution in arcade drive
constexpr float  SLOW_MODE_FACTOR    = 0.4f;   // speed multiplier when ButtonB held

constexpr double TURN_KP             = 2.0;    // proportional gain for turn_to_angle
constexpr double TURN_TOLERANCE_DEG  = 1.0;    // acceptable heading error to stop
constexpr double TURN_MIN_SPEED      = 6.0;    // minimum pct to overcome static friction
constexpr int    TURN_TIMEOUT_MS     = 3000;   // safety cutoff if turn never settles
constexpr int    TURN_SETTLE_LOOPS   = 3;      // consecutive in-tolerance loops required

constexpr double TIP_ANGLE_THRESHOLD_DEG = 12.0; // pitch angle considered "tipping"

/*---------------------------------------------------------------------------*/
/*                              Helper Functions                             */
/*---------------------------------------------------------------------------*/

void drive(int left_speed, int right_speed, int wt) {
  left_front.spin(forward, left_speed, pct);
  left_back.spin(reverse, left_speed, pct);

  right_front.spin(forward, right_speed, pct);
  right_back.spin(reverse, right_speed, pct);

  wait(wt, msec);
}

void drive_brake() {
  left_front.stop(brake);
  left_back.stop(brake);

  right_front.stop(brake);
  right_back.stop(brake);
}

void claw_hold() {
  claw.stop(hold);
  claw.setVelocity(100, percent);
}

void lift(directionType dir, int speed) {
  left_lift.spin(dir, speed, pct);
  right_lift.spin(dir, speed, pct);
}

void claws(directionType dir, int speed) {
  claw.spin(dir, speed, pct);
}

void turn_to_angle(double target_angle, int max_speed) {
  imu.setRotation(0, degrees);

  timer tmr;
  tmr.clear();

  int settled_loops = 0;

  while (true) {
    double current_angle = imu.rotation(degrees);
    double error = target_angle - current_angle;

    if (fabs(error) < TURN_TOLERANCE_DEG) {
      settled_loops++;
      if (settled_loops >= TURN_SETTLE_LOOPS) {
        break;
      }
    } else {
      settled_loops = 0;
    }

    if (tmr.time(msec) > TURN_TIMEOUT_MS) {
      break;
    }

    double speed = error * TURN_KP;

    if (speed > max_speed) speed = max_speed;
    if (speed < -max_speed) speed = -max_speed;

    if (speed > 0 && speed < TURN_MIN_SPEED) speed = TURN_MIN_SPEED;
    if (speed < 0 && speed > -TURN_MIN_SPEED) speed = -TURN_MIN_SPEED;

    drive(speed, -speed, 10);
  }
  drive_brake();
}

bool is_tipping() {
  double current_pitch = imu.pitch(degrees);
  return fabs(current_pitch) > TIP_ANGLE_THRESHOLD_DEG;
}

void handle_tipping() {
  if (is_tipping()) {
    drive_brake();
    left_lift.stop(hold);
    right_lift.stop(hold);
    Controller1.rumble(".");
  }
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
    Brain.Screen.print("Calibrating IMU");
    imu.calibrate();
    while (imu.isCalibrating()) {
        wait(20, msec);
    }
    Brain.Screen.clearScreen();
    Brain.Screen.print("IMU Calibrated");
    drive_brake();
    claw_hold();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    int forward_raw = Controller1.Axis3.position(pct);
    int turn_raw = Controller1.Axis1.position(pct);

    int forward_val = 0;
    int turn_val = 0;

    if (abs(forward_raw) > JOYSTICK_DEADBAND) {
        forward_val = forward_raw;
    }

    if (abs(turn_raw) > JOYSTICK_DEADBAND) {
        turn_val = turn_raw;
    }

    float slow_factor = Controller1.ButtonB.pressing() ? SLOW_MODE_FACTOR : 1.0f;

    float left_speed = (forward_val + (TURN_SENSITIVITY_K * turn_val)) * slow_factor;
    float right_speed = (forward_val - (TURN_SENSITIVITY_K * turn_val)) * slow_factor;

    drive(left_speed, right_speed, 0);

    if (Controller1.ButtonL1.pressing()) {
      lift(fwd, 100);
    }
    else if (Controller1.ButtonL2.pressing()) {
      lift(reverse, 100);
    }
    else {
      left_lift.stop(hold);
      right_lift.stop(hold);
    }

    if (Controller1.ButtonR1.pressing()) {
      claws(fwd, 70);
    }
    else if (Controller1.ButtonR2.pressing()) {
      claws(reverse, 70);
    }
    else {
      claw.stop(hold);
    }

    handle_tipping();

    // Brain.Screen.print("Right Speed: ", right_speed);
    // Brain.Screen.newLine();
    // Brain.Screen.print("Left Speed: ", left_speed);
    // Brain.Screen.newLine();


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  drive_brake();
  claw_hold();

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}