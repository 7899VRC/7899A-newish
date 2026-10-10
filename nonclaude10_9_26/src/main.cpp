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

pneumatics clawPiston = pneumatics(Brain.ThreeWirePort.A);
pneumatics flipPiston = pneumatics(Brain.ThreeWirePort.H);

inertial imu = inertial(PORT17);
inertial Gyro = inertial(PORT1);

motor left_front = motor(PORT19, ratio6_1, true);
motor left_back = motor(PORT5, ratio6_1, false);

motor right_front = motor(PORT15, ratio6_1, false);
motor right_back = motor(PORT9, ratio6_1, true);

motor left_lift = motor(PORT20, ratio18_1, false);
motor right_lift = motor(PORT14, ratio18_1, true);

motor claw = motor(PORT20, ratio6_1, true);

int clawMode = 0;
int flipMode = 0;

// ============================================================
// ODOMETRY STARTER  (drive-motor encoders + IMU heading)
// Paste this below `int flipMode = 0;` in your main.cpp
//
// Field coordinates (inches):
//   +Y = forward at heading 0, +X = right, heading is CLOCKWISE-positive degrees
//   (this matches how the VEX inertial sensor reports rotation)
// ============================================================

// ---------- CONFIG: change these to match your robot ----------
const double WHEEL_DIAMETER_IN = 3.25;  // 2.75, 3.25 or 4.0 for standard VEX omnis
const double GEAR_RATIO        = 1.333;   // wheel turns per ONE motor turn
                                        // (e.g. 48T motor gear -> 36T wheel gear = 48/36... see note below)
                                        // = (driving teeth) / (driven teeth)
const double IN_PER_DEG = (M_PI * WHEEL_DIAMETER_IN * GEAR_RATIO) / 360.0;

// ---------- Robot pose (shared between threads) ----------
vex::mutex poseMutex;
double poseX = 0.0;        // inches
double poseY = 0.0;        // inches
double poseHeading = 0.0;  // degrees, clockwise positive (from IMU)

// ---------- Helpers ----------
double wrap180(double deg) {
  while (deg > 180.0)  deg -= 360.0;
  while (deg <= -180.0) deg += 360.0;
  return deg;
}

double leftSideDegrees() {
  return (left_front.position(degrees) + left_back.position(degrees)) / 2.0;
}

double rightSideDegrees() {
  return (right_front.position(degrees) + right_back.position(degrees)) / 2.0;
}

void setPose(double x, double y, double headingDeg) {
  poseMutex.lock();
  poseX = x;
  poseY = y;
  poseHeading = headingDeg;
  poseMutex.unlock();
  imu.setRotation(headingDeg, degrees);
}

// Thread-safe getters
double getX()       { poseMutex.lock(); double v = poseX;       poseMutex.unlock(); return v; }
double getY()       { poseMutex.lock(); double v = poseY;       poseMutex.unlock(); return v; }
double getHeading() { poseMutex.lock(); double v = poseHeading; poseMutex.unlock(); return v; }

// ---------- The odometry loop ----------
int odometryLoop() {
  double prevLeft  = leftSideDegrees();
  double prevRight = rightSideDegrees();
  double prevHeadingRad = imu.rotation(degrees) * M_PI / 180.0;

  while (true) {
    double curLeft  = leftSideDegrees();
    double curRight = rightSideDegrees();
    double curHeadingDeg = imu.rotation(degrees);
    double curHeadingRad = curHeadingDeg * M_PI / 180.0;

    // Change in each side since last loop, converted to inches
    double dLeft  = (curLeft  - prevLeft)  * IN_PER_DEG;
    double dRight = (curRight - prevRight) * IN_PER_DEG;

    // Forward travel of the robot's center
    double dForward = (dLeft + dRight) / 2.0;

    // Use the average heading over this step (better than start or end alone)
    double avgHeading = (prevHeadingRad + curHeadingRad) / 2.0;

    poseMutex.lock();
    poseX += dForward * sin(avgHeading);
    poseY += dForward * cos(avgHeading);
    poseHeading = curHeadingDeg;
    poseMutex.unlock();

    prevLeft = curLeft;
    prevRight = curRight;
    prevHeadingRad = curHeadingRad;

    wait(10, msec);  // 100 Hz update
  }
  return 0;
}

// ---------- Brain screen debug ----------
int displayLoop() {
  while (true) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("X: %.2f in", getX());
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Y: %.2f in", getY());
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Heading: %.2f deg", getHeading());
    wait(50, msec);
  }
  return 0;
}

// ---------- Call this once from pre_auton() ----------
void initOdometry() {
  imu.calibrate();
  while (imu.isCalibrating()) wait(20, msec);   // IMU MUST finish calibrating, robot must be still

  left_front.resetPosition();  left_back.resetPosition();
  right_front.resetPosition(); right_back.resetPosition();
  imu.resetRotation();

  setPose(0, 0, 0);

  task odomTask(odometryLoop);
  task dispTask(displayLoop);
}

// ============================================================
// OPTIONAL: simple motion helpers that use the pose above
// ============================================================
void driveVolts(double leftV, double rightV) {
  left_front.spin(forward, leftV, volt);
  left_back.spin(forward, leftV, volt);
  right_front.spin(forward, rightV, volt);
  right_back.spin(forward, rightV, volt);
}

void stopDrive() {
  left_front.stop(brake);  left_back.stop(brake);
  right_front.stop(brake); right_back.stop(brake);
}

// Turn in place to an absolute field heading (degrees)
void turnToHeading(double targetDeg, double timeoutMs = 2000) {
  const double kP = 0.12;
  double start = Brain.timer(msec);
  while (Brain.timer(msec) - start < timeoutMs) {
    double err = wrap180(targetDeg - getHeading());
    if (fabs(err) < 1.0) break;
    double v = kP * err;
    if (v > 10) v = 10;
    if (v < -10) v = -10;
    driveVolts(v, -v);       // positive error = turn clockwise (left faster)
    wait(10, msec);
  }
  stopDrive();
}

// Drive to an absolute (x, y) point on the field, in inches
void driveToPoint(double tx, double ty, double timeoutMs = 4000) {
  const double kP_dist = 0.8;   // volts per inch of error
  const double kP_turn = 0.15;  // volts per degree of heading error
  double start = Brain.timer(msec);

  while (Brain.timer(msec) - start < timeoutMs) {
    double dx = tx - getX();
    double dy = ty - getY();
    double dist = sqrt(dx * dx + dy * dy);
    if (dist < 1.0) break;

    double targetHeading = atan2(dx, dy) * 180.0 / M_PI;  // 0 = +Y, clockwise positive
    double headingErr = wrap180(targetHeading - getHeading());

    // Slow down when pointed the wrong way
    double fwd = kP_dist * dist * cos(headingErr * M_PI / 180.0);
    double turn = kP_turn * headingErr;

    if (fwd > 10) fwd = 10;
    if (fwd < -10) fwd = -10;

    driveVolts(fwd + turn, fwd - turn);
    wait(10, msec);
  }
  stopDrive();
}

// ============================================================
// HOW TO USE
// ============================================================
// void pre_auton(void) {
//   initOdometry();
// }
//
// void autonomous(void) {
//   setPose(0, 0, 0);
//   driveToPoint(0, 24);     // 24 inches straight ahead
//   turnToHeading(90);       // face right
//   driveToPoint(24, 24);    // 24 inches to the right
// }

// ============================================================
// MOTOR TEMPERATURE MONITOR  (no printAt version)
// Paste below your motor definitions (and below odometry code if you use it).
//
//   GREEN  = OK
//   YELLOW = WARM  (getting hot, ease off)
//   RED    = HOT   (motor will start cutting power)
//   GRAY   = motor not detected (unplugged / wrong port)
//
// Rumbles the controller and prints the hot motor's name on the
// controller screen when a motor first goes RED.
//
// NOTE: This uses the whole Brain screen. If you're using the odometry
// code, delete `task dispTask(displayLoop);` in initOdometry().
// ============================================================

// ---------- CONFIG ----------
const int WARM_TEMP_C = 45;   // yellow at or above this (degrees C)
const int HOT_TEMP_C  = 55;   // red at or above this   (degrees C)

// ---------- Which motors to monitor ----------
struct MonitoredMotor {
  const char* label;   // max 8 characters
  motor* m;
  int lastTemp;
  int lastState;
  bool wasHot;
};

enum MotorState { STATE_OK = 0, STATE_WARM = 1, STATE_HOT = 2, STATE_MISSING = 3 };

MonitoredMotor monitored[] = {
  {"L FRONT", &left_front,  -999, -1, false},
  {"L BACK",  &left_back,   -999, -1, false},
  {"R FRONT", &right_front, -999, -1, false},
  {"R BACK",  &right_back,  -999, -1, false},
  {"L LIFT",  &left_lift,   -999, -1, false},
  {"R LIFT",  &right_lift,  -999, -1, false},
  {"CLAW",    &claw,        -999, -1, false},
};
const int NUM_MONITORED = sizeof(monitored) / sizeof(monitored[0]);

// ---------- Layout ----------
// Default Brain font (mono20): each character cell is 12 px wide, 20 px tall.
// Screen = 40 columns x 12 rows. Cursor positions are (row, column), 1-based.
const int COLS = 4;
const int BOX_W = 108;
const int BOX_H = 96;

int boxPixelX(int index) { return 12 + (index % COLS) * 120; }
int boxPixelY(int index) { return 20 + (index / COLS) * 100; }
int boxTextCol(int index) { return 3 + (index % COLS) * 10; }
int boxTextRow(int index) { return 3 + (index / COLS) * 5; }   // first text row inside box

MotorState getMotorState(motor& m, int& tempOut) {
  if (!m.installed()) {
    tempOut = 0;
    return STATE_MISSING;
  }
  tempOut = (int)m.temperature(celsius);
  if (tempOut >= HOT_TEMP_C)  return STATE_HOT;
  if (tempOut >= WARM_TEMP_C) return STATE_WARM;
  return STATE_OK;
}

void drawMotorBox(int index, const MonitoredMotor& mm, MotorState state, int temp) {
  vex::color fill;
  vex::color text;
  const char* status;

  switch (state) {
    case STATE_OK:   fill = vex::color::green;  text = vex::color::black; status = "OK";       break;
    case STATE_WARM: fill = vex::color::yellow; text = vex::color::black; status = "WARM";     break;
    case STATE_HOT:  fill = vex::color::red;    text = vex::color::white; status = "HOT!";     break;
    default:         fill = vex::color(70, 70, 70); text = vex::color::white; status = "NO MOTOR"; break;
  }

  Brain.Screen.setPenColor(vex::color::black);
  Brain.Screen.setFillColor(fill);
  Brain.Screen.drawRectangle(boxPixelX(index), boxPixelY(index), BOX_W, BOX_H);

  Brain.Screen.setPenColor(text);
  int row = boxTextRow(index);
  int col = boxTextCol(index);

  // Label
  Brain.Screen.setCursor(row, col);
  Brain.Screen.print("%s", mm.label);

  // Temperature
  Brain.Screen.setCursor(row + 1, col);
  if (state != STATE_MISSING) {
    Brain.Screen.print("%d C", temp);
  }

  // Status
  Brain.Screen.setCursor(row + 2, col);
  Brain.Screen.print("%s", status);
}

void drawSummaryBox(int index, int okCount, int warmCount, int hotCount, int missingCount) {
  Brain.Screen.setPenColor(vex::color::white);
  Brain.Screen.setFillColor(vex::color::black);
  Brain.Screen.drawRectangle(boxPixelX(index), boxPixelY(index), BOX_W, BOX_H);

  int row = boxTextRow(index);
  int col = boxTextCol(index);

  Brain.Screen.setCursor(row, col);     Brain.Screen.print("SUMMARY");
  Brain.Screen.setCursor(row + 1, col); Brain.Screen.print("OK:   %d", okCount);
  Brain.Screen.setCursor(row + 2, col); Brain.Screen.print("WARM: %d", warmCount);
  Brain.Screen.setCursor(row + 3, col); Brain.Screen.print("HOT:  %d", hotCount);
  if (missingCount > 0) {
    Brain.Screen.setCursor(row + 4, col); Brain.Screen.print("MISS: %d", missingCount);
  }
}

int motorMonitorLoop() {
  Brain.Screen.setFont(vex::fontType::mono20);
  Brain.Screen.clearScreen(vex::color::black);
  Brain.Screen.setPenColor(vex::color::white);
  Brain.Screen.setFillColor(vex::color::black);
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("MOTOR TEMPS  warm>=%dC hot>=%dC", WARM_TEMP_C, HOT_TEMP_C);

  int lastSummary = -1;
  bool alertShown = false;

  while (true) {
    int ok = 0, warm = 0, hot = 0, missing = 0;
    const char* newHotName = nullptr;

    for (int i = 0; i < NUM_MONITORED; i++) {
      int temp;
      MotorState state = getMotorState(*monitored[i].m, temp);

      switch (state) {
        case STATE_OK:   ok++;      break;
        case STATE_WARM: warm++;    break;
        case STATE_HOT:  hot++;     break;
        default:         missing++; break;
      }

      if (temp != monitored[i].lastTemp || (int)state != monitored[i].lastState) {
        drawMotorBox(i, monitored[i], state, temp);
        monitored[i].lastTemp = temp;
        monitored[i].lastState = (int)state;
      }

      if (state == STATE_HOT && !monitored[i].wasHot) {
        newHotName = monitored[i].label;
      }
      monitored[i].wasHot = (state == STATE_HOT);
    }

    int summaryKey = ok * 1000 + warm * 100 + hot * 10 + missing;
    if (summaryKey != lastSummary) {
      drawSummaryBox(NUM_MONITORED, ok, warm, hot, missing);
      lastSummary = summaryKey;
    }

    if (newHotName != nullptr) {
      Controller1.rumble("---");
      Controller1.Screen.clearLine(3);
      Controller1.Screen.setCursor(3, 1);
      Controller1.Screen.print("HOT: %s", newHotName);
      alertShown = true;
    } else if (hot == 0 && alertShown) {
      Controller1.Screen.clearLine(3);
      alertShown = false;
    }

    wait(250, msec);
  }
  return 0;
}

// ---------- Call once from pre_auton() ----------
void startMotorMonitor() {
  task monitorTask(motorMonitorLoop);
}

/*---------------------------------------------------------------------------*/
/*                              Tunable Constants                            */
/*---------------------------------------------------------------------------*/

constexpr int    JOYSTICK_DEADBAND   = 5;      // pct, ignore small stick drift
constexpr float  TURN_SENSITIVITY_K  = 0.5f;   // scales turn contribution in arcade drive
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
    initOdometry();
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
  driveToPoint(0, 100);
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
      clawMode ++;
      if (clawMode % 2 == 0){
        clawPiston.set(true);
        wait(0.5, sec);
      }
      else{
        clawPiston.set(false);
        wait(0.5, sec);
      }
    } 
    
    if (Controller1.ButtonR2.pressing()) {
      flipMode ++;
      if (flipMode % 2 == 0){
        flipPiston.set(true);
        wait(0.5, sec);
      }
      else {
        flipPiston.set(false);
        wait(0.5, sec);
      }
    } 
  }
    //handle_tipping();

    // Brain.Screen.print("Right Speed: ", right_speed);
    // Brain.Screen.newLine();
    // Brain.Screen.print("Left Speed: ", left_speed);
    // Brain.Screen.newLine();


    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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