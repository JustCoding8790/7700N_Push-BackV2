/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jnaor, JustCoding8                                                    */
/*    Created:      2/6/2026, 5:40:27 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// MARK: Includes
#include "vex.h"
#include <iostream>
#include <random>
#include <chrono>

// Essentials
using namespace vex;
int autonSelected = 0;
int autonMin = 0;
int autonMax = 8;
bool selectPressed = false;
bool selectingAuton = true;
bool redoSelection = false;
bool gui = true;
bool guiChanged = true;
int arcadeMode = 0;
double wheelDiamiter = 3.25;
double gearRatioExternal = 0.75;
double pi = 3.14;


// A global instance of competition
competition Competition;

// Core stuffs
controller Controller1;
brain Brain;


//MARK: Motors
// Drivetrain
/* Left Front */ motor leftF = motor(PORT1, ratio6_1, true);
/* Left Middle */ motor leftM = motor(PORT3, ratio6_1, true);
/* Left Rear */ motor leftR = motor(PORT2, ratio6_1, true);
/* Right Front */ motor rightF = motor(PORT4, ratio6_1, false);
/* Right Middle */ motor rightM = motor(PORT9, ratio6_1, false);
/* Right Rear */ motor rightR = motor(PORT10, ratio6_1, false);

// Others
/* Intake */ motor intake = motor(PORT7, ratio6_1, false);
/* Top Stage */ motor topStage = motor(PORT8, ratio6_1, false);

//MARK: Penaumatics
/* Unloader*/ digital_out unloader = digital_out(Brain.ThreeWirePort.A);

// Inertial
inertial inertialSensor = inertial(PORT12);
color magenta = color(255, 50, 150);
color brown = color(150, 100, 0);
color arcade = color(50, 150, 200);
color tank = color(0, 50, 0);
color dual = color(255, 200, 255);









//MARK: Functions


// Base Auton Funcs
bool speedIsLocked = false;
const double kp = 0.025;
const double ki = 0.0025; 
const double turnkp = 0.006; // 0.004 tried, overshooting by 3-4 deg still, 0.006 overshoots by up to 7-8 deg
const double turnki = 0.0006;

void driveTrainMove(float speed) {
  leftF.spin(fwd, speed, rpm);
  leftM.spin(fwd, speed, rpm);
  leftR.spin(fwd, speed, rpm);
  rightF.spin(fwd, speed, rpm);
  rightM.spin(fwd, speed, rpm);
  rightR.spin(fwd, speed, rpm);
}

void driveTrainStop() {
  leftF.stop();
  leftM.stop();
  leftR.stop();
  rightF.stop();
  rightM.stop();
  rightR.stop();
}

void setBrakeMode(brakeType mode) {
  leftF.setStopping(mode);
  leftM.setStopping(mode);
  leftR.setStopping(mode);
  rightF.setStopping(mode);
  rightM.setStopping(mode);
  rightR.setStopping(mode);
}

//MARK: Driver Funcs

void driveVolts(double lSpeed, double rSpeed, int waitTime) {
  lSpeed = lSpeed * 12.00;
  rSpeed = rSpeed * 12.00;
  leftF.spin(fwd, lSpeed, volt);
  leftM.spin(fwd, lSpeed, volt);
  leftR.spin(fwd, lSpeed, volt);
  rightF.spin(fwd, rSpeed, volt);
  rightM.spin(fwd, rSpeed, volt);
  rightR.spin(fwd, rSpeed, volt);
  wait(waitTime, msec);
}

void drivePct(double lSpeed, double rSpeed, int waitTime) {
  leftF.spin(fwd, lSpeed, pct);
  leftM.spin(fwd, lSpeed, pct);
  leftR.spin(fwd, lSpeed, pct);
  rightF.spin(fwd, rSpeed, pct);
  rightM.spin(fwd, rSpeed, pct);
  rightR.spin(fwd, rSpeed, pct);
  wait(waitTime, msec);
}


//MARK:--
//MARK: Auton funcs

void inchDrive(double target, long time) {
  int counter = 0;
  leftM.setPosition(0, rev);
  double currPos = leftM.position(rev) * pi * wheelDiamiter;
  double error = target - currPos;
  double acc_error = 0;
  double prev_error = 0;
  double speed = error * kp;
  long endTime = vex::timer::system() + time;
  while (fabs(error) > 0.5) {
    if (vex::timer::system() > endTime) {
      break;
    }
    currPos = leftM.position(rev) * pi * wheelDiamiter;
    prev_error = error;
    error = target - currPos;
    if (fabs(error) < 3) {
      acc_error = acc_error + error;
    }
    if ((error > 0 && prev_error < 0) || (error < 0 && prev_error > 0)) {
      acc_error = 0;
    }
    speed = error * kp + acc_error * ki;
    if(counter++ % 10 == 0){
    printf("rightFrontpos: %0.2f, currPos: %0.2f, error: %0.2f, kp: %0.2f, ki: %0.2f, speed: %0.2f\n", leftM.position(rev), currPos, error, kp * error, ki * acc_error, speed);
    }
    driveVolts(speed, speed, 10);
  }
  printf("currPos: %0.2f\n", currPos);
  driveTrainStop();
  wait(250, msec);
  currPos = leftM.position(rev) * pi * wheelDiamiter;
  printf("currPos: %0.2f\n", currPos);

}

void inchDrive(double target) {
  inchDrive(target, 5000);
}

void autonTurn(double targetRotation) {
// FOR TURINING RIGHT, USE POSITIVE
 int debugCounter = 0;
 inertialSensor.resetRotation();
 inertialSensor.setRotation(0, deg);
 double currRotation = inertialSensor.rotation(deg);
 double startRotation = currRotation;
 double error = targetRotation - currRotation;
 double acc_error = 0;
 double prev_error = 0;
 double speed = error * turnkp;

 while (fabs(error) > 2) {
  currRotation = inertialSensor.rotation(deg);
  prev_error = error;
  error = targetRotation - currRotation;
  if (fabs(error) < 15) {
      acc_error = acc_error + error;
    }
    if ((error > 0 && prev_error < 0) || (error < 0 && prev_error > 0)) {
      acc_error = 0;
    }
  speed = error * turnkp + acc_error * turnki;
  driveVolts(speed, -speed, 10);
  if (debugCounter++ % 10 == 0){
    printf("autonTurn %0.2f error: %0.2f [%0.2f] kp: %0.2f ki: %0.2f speed: %0.2f\n", inertialSensor.rotation(deg), targetRotation - inertialSensor.rotation(deg), error, turnkp * error, turnki * acc_error, speed);
  }
 }
  driveTrainStop();
  printf("autonTurn finished! autonTurn: %0.2f currRotation: %0.2f speed: %0.2f\n", targetRotation, currRotation, inertialSensor.rotation(deg) - startRotation, speed);
 wait(500, msec);
 currRotation = inertialSensor.rotation(deg);
 error = targetRotation - currRotation;
 printf("currRotation: %0.2f, error: %0.2f\n", currRotation, error);
}

void drawGUI() {
  // Draws 2 buttons to be used for selecting auto
  Brain.Screen.clearScreen();
  Brain.Screen.setFont(monoM);
  Brain.Screen.printAt(1, 200, "AUTON = %d, ", autonSelected);
  Brain.Screen.setFillColor(red);
  //Brain.Screen.drawCircle(310, 75, 25);
  if (!(selectPressed)) {
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(20, 50, 100, 100);
    Brain.Screen.drawCircle(310, 75, 25);
    Brain.Screen.setPenColor(black);
    Brain.Screen.printAt(25, 75, "SELECT");
    Brain.Screen.printAt(25, 100, "YOUR");
    Brain.Screen.printAt(25, 100, "AUTON");
    Brain.Screen.setPenColor(white);
  }
  else if (autonSelected == 0) {
    Brain.Screen.setFillColor(white);
    Brain.Screen.drawRectangle(20, 50, 100, 100);
    Brain.Screen.drawCircle(310, 75, 25);
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(25, 75, "Testing");
    Brain.Screen.printAt(25, 100, "Auton");
    Brain.Screen.printAt(25, 125, "(N/A)");
  }
  else if (autonSelected == 1) {
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(20, 50, 100, 100);
    Brain.Screen.drawCircle(310, 75, 25);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(25, 75, "x.x");
    Brain.Screen.printAt(25, 100, "x.x");
    Brain.Screen.printAt(25, 125, "(+x)");
  }
  else if (autonSelected == 2) {
    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(20, 50, 100, 100);
    Brain.Screen.drawCircle(310, 75, 25);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(25, 75, "x.x");
    Brain.Screen.printAt(25, 100, "x.x");
    Brain.Screen.printAt(25, 125, "(+x)");
  }
  else if (autonSelected == 3) {
    Brain.Screen.setFillColor(orange);
    Brain.Screen.drawRectangle(20, 50, 100, 100);
    Brain.Screen.drawCircle(310, 75, 25);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(25, 75, "x.x");
    Brain.Screen.printAt(25, 100, "x.x");
    Brain.Screen.printAt(25, 125, "(+x)");
  }
  else if (autonSelected == 4) {
    Brain.Screen.setFillColor(purple);
    Brain.Screen.drawRectangle(20, 50, 100, 100);
    Brain.Screen.drawCircle(310, 75, 25);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(25, 75, "x.x");
    Brain.Screen.printAt(25, 100, "x.x");
    Brain.Screen.printAt(25, 125, "(+x)");
  }
  else if (autonSelected == 5) {
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawRectangle(20, 50, 100, 100);
    Brain.Screen.drawCircle(310, 75, 25);
    Brain.Screen.setPenColor(black);
    Brain.Screen.printAt(25, 75, "x.x");
    Brain.Screen.printAt(25, 100, "x.x");
    Brain.Screen.printAt(25, 125, "(+x)");
    Brain.Screen.setPenColor(white);
  }
  else if (autonSelected == 6) {
    Brain.Screen.setFillColor(cyan);
    Brain.Screen.drawRectangle(20, 50, 100, 100);
    Brain.Screen.drawCircle(310, 75, 25);
    Brain.Screen.setPenColor(black);
    Brain.Screen.printAt(25, 75, "x.x");
    Brain.Screen.printAt(25, 100, "x.x");
    Brain.Screen.printAt(25, 125, "(+x)");
    Brain.Screen.setPenColor(white);
  }
  else if (autonSelected == 7) {
    Brain.Screen.setFillColor(magenta);
    Brain.Screen.drawRectangle(20, 50, 100, 100);
    Brain.Screen.drawCircle(310, 75, 25);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(25, 75, "x.x");
    Brain.Screen.printAt(25, 100, "x.x");
    Brain.Screen.printAt(25, 125, "(+x)");
  }
  else if (autonSelected == 8) {
    Brain.Screen.setFillColor(brown);
    Brain.Screen.drawRectangle(20, 50, 100, 100);
    Brain.Screen.drawCircle(310, 75, 25);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(25, 75, "x.x");
    Brain.Screen.printAt(25, 100, "x.x");
    Brain.Screen.printAt(25, 125, "(+x)");
  }
  if (selectingAuton) {
    Brain.Screen.setFillColor(red);
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawRectangle(170, 50, 100, 100);
    Brain.Screen.printAt(175, 75, "READY?");
    Brain.Screen.setFillColor(transparent);
    Brain.Screen.printAt(1, 40, "7700N VRC: Push Back [Auton Menu]");
  }
  else {
    // MARK: Commit = Add quote
    // For tracking commits
    std::random_device rd;
    std::mt19937 gen(rd() ^ std::chrono::high_resolution_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<> distrib(1, 14);
    int random_quote = distrib(gen);
    Brain.Screen.setFillColor(transparent);
    if (!(redoSelection)) {
      random_quote = 14;  // So we can see the version instantly on the brain
    }
    if (autonSelected == 0) {
      Brain.Screen.printAt(1, 40, "TUNING TEST ACTIVATED - SERIOUS MODE = TRUE");
    }
    if (random_quote == 1) {
      Brain.Screen.printAt(1, 40, "Do a barrel roll!");
    }
    else if (random_quote == 5) {
      Brain.Screen.printAt(1, 40, "Good day for a swell battle!");
    }
    else if (random_quote == 2) {
      Brain.Screen.printAt(1, 40, "A great slam and then some!");
    }
    else if (random_quote == 3) {
      Brain.Screen.printAt(1, 40, "A brawl is surely brewing!");
    }
    else if (random_quote == 4) {
      Brain.Screen.printAt(1, 40, "It's all or nothing...");
    }
    else if (random_quote == 6) {
      Brain.Screen.printAt(1, 40, "NGAHHHHHHHHKAERTJUJHFSARTO!");
    }
    else if (random_quote == 7){
      Brain.Screen.printAt(1, 40, "YOU. ARE. NOT. FOLLOWING. THE DRESS CODE!!!");
    }
    else if (random_quote == 8) {
      Brain.Screen.printAt(1, 40, "Here's a real high-class bout!");
    }
    else if (random_quote == 9) {
      Brain.Screen.printAt(1, 40, "This match is getting red hot!");
    }
    else if (random_quote == 10) {
      Brain.Screen.printAt(1, 40, "Good luck! Look for any screwdrivers!");
    }
    else if (random_quote == 11) {
      Brain.Screen.printAt(1, 40, "All aboard! Next stop: off the table.");
    }
    else if (random_quote == 12) {
      Brain.Screen.printAt(1, 40, "Would you like a push on your back?");
    }
    else if (random_quote == 13) {
      Brain.Screen.printAt(1, 40, "So much to do, so little time. - MM");
    }
    else if (random_quote == 14) {
      Brain.Screen.printAt(1, 40, "Tank drive is tanking!");
    }
    else {
      Brain.Screen.printAt(1, 40, "Just you and the clock... and maybe more.");
    }
    Brain.Screen.setFillColor(green);
    Brain.Screen.setPenColor(black);
    Brain.Screen.drawRectangle(170, 50, 100, 100);
    Brain.Screen.printAt(175, 75, "WALLOP!");
    Brain.Screen.printAt(1, 200, "You're up!");
  }
  
  Brain.Screen.setFont(monoS);
  Brain.Screen.drawCircle(310, 75, 25);
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(345, 80, "Redo Selection");
  /*Brain.Screen.setFillColor(white);
  Brain.Screen.drawCircle(310, 135, 20);
  Brain.Screen.setFillColor(black);
  Brain.Screen.printAt(345, 140, "Check Inertial");*/
  if (arcadeMode == 1){
    Brain.Screen.setFillColor(arcade);
    Brain.Screen.drawCircle(310, 135, 25);
    Brain.Screen.setFillColor(black);
    Brain.Screen.printAt(345, 140, "Arcade Drive");
  }
  else if (arcadeMode == 2){
    Brain.Screen.setFillColor(dual);
    Brain.Screen.drawCircle(310, 135, 25);
    Brain.Screen.setFillColor(black);
    Brain.Screen.printAt(345, 140, "Dual Arcade");
  }
  else {
    Brain.Screen.setFillColor(tank);
    Brain.Screen.drawCircle(310, 135, 25);
    Brain.Screen.setFillColor(black);
    Brain.Screen.printAt(345, 140, "Tank Drive");
  }
}

void selectAuton() {
  int x = Brain.Screen.xPosition(); // get the x position of last touch of the screen
  int y = Brain.Screen.yPosition(); // get the y position of last touch of the screen

  // check to see if buttons were pressed
  if (x >= 20 && x <= 120 && y >= 50 && y <= 150 && selectingAuton){ // select button pressed
    autonSelected++;
    selectPressed = true;
    if (autonSelected > autonMax){
      autonSelected = autonMin; // rollover
    }
    drawGUI();
    Brain.Screen.printAt(1, 200, "Auton Selected =  %d   ", autonSelected);
  }


  if (x >= 170 && x <= 270 && y >= 50 && y <= 150) {
    selectingAuton = false; // GO button pressed
    Brain.Screen.setFillColor(black);
    Brain.Screen.printAt(345, 80, "Redo");
    Brain.Screen.printAt(290, 130, "Selection");
    //Brain.Screen.printAt(1, 200, "Auton  =  %d   AND GO!           ", autonSelected);
    drawGUI();
  }
  else if (x >= 285 && x <= 335 && y >= 50 && y <= 100) {
    selectingAuton = true; // UNDO button pressed
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(1, 200, "Redoing Selection - Auton = %d", autonSelected);
    redoSelection = true;
    drawGUI();
  }
  else if (x >= 285 && x <= 335 && y >= 110 && y <= 160) {
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    arcadeMode += 1;
    if (arcadeMode == 1) {
      Brain.Screen.printAt(1, 200, "Left Arcade Activated - Auton = %d", autonSelected);
    }
    else if (arcadeMode == 2) {
      Brain.Screen.printAt(1, 200, "Dual Arcade Activated - Auton = %d", autonSelected);
    }
    else {
      Brain.Screen.printAt(1, 200, "Tank Activated - Auton = %d", autonSelected);
      arcadeMode = 0;
    }
    drawGUI();
  }

  wait(100, msec); // slow it down
  Brain.Screen.setFillColor(black);
}

// MARK: Motor Monitor

double YOFFSET = 20; //offset for the display
//Writes a line for the diagnostics of a motor on the Brain
void MotorDisplay(double y, double curr, double temp)
{
	Brain.Screen.setFillColor(transparent);
	Brain.Screen.printAt(5, YOFFSET + y, "Current: %.1fA", curr);
	
	if (curr < 1){
		Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);
	} else if(curr >= 1 && curr  <= 2.5) {
		Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);
	} else {
		Brain.Screen.setFillColor(red);
		Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);
	}

	
	Brain.Screen.setFillColor(transparent);
	Brain.Screen.printAt(160, YOFFSET + y, "Temp: %.1fC", temp);
	
	if (temp < 30) {
	  Brain.Screen.setFillColor(blue);
	}
  else if(temp < 40 && temp  >= 30){
    Brain.Screen.setFillColor(green);
  }
  else if(temp < 45 && temp  >= 40){
    // ^^ AND gate ^^ (if 1 & 2 are true, true; if 1 or 2 is false, false)
		Brain.Screen.setFillColor(yellow);
    Controller1.rumble(".");
  }
  else if(temp <= 50 && temp  >= 45){
    // ^^ AND gate ^^ (if 1 & 2 are true, true; if 1 or 2 is false, false)
		Brain.Screen.setFillColor(orange);
    Controller1.rumble("-");
	}
  else if(temp <= 60 && temp  >= 50){
    Brain.Screen.setFillColor(red);
    Controller1.Screen.print("MOTORS OVERHEATING: COOL THEM DOWN ASAP");
    Controller1.rumble("- - - - - - -");
  }
  else {
		Brain.Screen.setFillColor(purple);
    while (true) {
      Controller1.rumble("-");
      Controller1.Screen.print("YOUR MOTORS ARE SPONTANIOUSLY COMBUSTING: SHUT DOWN YOUR ROBOT IMMEDIATELY");
      wait(500, msec);
    }
		/*Brain.Screen.drawRectangle(275, YOFFSET + y - 15, 15, 15);
		Brain.Screen.setFillColor(transparent);*/
	}
}


// Displays information on the brain
void Display()
{
  Brain.Screen.setFont(monoM);
	double leftFCurr = leftF.current(amp);
	double leftFTemp = leftF.temperature(celsius);
	double leftBackCurr = leftR.current(amp);
	double leftBackTemp = leftR.temperature(celsius);
  double leftMTemp = leftM.temperature(celsius);
  double leftMCurr = leftM.current(amp);
  double rightMTemp = rightM.temperature(celsius);
  double rightMCurr = rightM.current(amp);
	double rightFCurr = rightF.current(amp);
	double rightFTemp = rightF.temperature(celsius);
	double rightBackCurr = rightR.current(amp);
	double rightBackTemp = rightR.temperature(celsius);
  double intakeCurr = intake.current(amp);
	double intakeTemp = intake.temperature(celsius);
  double intakeConveyorCurr = topStage.current(amp);
  double intakeConveyorTemp = topStage.temperature(celsius);
	if (leftF.installed()){
		MotorDisplay(1, leftFCurr, leftFTemp);
		Brain.Screen.printAt(300, YOFFSET + 1, "LeftFront");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 1, "LeftFront - DISCONNECTED");
	}
	
	if (leftM.installed()) {
    MotorDisplay(21, leftMCurr, leftMTemp);
    Brain.Screen.printAt(300, YOFFSET + 21, "LeftMiddle");
  } else {
    Brain.Screen.printAt(5, YOFFSET + 21, "LeftMiddle - DISCONNECTED");
  }

	if (leftR.installed()){
		MotorDisplay(41, leftBackCurr, leftBackTemp);
		Brain.Screen.printAt(300, YOFFSET + 41, "LeftBack");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 41, "LeftBack - DISCONNECTED");
  }

	if (rightF.installed()) {
		MotorDisplay(61, rightFCurr, rightFTemp);
		Brain.Screen.printAt(300, YOFFSET + 61, "RightFront");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 61, "RightFront - DISCONNECTED");
	}

  if (rightM.installed()) {
    MotorDisplay(81, rightMCurr, rightMTemp);
    Brain.Screen.printAt(300, YOFFSET + 81, "RightMiddle");
  } else {
    Brain.Screen.printAt(5, YOFFSET + 81, "RightMiddle - DISCONNECTED");
  }

  if (rightR.installed()) {
    MotorDisplay(101, rightBackCurr, rightBackTemp);
    Brain.Screen.printAt(300, YOFFSET + 101, "RightRear");
  } else {
    Brain.Screen.printAt(5, YOFFSET + 101, "RightRear - DISCONNECTED");
  }
	
	if (intake.installed()) {
		MotorDisplay(121, intakeCurr, intakeTemp);
		Brain.Screen.printAt(300, YOFFSET + 121, "Intake");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 121, "Intake - DISCONNECTED");
	}

  if (topStage.installed()){
		MotorDisplay(141, intakeConveyorCurr, intakeConveyorTemp);
		Brain.Screen.printAt(300, YOFFSET + 141, "IntakeConveyor");
	} else {
		Brain.Screen.printAt(5, YOFFSET + 141, "IntakeConveyor - DISCONNECTED");
	}
}

//MARK: Auton FR XD

void pre_auton(void) {
  drawGUI();
  Brain.Screen.pressed(selectAuton);
  setBrakeMode(brakeType::brake);
}


void autonomous(void) {
  switch (autonSelected) {
    case 0:
      // Testing Auton
      printf("GO! - Position: %0.2f", leftM.position(rev));
      inchDrive(20);
      wait(2, sec);
      printf("Results - Position: %0.2f", leftM.position(rev));
      printf("GO! - Rotation: %0.2f", inertialSensor.rotation(deg));
      autonTurn(90);
      wait(2, sec);
      printf("Results - Rotation: %0.2f", inertialSensor.rotation(deg));
      break;

//MARK: xx.xx
case 1:
break;

//MARK: xx.xx
case 2:
break;

//MARK: xx.xx
case 3:
break;

//MARK: xx.xx
case 4:
break;

//MARK: xx.xx
case 5:
break;

//MARK: xx.xx
case 6:
break;

//MARK: xx.xx
case 7:
break;

//MARK: xx.xx
case 8:
break;
}
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
//MARK:--
//MARK: Usercontrol
void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    Brain.Screen.clearScreen();
   setBrakeMode(brake);
   // Drive
   while (1) {
    if (guiChanged) {
      if (gui) {
        drawGUI();
      }
      else {
        Display();
      }
      guiChanged = false;
    }
    int lStick = 0;
    int rStick = 0;
    if (arcadeMode == 1) {
      //int axis3 = Controller1.Axis3.position(pct);
      //int axis4 = Controller1.Axis4.position(pct);
      rStick = Controller1.Axis3.position(pct) * 1.25 + Controller1.Axis4.position(pct) * 1.4;
      lStick = Controller1.Axis3.position(pct) * 1.25 - Controller1.Axis4.position(pct) * 1.4;
    }
    else if (arcadeMode == 2) {
      // int axis3 = Controller1.Axis3.position(pct);
      // int axis1 = Controller1.Axis1.position(pct);
      rStick = Controller1.Axis3.position(pct) * 1.25 + Controller1.Axis1.position(pct) * 1.4;
      lStick = Controller1.Axis3.position(pct) * 1.25 - Controller1.Axis1.position(pct) * 1.4;
      /*if (axis3 > axis1) {
        lStick -= Controller1.Axis3.position(pct) * 0.75;
        rStick -= Controller1.Axis3.position(pct) * 0.75;
      }*/
    }
    else {
      rStick = Controller1.Axis2.position(pct);
      lStick = Controller1.Axis3.position(pct);
    }
    if (fabs(lStick) < 5.0) {
      lStick = 0;
      driveTrainStop();
    }
    if (fabs(rStick) < 5.0) {
      rStick = 0;
      driveTrainStop();
    }

    drivePct(lStick, rStick, 10);

    if (Controller1.ButtonR1.pressing()) {
      intake.spin(fwd, 100000, rpm);
    }
    else if (Controller1.ButtonX.pressing()) {
      intake.spin(fwd, -100000, rpm);
    }
    else {
      intake.stop();
    }
    if (Controller1.ButtonR2.pressing()) {
      topStage.spin(fwd, 100000, rpm);
      intake.spin(fwd, 100000, rpm);
    }
    else {
      topStage.stop();
    }
   }
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
