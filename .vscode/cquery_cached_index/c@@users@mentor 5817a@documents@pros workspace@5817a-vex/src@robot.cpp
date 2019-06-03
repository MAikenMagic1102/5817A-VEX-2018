#include "main.h"
#include "robot.h"
#include "subsystems/intake.h"
#include "subsystems/wrist.h"
#include "subsystems/lift.h"
#include "subsystems/shooter.h"

Controller controller;
pros::Controller master (pros::E_CONTROLLER_MASTER);

bool  flip_drive = false;
bool  flip_cap = false;
bool  half_flip = false;
bool  flag_select = false;

int flag = 1;

// This is the chosing process for autonomous Thomas !!!









//Motor Port, Motor Gearset, Is motor reversed?, What type of movement? Rotations?/Counts?/Degrees?
int leftdrive_port = 1;
int leftdrive2_port = 9;
int rightdrive_port = -2;
int rightdrive2_port = -10;

int vision_port = 19;

Intake intake_;
Wrist wrist_;
Lift lift_;
Shooter shooter_;

pros::vision_signature_s_t BLUEFLAG;
pros::vision_signature_s_t GREENFLAG;

auto drive = ChassisControllerFactory::create(
  {leftdrive_port, leftdrive2_port}, {rightdrive_port, rightdrive2_port},
  AbstractMotor::gearset::green,
  {4_in, 14.25_in}
);

pros::Vision vision(vision_port);

//Creates a larger slow speed band along joystick axis.
float drive_math(float input){
 float out;
 float sing;

  if(input < 0){
    sing = -1;
    input = -input;
  }
  else {
    sing = 1;
  }

  if (input < .05){
    out = 0;
  }else if(input < .80){
    out = input*.5;
  }else {
    out = input;
  }
  return out*sing;
}


/* ******************** Helper Functions *************************** */

double radtodeg(double rad) { return (double)((rad * 180) / PI); }
double degtorad(double deg) { return (double)((deg * PI) / 180); }
double inchtocm(double inch) { return (double)(inch * 2.54); }
double cmtoinch(double cm) { return (double)(cm / 2.54); }
double stob(double tickst) {
  return (double)(tickst * (((4 * PI) / 360) / ((3.25 * PI) / 360)));
}

/* ******************** Vision Sensor Functions *************************** */

double distanceToObjectInches(pros::vision_object_s_t object, double height_mm) {
  double focal_length_mm = 3.814;
  double image_height_pixels = 400;
  // https://photo.stackexchange.com/questions/12434/how-do-i-calculate-the-distance-of-an-object-in-a-photo
  double sensor_height_mm = 5.465; // guessed pls change if actual value becomes availible
  double distance_mm = 0;
  distance_mm = (focal_length_mm * height_mm * image_height_pixels) / (object.height * sensor_height_mm);
  return cmtoinch(distance_mm / 10); // mm to cm then to inches
}

double distanceToBiggestFlagInches(void) {
  pros::vision_object_s_t object = vision.get_by_sig(0, 1);
  //vision.get_by_sig(0, FLAG_BLUE_SIG); // We should use color codes instead of simple sigs
  double height_mm = 138;
  double ret_val = 0.0;
  ret_val = distanceToObjectInches(object, height_mm);
  return ret_val;
}

void alignWithObject(int sig, double rpm) {
  vision.set_exposure(50);
  double middle = 316 / 2;
  bool linedup = false;
  pros::vision_object_s_t object;
  while (!linedup) {
    if (vision.get_object_count() > 0) {
      object = vision.get_by_sig(0, sig);
      if (object.x_middle_coord < middle - 5) {
        //turnleft(rpm);
      } else if (object.x_middle_coord > middle + 5) {
        //turnright(rpm);
      } else {
        linedup = true;
        //move(0);
      }
    } else {
      //move(0);
    }
  }
}


void init(){
  master.clear();
  wrist_.set_home();
  lift_.set_home();
  vision.clear_led();
  vision.set_auto_white_balance(true);
}

void set_vision_sigs(){
//  data":{"brightness":23,"signatures":[{"name":"1","parameters":{"uMin":-4001,"uMax":-3353,"uMean":-3677,"vMin":-5569,"vMax":-4759,"vMean":-5164,"rgb":5797964.6,"type":0,"name":"1"
      //
       GREENFLAG.id = 1;
       GREENFLAG.range = 6.3;
       GREENFLAG.u_min = -4001;
       GREENFLAG.u_max = -3353;
       GREENFLAG.u_mean = -3677;
       GREENFLAG.v_min = -5569;
       GREENFLAG.v_max = -4759;
       GREENFLAG.v_mean = -5164;
       GREENFLAG.rgb = 5797964;
       GREENFLAG.type = 0;
       vision.set_signature(1, &GREENFLAG);
      //
      // BLUEFLAG.id = 2;
      // BLUEFLAG.range = 3;
      // BLUEFLAG.u_min = -3085;
      // BLUEFLAG.u_max = 1707;
      // BLUEFLAG.u_mean = -2396;
      // BLUEFLAG.v_min = 7691;
      // BLUEFLAG.v_max = 11253;
      // BLUEFLAG.v_mean = 9472;
      // BLUEFLAG.rgb = 1120307;
      // BLUEFLAG.type = 0;
      // vision.set_signature(2, &BLUEFLAG);
}

void driver_control(){


  pros::vision_object_s_t object = vision.get_by_sig(0, 1);

  //Drivetrain Flip Toggle Logic
  if(master.get_digital_new_press(DIGITAL_LEFT)){
      flip_drive = !flip_drive;
  }
  pros::lcd::print(3, "DRIVE FLIPPED?: %d", flip_drive);
  pros::lcd::print(4, "MASTER LEFT STICK: %d", drive_math(controller.getAnalog(ControllerAnalog::leftY)));
  //pros::lcd::print(5, "lift CURRENT DRAW: %d", 0);
  pros::lcd::print(6, "Flip Cap: %d", flip_cap);

  //Drivetrain Flip Check and Math
  if(flip_drive){
    drive.arcade(drive_math(controller.getAnalog(ControllerAnalog::leftY) * -1), drive_math(controller.getAnalog(ControllerAnalog::rightX) * -1), 0);
  }else{
    drive.arcade(drive_math(controller.getAnalog(ControllerAnalog::leftY)), drive_math(controller.getAnalog(ControllerAnalog::rightX)), 0);
  }

  if(master.get_digital(DIGITAL_R1)){
    intake_.intake_fwd();
  }else{
    if(master.get_digital(DIGITAL_R2)){
      intake_.intake_rvs();
    }else{
      intake_.intake_stop();
    }
  }

  if(master.get_digital(DIGITAL_L1)){
    lift_.liftup();
  }else{
    if(master.get_digital(DIGITAL_L2)){
      lift_.liftdown();
    }else{
      if(master.get_digital(DIGITAL_DOWN)){
        lift_.return_home();
      }else{
        if(master.get_digital(DIGITAL_X)){
          lift_.high_cap();
        }else{
          if(master.get_digital(DIGITAL_B)){
            lift_.mid_cap();
          }else{
            lift_.lift_stop();
          }
        }
      }
    }
  }

  if(master.get_digital(DIGITAL_Y)){
    shooter_.fire();
  }else{
      if(master.get_digital_new_press(DIGITAL_UP)){
        shooter_.fire_rot();
      }else{
        shooter_.stop();
      }
  }


  if(master.get_digital_new_press(DIGITAL_A)){
    flip_cap = !flip_cap;
  }

  if(master.get_digital_new_press(DIGITAL_RIGHT)){
    half_flip = !half_flip;
  }

  if(flip_cap){
    wrist_.flip_cap();
  }else{
    if(half_flip){
      wrist_.turn_90();
    }else{
      wrist_.return_home();
    }
  }

  if(master.get_digital_new_press(DIGITAL_Y)){
    flag_select = !flag_select;
  }
//hit that yeet
  if(flag_select){
    flag = 2; //Middle flag
  }else{
    flag = 1; //high flag
  }
}
//////////////////////////////////////////////////////////////////////////////
//                             Autonomous'                                  //
//                             selections                                   //
//////////////////////////////////////////////////////////////////////////////
void redTop(){
  //  This is the code for Top Red Side Autonomous -Alex

lift_.auto_lift();
lift_.return_home();


intake_.intake_rvs(); //Start intake

drive.moveDistance(-23_in);

lift_.liftdown();

shooter_.fire();
pros::delay(3000); // Hit Mid Flag (hopefully)
shooter_.stop();

drive.turnAngle(-20_deg); // Turn to hit bottom flag

drive.moveDistance(-19_in); // Drive to hit bottom flag
drive.waitUntilSettled(); // Gay



drive.moveDistance(44_in); // Go to beg
drive.turnAngle(93_deg); // Turn to hit mid cap


drive.moveDistance(-35_in); // Try to hit cap

drive.moveDistance(10_in); // Crap go back
intake_.intake_stop(); // Stop intake cause Thomas
drive.turnAngle(94_deg); // Turn to face platform



drive.moveDistance(-30_in); // CLIMB!!!!!!!!!!

}
void blueTop(){
  lift_.auto_lift();
  lift_.return_home();


  intake_.intake_rvs(); //Start intake

  drive.moveDistance(-23_in); // Go to hit that flag yo

  lift_.liftdown(); // Put lift down cause Thomas

  shooter_.fire();
  pros::delay(3000); // FIRE!!!
  shooter_.stop();


  drive.turnAngle(20_deg); // Turn to hit bottom flag

  drive.moveDistance(-19_in); // Drive to hit bottom flag
  drive.waitUntilSettled(); // Gay



  drive.moveDistance(44_in); // Return to Origin
  drive.turnAngle(-96_deg); // turn to mid cap


  drive.moveDistance(-32_in); // Try to hit that cap yo

  drive.moveDistance(10_in); // Oh crap move back
  intake_.intake_stop();
  drive.turnAngle(-94_deg); // Turn to platform

  drive.moveDistance(-30_in); // CLIMB!

}
void doNothing(){

// Wow wonder what this does -_-

 drive.moveDistance(1_in);
 drive.waitUntilSettled();
}
void blueBot(){

intake_.intake_rvs();

drive.moveDistance(-37_in); // That cap be looking like a fine treet
drive.waitUntilSettled(); // Gay

drive.moveDistance(5_in); // Obtain ball
drive.waitUntilSettled();

intake_.intake_stop(); // "STOP" -Thomas

drive.turnAngle(90_deg); // Turn to Big mountain

drive.moveDistance(-30_in); // CLIMBBBB

}
void redBot(){

  intake_.intake_rvs(); // Start that intake boi

  drive.moveDistance(-37_in); // Obtain the ballllllllllllllllllllll
  drive.waitUntilSettled(); // Gay

  drive.moveDistance(5_in); // Go back
  drive.waitUntilSettled(); // Gay

  intake_.intake_stop(); // Thomas

  drive.turnAngle(-90_deg); // Turn to big platform

  drive.moveDistance(-30_in); // Climb!!!!!!!!!!

 }











//                      -FIN
