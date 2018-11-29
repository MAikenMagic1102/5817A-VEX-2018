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
pros::vision_signature_s_t REDFLAG;

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
  lift_.init_Lift();
  lift_.set_home();
}

void set_vision_sigs(){

      REDFLAG.id = 1;
      REDFLAG.range = 3;
      REDFLAG.u_min = 4681;
      REDFLAG.u_max = 7621;
      REDFLAG.u_mean = 6151;
      REDFLAG.v_min = -1117;
      REDFLAG.v_max = -429;
      REDFLAG.v_mean = -773;
      REDFLAG.rgb = 7160124;
      REDFLAG.type = 0;
      vision.set_signature(1, &REDFLAG);

      BLUEFLAG.id = 2;
      BLUEFLAG.range = 3;
      BLUEFLAG.u_min = -3085;
      BLUEFLAG.u_max = 1707;
      BLUEFLAG.u_mean = -2396;
      BLUEFLAG.v_min = 7691;
      BLUEFLAG.v_max = 11253;
      BLUEFLAG.v_mean = 9472;
      BLUEFLAG.rgb = 1120307;
      BLUEFLAG.type = 0;
      vision.set_signature(2, &BLUEFLAG);
}

void driver_control(){



  //Drivetrain Flip Toggle Logic
  if(master.get_digital_new_press(DIGITAL_LEFT)){
      flip_drive = !flip_drive;
  }
  pros::lcd::print(3, "DRIVE FLIPPED?: %d", flip_drive);
  pros::lcd::print(4, "MASTER LEFT STICK: %d", drive_math(controller.getAnalog(ControllerAnalog::leftY)));
  pros::lcd::print(5, "lift CURRENT DRAW: %d", 0);
  pros::lcd::print(6, "Distance to flag INCHES: ", distanceToBiggestFlagInches());

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

  if(flag_select){
    flag = 2; //Middle flag
  }else{
    flag = 1; //high flag
  }
}

void automode(){

}
