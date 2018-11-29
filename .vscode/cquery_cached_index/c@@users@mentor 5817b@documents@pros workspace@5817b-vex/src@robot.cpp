#include "main.h"
#include "robot.h"
#include "subsystems/intake.h"
#include "subsystems/wrist.h"
#include "subsystems/arm.h"
#include "subsystems/shooter.h"

Controller controller;
pros::Controller master (pros::E_CONTROLLER_MASTER);

bool  flip_drive = false;
bool  flip_cap = false;
bool  flag_select = false;

int flag = 1;
int distance = 1;

//Motor Port, Motor Gearset, Is motor reversed?, What type of movement? Rotations?/Counts?/Degrees?
int leftdrive_port = 18;
int rightdrive_port = 19;

Intake intake;
Wrist wrist_;
Arm arm_;
Shooter shooter_;

auto drive = ChassisControllerFactory::create(
  leftdrive_port, rightdrive_port,
  AbstractMotor::gearset::red,
  {4_in, 16_in}
);

float drive_math(float input){
 float out;
 float sing;

 //remember sing
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


void init(){
  master.clear();
  wrist_.set_home();
}

void driver_control(){

  //Drivetrain Flip Toggle Logic
  if(master.get_digital_new_press(DIGITAL_UP)){
      flip_drive = !flip_drive;
  }
  pros::lcd::print(3, "DRIVE FLIPPED?: %d", flip_drive);
  pros::lcd::print(4, "MASTER LEFT STICK: %d", drive_math(controller.getAnalog(ControllerAnalog::leftY)));
  //Drivetrain Flip Check and Math
  if(flip_drive){
    drive.tank(drive_math(controller.getAnalog(ControllerAnalog::rightY) * -1), drive_math(controller.getAnalog(ControllerAnalog::leftY)), 0);
  }else{
    drive.tank(drive_math(controller.getAnalog(ControllerAnalog::leftY)), drive_math(controller.getAnalog(ControllerAnalog::rightY) * -1), 0);
  }

  if(master.get_digital(DIGITAL_R1)){
    intake.intake_fwd();
  }else{
    if(master.get_digital(DIGITAL_R2)){
      intake.intake_rvs();
    }else{
      intake.intake_stop();
    }
  }

  if(master.get_digital(DIGITAL_L1)){
    arm_.liftup();
  }else{
    if(master.get_digital(DIGITAL_L2)){
      arm_.liftdown();
    }else{
      arm_.arm_stop();
    }
  }

  if(master.get_digital_new_press(DIGITAL_DOWN)){
    flip_cap = !flip_cap;
  }

  if(flip_cap){
    wrist_.flip_cap();
  }else{
    wrist_.return_home();
  }

  if(master.get_digital_new_press(DIGITAL_Y)){
    flag_select = !flag_select;
  }

  if(flag_select){
    flag = 2; //Middle flag
  }else{
    flag = 1; //high flag
  }

  //Distance 1 = close, Distance 2 = midrange, Distance = 3, full field
  if(master.get_digital_new_press(DIGITAL_X)){
    if(distance < 3)
      distance++;
  }
  if(master.get_digital_new_press(DIGITAL_B)){
    if(distance > 1)
      distance--;
  }

  shooter_.hood_setAngle(flag, distance);
  master.print(2, 0,"Flag %d  Dist %d\n", (flag), (distance));
}

void automode(){

}
