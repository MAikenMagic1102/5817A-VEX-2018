#include "main.h"
#include "arm.h"

int arm_port = 15;
int arm_final_ratio = 5;

pros::Motor arm (arm_port, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);


Arm::Arm(){
  arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void Arm::liftup(){
  arm.move_velocity(arm_math());
}

void Arm::liftdown(){
  arm.move_velocity(-arm_math());
}

void Arm::arm_stop(){
  arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  arm.move_velocity(0);
}

int Arm::arm_math(){
  pros::lcd::print(5, "ARM CURRENT POSITION: %d", arm.get_position());
  if(arm.get_position() < (90 * arm_final_ratio)){
    return 100;
  }else if(arm.get_position() < (150 * arm_final_ratio)){
    return 40;
  }else if(arm.get_position() < (240 * arm_final_ratio)){
    return 20;
  }else{
    return 20;
  }
}
