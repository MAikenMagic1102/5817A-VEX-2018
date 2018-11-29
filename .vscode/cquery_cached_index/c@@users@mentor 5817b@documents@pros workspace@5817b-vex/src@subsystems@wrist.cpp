#include "main.h"
#include "wrist.h"

int wrist_port = 12;
pros::Motor wrist (wrist_port, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);

Wrist::Wrist(){}

void Wrist::set_home(){
  wrist.tare_position();
}

void Wrist::flip_cap(){
  wrist.move_absolute(180, 100);
}

void Wrist::return_home(){
  wrist.move_absolute(0, -100);
}
