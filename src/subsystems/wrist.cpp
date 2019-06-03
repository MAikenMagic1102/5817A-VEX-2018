#include "main.h"
#include "wrist.h"

int wrist_port = 5;
pros::Motor wrist (wrist_port, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

Wrist::Wrist(){}

void Wrist::set_home(){
  wrist.tare_position();
}

void Wrist::flip_cap(){
  wrist.move_absolute(180, 100);
}

void Wrist::turn_90(){
  wrist.move_absolute(90, 100);
}

void Wrist::return_home(){
  wrist.move_absolute(0, -100);
}
