#include "main.h"
#include "shooter.h"

int hood_port = 10;
int shooter_port = 1;

pros::Motor shooter (shooter_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor hood (hood_port, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);

Shooter::Shooter(){}

void Shooter::hood_sethome(){
  hood.tare_position();
}

void Shooter::hood_setAngle(int flag, int distance){
  if(flag == 1){
    if(distance == 1){}

    if(distance == 2){}

    if(distance == 3){}
  }

  if(flag == 2){
    if(distance == 1){}

    if(distance == 2){}

    if(distance == 3){}
  }
}

void Shooter::shooter_spinup(){
  shooter.move_velocity(540);
}

void Shooter::shooter_shutdown(){
  shooter.move_velocity(0);
}
