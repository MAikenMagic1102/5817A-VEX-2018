#include "main.h"
#include "shooter.h"

int shooter_port = 7;

pros::Motor shooter (shooter_port, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_ROTATIONS);

Shooter::Shooter(){}

void Shooter::fire(){
  shooter.move_velocity(100);
}

void Shooter::fire_rot(){
  shooter.move_absolute(120, 100);
}

void Shooter::stop(){
  shooter.move_velocity(0);
  //shooter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}
