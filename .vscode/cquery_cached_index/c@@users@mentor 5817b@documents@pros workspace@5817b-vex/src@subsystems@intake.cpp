#include "main.h"
#include "Intake.h"


int intake_port = 2;
int feeder_port = 3;

pros::Motor intake1 (intake_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor feeder (feeder_port, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_ROTATIONS);

  Intake::Intake(){}

  void Intake::intake_fwd(){
    intake1.move_velocity(600);
  }

  void Intake::intake_rvs(){
    intake1.move_velocity(-600);
  }

  void Intake::feeder_fwd(){
    feeder.move_velocity(600);
  }

  void Intake::feeder_rvs(){
    feeder.move_velocity(-600);
  }

  void Intake::intake_stop(){
    intake1.move_velocity(0);
  }

  void Intake::feeder_stop(){
    feeder.move_velocity(0);
  }
