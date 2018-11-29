#include "main.h"
#include "Intake.h"


int intake_port = 8;

pros::Motor intake (intake_port, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_ROTATIONS);

  Intake::Intake(){}

  void Intake::intake_fwd(){
    intake.move_velocity(200);
  }

  void Intake::intake_rvs(){
    intake.move_velocity(-200);
  }

  void Intake::intake_stop(){
    intake.move_velocity(0);
  }
