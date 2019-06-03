#include "main.h"
#include "lift.h"

int lift_port = 3;
int lift_final_ratio = 7;

pros::Motor lift (lift_port, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::ADIPort lift_limit ('c', pros::E_ADI_DIGITAL_IN);

Lift::Lift(){
  lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void Lift::auto_lift(){
  lift.move_absolute(150, 25);
  while(!((lift.get_position() < 155) && (lift.get_position() > 145))){
    pros::delay(2);
  }
}

void Lift::set_home(){
  lift.tare_position();
}

void Lift::return_home(){
  lift.move_absolute(0, 100);
}

void Lift::liftup(){
  lift.move_velocity(lift_math());
}

void Lift::liftdown(){
  lift.move_velocity(-lift_math());
}

void Lift::high_cap(){
  lift.move_absolute(590, 100);
}

void Lift::mid_cap(){
  lift.move_absolute(400, 100);
}

void Lift::high_flag(){

}

void Lift::mid_flag(){
  lift.move_absolute(280, 40);
  while(!((lift.get_position() < 285) && (lift.get_position() > 275))){
    pros::delay(2);
  }
}

void Lift::lift_stop(){
  lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lift.move_velocity(0);
}

int Lift::lift_math(){
  return 100;
}

int Lift::get_position(){
  return lift.get_position();
}


int Lift::get_current(){
  return lift.get_current_draw();
}

void Lift::init_Lift(){
  int time = 0;
  while(lift_limit.get_value() == 0 || time < 1000 ){
    time++;
    lift.move_velocity(-100);
  }
  lift.move_velocity(0);
  this->set_home();
}
