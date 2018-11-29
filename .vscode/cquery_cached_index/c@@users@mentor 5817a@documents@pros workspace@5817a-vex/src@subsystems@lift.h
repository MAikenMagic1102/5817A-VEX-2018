#ifndef LIFT_H
#define LIFT_H

class Lift{
public:
  Lift();

  void set_home();

  void return_home();

  void liftup();

  void liftdown();

  void high_cap();

  void mid_cap();

  void high_flag();

  void mid_flag();

  void lift_stop();

  int lift_math();

  int get_position();

  int get_current();

  void init_Lift();

};

#endif
