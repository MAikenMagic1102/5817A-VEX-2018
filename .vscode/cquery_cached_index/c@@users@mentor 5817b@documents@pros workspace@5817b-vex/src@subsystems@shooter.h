#ifndef SHOOTER_H
#define SHOOTER_H

class Shooter{
public:
  Shooter();

  void hood_sethome();

  void hood_setAngle(int flag, int distance);

  void shooter_spinup();

  void shooter_shutdown();
};

#endif
