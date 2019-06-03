#ifndef ROBOT_H
#define ROBOT_H

#include "main.h"

void init();

float drive_math();

void driver_control();

void automode();
void redTop();
void redBot();
void blueTop();
void blueBot();
void doNothing();

// Helper Functions
double radtodeg(double rad);
double degtorad(double deg);
double inchtocm(double inch);
double cmtoinch(double cm);

// Vision Sensor Functions
double distanceToObjectInches(pros::vision_object_s_t object, double height_mm);
double distanceToBiggestFlagInches();
void alignWithObject(int sig, double rpm);

void set_vision_sigs();


#endif
