#include "state.hpp"

#define MOTOR_MAX_SPEED 400 //8.5

void getMotorsAndSensors(Supervisor *robot, double timeStep);
void disableSensors();

void setStartSpeed(Supervisor* robot);

void scaleNetworkOutput(double com[22]);

void displayOutput22(double output[OUTPUT_STATE_SIZE]);
void displayOutput17(double output[OUTPUT_CONV_STATE_SIZE]);

void convert(double axisAngle[4], double rotations[3]); //Checked

void axisAngleToEuler(double command[OUTPUT_STATE_SIZE], 
                      double convCom[OUTPUT_CONV_STATE_SIZE]);
                      
void setMotorsVelocity(Supervisor *robot);
                      
void clampCommands(double c[OUTPUT_CONV_STATE_SIZE]);

void applyMotorsTargets(double command[OUTPUT_CONV_STATE_SIZE]);

void applyTorquePID(double c[OUTPUT_CONV_STATE_SIZE], double timeStep);

void gainTuningForDefaultPIDVelocityControl();