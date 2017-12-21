// File: Optimizer.cpp
// Date:
// Description:
// Author:
// Modifications:

/*
// INTERFACE WEBOTS
pRevertSimulation();    JE METS CA OÙ MOI ?!
pNewCycle();
*/

//#define DEBUG_SPEC

#include <iostream>
#include <fstream>
#include <math.h>

#include "state.hpp"  //Reponsible for robot's state observation
#include "apply.hpp"  //Responsible for receiving Network's answer and sending motor commands
#include "softRevert.hpp"

#include "wrapper.hpp" //Allows to use Deeploco.dll

#define CONTROL_PERIOD 33.33 //30 Hz expressed as a period of 33.33 ms

// All the webots classes are defined in the "webots" namespace
using namespace webots;

Supervisor *robot = new Supervisor();
double timeStep = robot->getBasicTimeStep(); //Simulation runs at 1 kHz
//Initialise state vector for controller input (size 125)
double state[INPUT_STATE_SIZE] = {0};
//Initialise command vector for controller output (size 22)
double command[OUTPUT_STATE_SIZE] = {0};
//Initialise converted command vector for controller output (size 17)
double convCom[OUTPUT_CONV_STATE_SIZE] = {0};

//Timer for Controller (called at 30 Hz)
double timer = 0;

//Use the torso node for fall detection
Node *tt = robot->getFromDef("torsoTop");

//Default target indicators localisation
double indLoc[3] = {0,-1,0};

void interfaceTest()
{
  // This function is called from the DLL
  printf("Interface between DLL and Optimizer operational.\n");
}
void stepSimulation()
{
  //printf("Stepping simulation\n");
  if(robot->step(33*timeStep) == -1)
    exit(EXIT_SUCCESS);
  else
    timer += 20*timeStep; // factor set by hand... 
}
void newCycle()
{
  //printf("newCycle()\n");
  timer = 0;
}
void getState(double state[INPUT_STATE_SIZE])
{
  //Create state vector
  //printf("Getting state:\n");
  buildStateVector(state,robot,timer);
  //printf("State built\n");
  //displayInput(state);
}

bool verifyValidity(double a[OUTPUT_STATE_SIZE])
{
  for(int i=0;i<OUTPUT_STATE_SIZE;i++)
  {
#ifdef DEBUG_SPEC
    std::cout<<a[i]<<std::endl;
#endif
    if(abs(a[i]) < 0.000000001 || abs(a[i]) > 1000 || a[i] != a[i]) 
      return 0;
  }
  return 1;
}

int hack = 1;
//TODO: improve HACK -> done!
void getActionAndApply(double action[OUTPUT_STATE_SIZE])
{
  if(verifyValidity(action))
  {
    //Retreive action from network
    axisAngleToEuler(action,convCom);
#ifdef DEBUG_SPEC
    displayOutput17(convCom);
#endif

    //Apply target positions to robot's motors
    applyMotorsTargets(convCom);

    hack = 0;
  }
  else
    hack = 1;
}
bool detectFall()
{
  int fall = tt->getNumberOfContactPoints();
  const double* com = tt->getCenterOfMass();
  //printf("Number contacts: %d\n",numContacts);
  if(fall > 0 || com[1] < 0.8)
    return 1;
  else
    return 0;
}
void revertSimulation() // soft-revert
{
  printf("revertSimulation()\n");
  placeTargetIndicators(robot,indLoc,indLoc);
  softRevert(robot);
  setStartSpeed(robot);

  defineHeading();
  setFirstSteps(robot);

  newCycle();
  resetPhase();
}

std::ofstream outfile;
int counter = 0;
void saveReward(double reward)
{
  if(counter >= 1000)
  {
    outfile << reward << std::endl;
    counter = 0;
  }
  else
    counter++;
  return;
}

int main(int argc, char **argv)
{
  // Init file for reward
  outfile.open("reward.txt", std::ofstream::out | std::ofstream::trunc);

  //Deliver to the DLL functions to control Webots
  getFuncPointers(interfaceTest,
                  stepSimulation,
                  newCycle,
                  getState,
                  getActionAndApply,
                  detectFall,
                  revertSimulation,
                  saveReward);
  //Verify the pointers in the DLL are operational
  checkFunctionsPointers();
  
  printf("TimeStep: %.3lf [ms]\n",timeStep); // 1.000

  getMotorsAndSensors(robot,timeStep);
  
  //Give an initial impulse to the robot at start
  setStartSpeed(robot);
  init_softRevert(robot);
  
  //Define initial heading and give first 2 targets for feet positions
  defineHeading();
  setFirstSteps(robot);
  
  //Define max speed for motors
  //setMotorsVelocity(robot);

  //Tune the PID controller of each joint
  gainTuningForDefaultPIDVelocityControl();

  //Initialise stuff for Optimizer_Webots.dll
  cParseArgs(argc,argv);
  //Scenario set up for learning
  cSetupScenario();
  //Hack to initialise the model with a trained policy
  cInitCaffe();
  //Let the Optimizer do its job
  cRun();
  //Close everything
  cCleanUp();
  
  /*****************CLEAN UP AND EXIT******************/
  outfile.close();
  disableSensors();
  delete robot;
  return 0;
}

/* (deprecated)
//Call Neural Networks to calculate output
cEvaluateNetwork(state,command);
//displayOutput22(command);

//Command conversion to adequate representation
axisAngleToEuler(command,convCom);
//displayOutput17(convCom);

//Apply target positions to robot's motors
applyMotorsTargets(convCom); 
*/