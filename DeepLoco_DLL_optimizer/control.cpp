#include "control.hpp"
#include "state.hpp"

#define STEP_LENGTH	0.8 //1.2
#define HIP_OFFSET	0.12
#define X 		0
#define Y 		1
#define Z 		2

static double p01[6] = {0};
static double heading[2] = {0};

void defineHeading()
{
  // Gives a heading vector to follow in the horizontal plane in world coordinates
  double norm = 0;
  heading[0] = 1; // x
  heading[1] = 0; // z
  norm = sqrt(pow(heading[0],2) + pow(heading[1],2));
  if(norm){heading[0] /= norm; heading[1] /= norm;}
  return;
}

double* getHeading()
{
  return heading;
}

double* getFeetTargets()
{
  return p01;
}

void getRootPositionInPlane(double v[3], Supervisor *robot)
{
  Node *root = robot->getFromDef("root");
  const double *position = root->getPosition();
  v[X] = position[X]; // x
  v[Z] = position[Z]; // z
}

void setFirstSteps(Supervisor *robot)
{
  //Define first 2 targets at begining for swinging foot (left) and standing foot (right)
  double ro[3] = {0}; 
  getRootPositionInPlane(ro,robot);

  //Perpendicular vector to heading, used for hip offset in foot target placement
  double perpHeading[2] = {-heading[1], heading[0]};
  
  //Swinging foot
  p01[X] = ro[X] - perpHeading[0]*HIP_OFFSET + heading[0]*0.7;
  p01[Z] = ro[Z] - perpHeading[1]*HIP_OFFSET + heading[1]*0.7;
  //Standing foot
  p01[3+X] = ro[X] + perpHeading[0]*HIP_OFFSET + heading[0]*1.0;
  p01[3+Z] = ro[Z] + perpHeading[1]*HIP_OFFSET + heading[1]*1.0;
  
  double t0[3] = {p01[0],p01[1],p01[2]};
  double t1[3] = {p01[3+0],p01[3+1],p01[3+2]};
  Node* target0 = robot->getFromDef("Target0");
  Node* target1 = robot->getFromDef("Target1");
  Field* tar0 = target0->getField("translation");
  if(tar0 != NULL)
    tar0->setSFVec3f(t0);
  Field* tar1 = target1->getField("translation");
  if(tar1 != NULL)
    tar1->setSFVec3f(t1);
  return;
}

void planFootstep(bool stance, Supervisor *robot)
{
  //Define target for currently standing foot (p1)
  double ro[3] = {0}; 
  getRootPositionInPlane(ro,robot);
  double perpHeading[2] = {-heading[1], heading[0]};
  
  //p1 -> p0
  p01[X] = p01[3+X];
  p01[Z] = p01[3+Z];
  //New p1
  if(stance == RIGHT_STANCE)
  {
    p01[3+X] = ro[X] + perpHeading[0]*HIP_OFFSET + heading[0]*STEP_LENGTH;
    p01[3+Z] = ro[Z] + perpHeading[1]*HIP_OFFSET + heading[1]*STEP_LENGTH;
  }
  else if(stance == LEFT_STANCE)
  {
    p01[3+X] = ro[X] - perpHeading[0]*HIP_OFFSET + heading[0]*STEP_LENGTH;
    p01[3+Z] = ro[Z] - perpHeading[1]*HIP_OFFSET + heading[1]*STEP_LENGTH;
  }
  
  double t0[3] = {p01[0],p01[1],p01[2]};
  double t1[3] = {p01[3+0],p01[3+1],p01[3+2]};
  placeTargetIndicators(robot,t0,t1);
  
  return;
}

void placeTargetIndicators(Supervisor *robot, double t0[3], double t1[3])
{
  //Place indicators where the robot should step
  Node* target0 = robot->getFromDef("Target0");
  Node* target1 = robot->getFromDef("Target1");
  Field* tar0 = target0->getField("translation");
  if(tar0 != NULL)
    tar0->setSFVec3f(t0);
  Field* tar1 = target1->getField("translation");
  if(tar1 != NULL)
    tar1->setSFVec3f(t1);
}