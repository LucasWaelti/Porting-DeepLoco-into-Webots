#include "apply.hpp"

#define PI 3.141592653589793
#define LT 12 //[ms]

//Torso
Motor *mTorsoY;
Motor *mTorsoZ;
Motor *mTorsoX;
//Left leg
Motor *mLeHipX;
Motor *mLeHipY;
Motor *mLeHipZ;
Motor *mLeKnee;
Motor *mLeAnkleY;
Motor *mLeAnkleX;
Motor *mLeAnkleZ;
//Right leg
Motor *mRiHipX;
Motor *mRiHipY;
Motor *mRiHipZ;
Motor *mRiKnee;
Motor *mRiAnkleY;
Motor *mRiAnkleX;
Motor *mRiAnkleZ;

//Torso
PositionSensor *sTorsoY;
PositionSensor *sTorsoZ;
PositionSensor *sTorsoX;
//Left leg
PositionSensor *sLeHipX;
PositionSensor *sLeHipY;
PositionSensor *sLeHipZ;
PositionSensor *sLeKnee;
PositionSensor *sLeAnkleY;
PositionSensor *sLeAnkleX;
PositionSensor *sLeAnkleZ;
//Right leg
PositionSensor *sRiHipX;
PositionSensor *sRiHipY;
PositionSensor *sRiHipZ;
PositionSensor *sRiKnee;
PositionSensor *sRiAnkleY;
PositionSensor *sRiAnkleX;
PositionSensor *sRiAnkleZ;

double previous_error_mTorsoY = 0;
double previous_error_mTorsoZ = 0;
double previous_error_mTorsoX = 0;
double previous_error_mLeHipX = 0;
double previous_error_mLeHipY = 0;
double previous_error_mLeHipZ = 0;
double previous_error_mLeKnee = 0;
double previous_error_mLeAnkleY = 0;
double previous_error_mLeAnkleX = 0;
double previous_error_mLeAnkleZ = 0;
double previous_error_mRiHipX = 0;
double previous_error_mRiHipY = 0;
double previous_error_mRiHipZ = 0;
double previous_error_mRiKnee = 0;
double previous_error_mRiAnkleY = 0;
double previous_error_mRiAnkleX = 0;
double previous_error_mRiAnkleZ = 0;

void getMotorsAndSensors(Supervisor *robot, double timeStep)
{
  //Torso
  mTorsoY = robot->getMotor("torso_y");//  YAW
  mTorsoZ = robot->getMotor("torso_z");//PITCH
  mTorsoX = robot->getMotor("torso_x");// ROLL
  //Left leg
  mLeHipX = robot->getMotor("left_hip_x");
  mLeHipY = robot->getMotor("left_hip_y");
  mLeHipZ = robot->getMotor("left_hip_z");
  mLeKnee = robot->getMotor("left_knee_z");
  mLeAnkleY = robot->getMotor("left_ankle_y");
  mLeAnkleX = robot->getMotor("left_ankle_x");
  mLeAnkleZ = robot->getMotor("left_ankle_z");
  //Right leg
  mRiHipX = robot->getMotor("right_hip_x");
  mRiHipY = robot->getMotor("right_hip_y");
  mRiHipZ = robot->getMotor("right_hip_z");
  mRiKnee = robot->getMotor("right_knee_z");
  mRiAnkleY = robot->getMotor("right_ankle_y");
  mRiAnkleX = robot->getMotor("right_ankle_x");
  mRiAnkleZ = robot->getMotor("right_ankle_z");

  //Torso
  sTorsoY = robot->getPositionSensor("s_torso_y");
  sTorsoZ = robot->getPositionSensor("s_torso_z");
  sTorsoX = robot->getPositionSensor("s_torso_x");
  //Left leg
  sLeHipX = robot->getPositionSensor("s_left_hip_x");
  sLeHipY = robot->getPositionSensor("s_left_hip_y");
  sLeHipZ = robot->getPositionSensor("s_left_hip_z");
  sLeKnee = robot->getPositionSensor("s_left_knee_z");
  sLeAnkleY = robot->getPositionSensor("s_left_ankle_y");
  sLeAnkleX = robot->getPositionSensor("s_left_ankle_x");
  sLeAnkleZ = robot->getPositionSensor("s_left_ankle_z");
  //Right leg
  sRiHipX = robot->getPositionSensor("s_right_hip_x");
  sRiHipY = robot->getPositionSensor("s_right_hip_y");
  sRiHipZ = robot->getPositionSensor("s_right_hip_z");
  sRiKnee = robot->getPositionSensor("s_right_knee_z");
  sRiAnkleY = robot->getPositionSensor("s_right_ankle_y");
  sRiAnkleX = robot->getPositionSensor("s_right_ankle_x");
  sRiAnkleZ = robot->getPositionSensor("s_right_ankle_z");
  
  //Enable all sensors
  sTorsoX->enable((int)timeStep);
  sTorsoY->enable((int)timeStep);
  sTorsoZ->enable((int)timeStep);
  sLeHipX->enable((int)timeStep);
  sLeHipY->enable((int)timeStep);
  sLeHipZ->enable((int)timeStep);
  sLeKnee->enable((int)timeStep);
  sLeAnkleY->enable((int)timeStep);
  sLeAnkleX->enable((int)timeStep);
  sLeAnkleZ->enable((int)timeStep);
  sRiHipX->enable((int)timeStep);
  sRiHipY->enable((int)timeStep);
  sRiHipZ->enable((int)timeStep);
  sRiKnee->enable((int)timeStep);
  sRiAnkleY->enable((int)timeStep);
  sRiAnkleX->enable((int)timeStep);
  sRiAnkleZ->enable((int)timeStep);
}
void disableSensors()
{
  sTorsoY->disable();
  sTorsoZ->disable();
  sTorsoX->disable();
  sLeHipX->disable();
  sLeHipY->disable();
  sLeHipZ->disable();
  sLeKnee->disable();
  sLeAnkleY->disable();
  sLeAnkleX->disable();
  sLeAnkleZ->disable();
  sRiHipX->disable();
  sRiHipY->disable();
  sRiHipZ->disable();
  sRiKnee->disable();
  sRiAnkleY->disable();
  sRiAnkleX->disable();
  sRiAnkleZ->disable();
}

void setStartSpeed(Supervisor* robot)
{
  Node *root = robot->getFromDef("root");
  Node *tt = robot->getFromDef("torsoTop");
  Node *rh = robot->getFromDef("rightHip");
  Node *rk = robot->getFromDef("rightKnee");
  Node *ra = robot->getFromDef("rightAnkle");
  Node *lh = robot->getFromDef("leftHip");
  Node *lk = robot->getFromDef("leftKnee");
  Node *la = robot->getFromDef("leftAnkle");
  
  const double vel_root[6] = {1.58487,-0.0521823,0.139421,
                             -1.33136,0.691321,-0.193296}; 
  root->setVelocity(vel_root);
  
  const double vel_tt[6] = {1.60774,-0.0522843,-0.0513435,
                           -0.572099,0.85682,-0.0614508};
  tt->setVelocity(vel_tt);
  
  const double vel_rh[6] = {1.50447,0.00503787,0.109682,
                            -0.507421,2.01419,-0.595114};
  rh->setVelocity(vel_rh);
  
  const double vel_rk[6] = {0.876565,-0.134234,0.0222482,
                            -0.308426,2.18827,-2.41999};
  rk->setVelocity(vel_rk);
  
  const double vel_ra[6] = {0.285568,-0.428065,-0.0300882,
                            0.299452,0.629367,-3.65282};
  ra->setVelocity(vel_ra);
  
  const double vel_lh[6] = {1.72119,-0.223083,-0.0377219,
                            1.10552,0.190828,0.979251};
  lh->setVelocity(vel_lh);
  
  const double vel_lk[6] = {1.272,0.206384,-0.245577,
                            0.166101,0.289852,-4.21494};
  lk->setVelocity(vel_lk);
  
  const double vel_la[6] = {0.337835,0.513694,-0.173132,
                            -0.882759,-0.723344,-5.60772};
  la->setVelocity(vel_la);
}

void scaleNetworkOutput(double com[22]) // Not used here
{
  // This is actually already applied to the output in function Eval() of file NeuralNet.cpp
  double scale[22] = {0.250000, 1.000000, 1.000000, 1.000000, 0.097276, 1.000000, 1.000000, 
                      1.000000, 0.159236, 0.159236, 1.000000, 1.000000, 1.000000, 0.097276, 
                      1.000000, 1.000000, 1.000000, 0.159236, 0.159236, 1.000000, 1.000000, 
                      1.000000};
  double offset[22]= {0.000000, 0.000000, 0.000000, -0.200000, 0.000000, 0.000000, 0.000000, 
                     -0.200000, 1.570000, 0.000000, 0.000000, 0.000000, -0.200000, 0.000000, 
                      0.000000, 0.000000, -0.200000, 1.570000, 0.000000, 0.000000, 0.000000, 
                     -0.200000};
  int i = 0;
  //for(i=0;i<22;i++)
  //  com[i] = scale[i]*(com[i] + offset[i]); // -> Wrong way around!!
  for(i=0;i<22;i++)
    com[i] = com[i]/scale[i] - offset[i]; // This is actually already applied to the output 
  
  return;
}

void displayOutput22(double output[OUTPUT_STATE_SIZE])
{
  int i = 0;
  for(i=0;i<OUTPUT_STATE_SIZE;i++)
    std::cout << "[Output (Axis-Angle)] " << output[i] << std::endl;
}

void displayOutput17(double output[OUTPUT_CONV_STATE_SIZE])
{
  int i = 0;
  for(i=0;i<OUTPUT_CONV_STATE_SIZE;i++)
    std::cout << "[Output (Euler)] " << output[i] << std::endl;
}

void convert(double axisAngle[4], double rotations[3])
{
    //Recreates the global rotation matrix from the angle-axis representation
    //Calculates the respective angle rotations in the Euler representation (ZYX convention)
    double norm = 0;
    norm = sqrt(pow(axisAngle[0],2) + pow(axisAngle[1],2) + pow(axisAngle[2],2));
    //norm = 1;
    axisAngle[0] /= norm; axisAngle[1] /= norm; axisAngle[2] /= norm;
    
    double nx = axisAngle[0]*axisAngle[0]*(1-cos(axisAngle[3])) + cos(axisAngle[3]);
    double ny = axisAngle[0]*axisAngle[1]*(1-cos(axisAngle[3])) + axisAngle[2]*sin(axisAngle[3]);
    double nz = axisAngle[0]*axisAngle[2]*(1-cos(axisAngle[3])) - axisAngle[1]*sin(axisAngle[3]);

    //double sx = axisAngle[1]*axisAngle[0]*(1-cos(axisAngle[3])) - axisAngle[2]*sin(axisAngle[3]);
    //double sy = axisAngle[1]*axisAngle[1]*(1-cos(axisAngle[3])) + cos(axisAngle[3]);
    double sz = axisAngle[1]*axisAngle[2]*(1-cos(axisAngle[3])) + axisAngle[0]*sin(axisAngle[3]);

    //double ax = axisAngle[0]*axisAngle[2]*(1-cos(axisAngle[3])) + axisAngle[1]*sin(axisAngle[3]);
    //double ay = axisAngle[1]*axisAngle[2]*(1-cos(axisAngle[3])) - axisAngle[0]*sin(axisAngle[3]);
    double az = axisAngle[2]*axisAngle[2]*(1-cos(axisAngle[3])) + cos(axisAngle[3]);

    //Angles around basis vectors
    double X = atan2(sz,az);
    double Y = asin(-nz);
    double Z = atan2(ny,nx);

    rotations[0] = X;
    rotations[1] = Y;
    rotations[2] = Z;
    
    return;
}

void axisAngleToEuler(double command[OUTPUT_STATE_SIZE], 
                      double convCom[OUTPUT_CONV_STATE_SIZE])
{
  //Given part order (ID): 0-root, 1-torso, 2-right_hip, 3-right_knee, 
  //		    4-right_ankle, 5-left_hip, 6-left_knee, 7-left_ankle
  //Joint order: 0-torso, 1-rightHip, 2-rightKnee, 3-rightAnkle
  //	     4-leftHip, 5-leftKnee, 6-leftAnkle
  double aa[4] = {0}, euler[3] = {0};
  int i = 0, part = 0, com = 0, conv = 0;
  
  for(part=0;part<NUM_PARTS-1;part++)
  {
    if(part==2 || part==5) //For both knees (right then left)
    {
      convCom[conv] = command[com]; //Directly give z value
      //printf("Knee angle: %lf (%d)\n", convCom[conv],com);
      conv++; com++;
    }
    
    else
    {
      aa[0] = command[com + 1]; 
      aa[1] = command[com + 2];
      aa[2] = command[com + 3];
      aa[3] = command[com + 0];//Angle
      convert(aa,euler);
      for(i=0;i<3;i++)
        convCom[conv + i] = euler[i];
      conv += 3; com += 4;
    }
  }
  
  return;
}

void setMotorsVelocity(Supervisor *robot)// Not used
{
  mTorsoX->setVelocity(MOTOR_MAX_SPEED);
  mTorsoY->setVelocity(MOTOR_MAX_SPEED);
  mTorsoZ->setVelocity(MOTOR_MAX_SPEED); 
  
  mRiHipX->setVelocity(MOTOR_MAX_SPEED);
  mRiHipY->setVelocity(MOTOR_MAX_SPEED);
  mRiHipZ->setVelocity(MOTOR_MAX_SPEED);
  
  mRiKnee->setVelocity(MOTOR_MAX_SPEED);
  
  mRiAnkleX->setVelocity(MOTOR_MAX_SPEED);
  mRiAnkleY->setVelocity(MOTOR_MAX_SPEED);
  mRiAnkleZ->setVelocity(MOTOR_MAX_SPEED);
  
  mLeHipX->setVelocity(MOTOR_MAX_SPEED);
  mLeHipY->setVelocity(MOTOR_MAX_SPEED);
  mLeHipZ->setVelocity(MOTOR_MAX_SPEED);
  
  mLeKnee->setVelocity(MOTOR_MAX_SPEED);
  
  mLeAnkleX->setVelocity(MOTOR_MAX_SPEED);
  mLeAnkleY->setVelocity(MOTOR_MAX_SPEED);
  mLeAnkleZ->setVelocity(MOTOR_MAX_SPEED);
}

void clampCommands(double c[OUTPUT_CONV_STATE_SIZE])
{
  //Make sure the position commands do not exceed allowed range
  if(c[0]>0.5)//Torso x
    c[0] = 0.5;
  else if(c[0]<-0.5)
    c[0] = -0.5;
    
  if(c[1]>1)//Torso y
    c[1] = 1;
  else if(c[1]<-1)
    c[1] = -1;
    
  if(c[2]>1)//Torso z
    c[2] = 1;
  else if(c[2]<-1)
    c[2] = -1;
  
  if(c[3]>1.2)//Right hip x
    c[3] = 1.2;
  else if(c[3]<-1.2)
    c[3] = -1.2;
    
  if(c[4]>1)//Right hip y
    c[4] = 1;
  else if(c[4]<-1)
    c[4] = -1;
    
  if(c[5]>1)//Right hip z
    c[5] = 1;
  else if(c[5]<-1)
    c[5] = -1;
  
  if(c[6]>0)//Right knee
    c[6] = 0;
  else if(c[6]<-3.14)
    c[6] = -3.14;
    
  if(c[7]>1)//Right ankle x
    c[7] = 1;
  else if(c[7]<-1)
    c[7] = -1;
    
  if(c[8]>1)//Right ankle y
    c[8] = 1;
  else if(c[8]<-1)
    c[8] = -1;
    
  if(c[9]>1.57)//Right ankle z
    c[9] = 1.57;
  else if(c[9]<-1.57)
    c[9] = -1.57;
    
  if(c[10]>1.2)//Left hip x
    c[10] = 1.2;
  else if(c[10]<-1.2)
    c[10] = -1.2;
    
  if(c[11]>1)//Left hip y
    c[11] = 1;
  else if(c[11]<-1)
    c[11] = -1;
    
  if(c[12]>1)//Left hip z
    c[12] = 1;
  else if(c[12]<-1)
    c[12] = -1;
    
  if(c[13]>0)//Left knee
    c[13] = 0;
  else if(c[13]<-3.14)
    c[13] = -3.14;
    
  if(c[14]>1)//Left ankle x
    c[14] = 1;
  else if(c[14]<-1)
    c[14] = -1;
    
  if(c[15]>1)//Left ankle y
    c[15] = 1;
  else if(c[15]<-1)
    c[15] = -1;
    
  if(c[16]>1.57)//Left ankle z
    c[16] = 1.57;
  else if(c[16]<-1.57)
    c[16] = -1.57;
}

void applyMotorsTargets(double c[OUTPUT_CONV_STATE_SIZE])
{
  //Given part order (ID): 0-root, 1-torso, 2-right_hip, 3-right_knee, 
  //		    4-right_ankle, 5-left_hip, 6-left_knee, 7-left_ankle
  //Neural net tends to send out of range commands
  clampCommands(c);
  
  mTorsoX->setPosition(c[0]);
  mTorsoY->setPosition(c[1]);
  mTorsoZ->setPosition(c[2]); 
  
  mRiHipX->setPosition(c[3]);
  mRiHipY->setPosition(c[4]);
  mRiHipZ->setPosition(c[5]);
  
  mRiKnee->setPosition(c[6]);
  
  mRiAnkleX->setPosition(c[7]);
  mRiAnkleY->setPosition(c[8]);
  mRiAnkleZ->setPosition(c[9]);
  
  mLeHipX->setPosition(c[10]);
  mLeHipY->setPosition(c[11]);
  mLeHipZ->setPosition(c[12]);
  
  mLeKnee->setPosition(c[13]);
  
  mLeAnkleX->setPosition(c[14]);
  mLeAnkleY->setPosition(c[15]);
  mLeAnkleZ->setPosition(c[16]);
  
  return;
}

//Arg... the torque control implemented in DeepLoco is weird! Nothing to do with what's here. 
//This implementation is completely unstable anyway. 
double calcTorque(double command, double value, double& prev_error,
                  double timeStep, int Kp, int Kd)
{
  timeStep /= 1000;//Convert to [s]

  double error = command - value; 
  double error_derivative = (error - prev_error)/timeStep;
  double torque = Kp*error + Kd*error_derivative;
  prev_error = error;
  return torque;
}
void applyTorquePID(double c[OUTPUT_CONV_STATE_SIZE], double timeStep)
{
  double torsoP = 10, torsoD = 1;
  double hipP = 3, hipD = 0.3;
  double kneeP = 3, kneeD = 0.3;
  double ankleP = 0.1, ankleD = 0.01;
  mTorsoY->setTorque(calcTorque(c[1],sTorsoY->getValue(),previous_error_mTorsoY,timeStep,torsoP,torsoD));
  mTorsoZ->setTorque(calcTorque(c[2],sTorsoZ->getValue(),previous_error_mTorsoZ,timeStep,torsoP,torsoD));
  mTorsoX->setTorque(calcTorque(c[0],sTorsoX->getValue(),previous_error_mTorsoX,timeStep,torsoP,torsoD));
  mLeHipX->setTorque(calcTorque(c[10],sLeHipX->getValue(),previous_error_mLeHipX,timeStep,hipP,hipD));
  mLeHipY->setTorque(calcTorque(c[11],sLeHipY->getValue(),previous_error_mLeHipY,timeStep,hipP,hipD));
  mLeHipZ->setTorque(calcTorque(c[12],sLeHipZ->getValue(),previous_error_mLeHipZ,timeStep,hipP,hipD));
  mLeKnee->setTorque(calcTorque(c[13],sLeKnee->getValue(),previous_error_mLeKnee,timeStep,kneeP,kneeD));
  mLeAnkleY->setTorque(calcTorque(c[15],sLeAnkleY->getValue(),previous_error_mLeAnkleY,timeStep,ankleP,ankleD));
  mLeAnkleX->setTorque(calcTorque(c[14],sLeAnkleX->getValue(),previous_error_mLeAnkleX,timeStep,ankleP,ankleD));
  mLeAnkleZ->setTorque(calcTorque(c[16],sLeAnkleZ->getValue(),previous_error_mLeAnkleZ,timeStep,ankleP,ankleD));
  mRiHipX->setTorque(calcTorque(c[3],sRiHipX->getValue(),previous_error_mRiHipX,timeStep,hipP,hipD));
  mRiHipY->setTorque(calcTorque(c[4],sRiHipY->getValue(),previous_error_mRiHipY,timeStep,hipP,hipD));
  mRiHipZ->setTorque(calcTorque(c[5],sRiHipZ->getValue(),previous_error_mRiHipZ,timeStep,hipP,hipD));
  mRiKnee->setTorque(calcTorque(c[6],sRiKnee->getValue(),previous_error_mRiKnee,timeStep,hipP,hipD));
  mRiAnkleY->setTorque(calcTorque(c[8],sRiAnkleY->getValue(),previous_error_mRiAnkleY,timeStep,ankleP,ankleD));
  mRiAnkleX->setTorque(calcTorque(c[7],sRiAnkleX->getValue(),previous_error_mRiAnkleX,timeStep,ankleP,ankleD));
  mRiAnkleZ->setTorque(calcTorque(c[9],sRiAnkleZ->getValue(),previous_error_mRiAnkleZ,timeStep,ankleP,ankleD));
}

void gainTuningForDefaultPIDVelocityControl()
{
  //Default PID values are PID = 10,0,0 for all joints (ankles were set to 5,0,0)
  //Parameters set in DeepLoco: 
  /*
    torso = 1000,0,100
    hip   = 300,0,30
    knee  = 300,0,30
    ankle = 100,0,10      -> should try to respect this pattern while tuning
  */
  enum enumPID {P,I,D};

  double torsoPID[3] = {20,0,1.511};
  mTorsoY->setControlPID(torsoPID[P],torsoPID[I],torsoPID[D]);
  mTorsoZ->setControlPID(torsoPID[P],torsoPID[I],torsoPID[D]);
  mTorsoX->setControlPID(torsoPID[P],torsoPID[I],torsoPID[D]);

  double hipPID[3] = {8.9705,0,0.0545};
  mLeHipX->setControlPID(hipPID[P],hipPID[I],hipPID[D]);
  mLeHipY->setControlPID(hipPID[P],hipPID[I],hipPID[D]);
  mLeHipZ->setControlPID(hipPID[P],hipPID[I],hipPID[D]);
  mRiHipX->setControlPID(hipPID[P],hipPID[I],hipPID[D]);
  mRiHipY->setControlPID(hipPID[P],hipPID[I],hipPID[D]);
  mRiHipZ->setControlPID(hipPID[P],hipPID[I],hipPID[D]);

  double kneePID[3] = {8.9705,0,0.0545};
  mLeKnee->setControlPID(kneePID[P],kneePID[I],kneePID[D]);
  mRiKnee->setControlPID(kneePID[P],kneePID[I],kneePID[D]);

  double anklePID[3] = {3,0,0.6}; //2.85   2.96,0,0.5512
  mLeAnkleY->setControlPID(anklePID[P],anklePID[I],anklePID[D]);
  mLeAnkleX->setControlPID(anklePID[P],anklePID[I],anklePID[D]);
  mLeAnkleZ->setControlPID(anklePID[P],anklePID[I],anklePID[D]);
  mRiAnkleY->setControlPID(anklePID[P],anklePID[I],anklePID[D]);
  mRiAnkleX->setControlPID(anklePID[P],anklePID[I],anklePID[D]);
  mRiAnkleZ->setControlPID(anklePID[P],anklePID[I],anklePID[D]);

  double maxTorque = 90;
  mLeAnkleY->setAvailableTorque(maxTorque);
  mLeAnkleX->setAvailableTorque(maxTorque);
  mLeAnkleZ->setAvailableTorque(maxTorque);
  mRiAnkleY->setAvailableTorque(maxTorque);
  mRiAnkleX->setAvailableTorque(maxTorque);
  mRiAnkleZ->setAvailableTorque(maxTorque);
}

/*
Backups: 

number of steps: 7
{
  {20,0,0.5}
  {9,0,0}
  {9,0,0}
  {3,0,0.5}
}
number of steps: 3
{
  {20,0,1.511}
  {8.9705,0,0.0545}
  {8.9705,0,0.0545}
  {2.87,0,0.5512}
}

*/