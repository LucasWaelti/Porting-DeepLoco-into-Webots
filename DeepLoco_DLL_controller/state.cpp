#include "state.hpp"

double phase = 0;		//Defines the progression in the walk cycle
bool stance = RIGHT_STANCE;	//Defines if currently walking on right or left foot
bool old_stance = RIGHT_STANCE;
bool stance_changed = false;
bool foot_landed = false;
int old_phi = 0;

//This array contains the name of all parts
//0-root,1-torso,2-right_hip,3-right_knee,4-right_ankle,5-left_hip,6-left_knee,7-left_ankle
const char parts[8][11] = {"root","torsoTop","rightHip","rightKnee",
                           "rightAnkle","leftHip","leftKnee","leftAnkle"};
//This array also contains the intermediate parts implemented in Webots
const char wParts[13][20] = {"root","torsoBottom","torsoTop","rightHipArt","rightHip",
                             "rightKnee","rightKneeArt","rightAnkle","leftHipArt",
                             "leftHip","leftKnee","leftKneeArt","leftAnkle"};


//////////////////////////////Utilitary functions//////////////////////////////
void displayInput(double state[INPUT_STATE_SIZE])
{
  std::cout << "////////////////////////////////////////////////////////////" << std::endl;
  std::cout << "State vector:" << std::endl;
  int i = 0;
  for(i=0;i<INPUT_STATE_SIZE;i++)
    std::cout << state[i] << std::endl;
}
void transpose(const double* root, double tRoot[9])
{
  //tRoot = rootT
  tRoot[0] = root[0]; tRoot[1] = root[3]; tRoot[2] = root[6];
  tRoot[3] = root[1]; tRoot[4] = root[4]; tRoot[5] = root[7];
  tRoot[6] = root[2]; tRoot[7] = root[5]; tRoot[8] = root[8];
}
void transpose(double root[9], double tRoot[9])//1 overload to get rid of const array
{
  //tRoot = root^T
  tRoot[0] = root[0]; tRoot[1] = root[3]; tRoot[2] = root[6];
  tRoot[3] = root[1]; tRoot[4] = root[4]; tRoot[5] = root[7];
  tRoot[6] = root[2]; tRoot[7] = root[5]; tRoot[8] = root[8];
}
void mult(const double* rotPart, double tRoot[9], double relRot[9])
{
  //relRot = rotPart * tRoot (3x3 matrix only)
  int i=0,j=0,k=0;
  for(i=0;i<7;i+=3)//For each line of rotPart		i = 0,3,6
    for(j=0;j<3;j++)//For each column of tRoot	j = 0,1,2
      for(k=0;k<3;k++)//For each element		k = 0,1,2
        relRot[i+j] += rotPart[i+k]*tRoot[j+3*k];
}
void mult(const double* rotPart, const double tRoot[9], double relRot[9])
{
  //relRot = rotPart * tRoot (3x3 matrix only)
  int i=0,j=0,k=0;
  for(i=0;i<7;i+=3)//For each line of rotPart		i = 0,3,6
    for(j=0;j<3;j++)//For each column of tRoot	j = 0,1,2
      for(k=0;k<3;k++)//For each element		k = 0,1,2
        relRot[i+j] += rotPart[i+k]*tRoot[j+3*k];
}
void mult(double rotPart[9], double tRoot[9], double relRot[9])//1 overload to get rid of const array
{
  //relRot = rotPart * tRoot (3x3 matrix only)
  int i=0,j=0,k=0;
  for(i=0;i<7;i+=3)//For each line of rotPart		i = 0,3,6
    for(j=0;j<3;j++)//For each column of tRoot	j = 0,1,2
      for(k=0;k<3;k++)//For each element		k = 0,1,2
        relRot[i+j] += rotPart[i+k]*tRoot[j+3*k];
}
void multMatrixVector(const double m[9],double v[3], double T[3])
{
  //T = m*v (3x3 * 3x1)
  int i=0,j=0;
  for(i=0;i<3;i++)//For each line of m
    for(j=0;j<3;j++)
      T[i] += m[3*i+j]*v[j];
}
void multMatrixVector(double m[9],double v[3], double T[3])//1 overload to get rid of const array
{
  //T = m*v (3x3 * 3x1)
  int i=0,j=0;
  for(i=0;i<3;i++)//For each line of m
    for(j=0;j<3;j++)
      T[i] += m[3*i+j]*v[j];
}
void dispRot(double *rot)
{
  //Displays the rotation matrix in consol for specified part
  int i=0,j=0;
  for(i=0;i<7;i+=3)
  {
    for(j=0;j<3;j++)
      printf("%lf ", rot[i+j]);
    printf("\n");
  }
}
//////////////////////////end Utilitary functions//////////////////////////////

void BuildOriginTrans(double mat[9], Supervisor *robot)
{
  //This function only creates the ROTATION MATRIX for conversion from world to origin
  Node *root = robot->getFromDef("root");
  const double *root_rot = root->getOrientation();
  
  double x[3] = {1,0,0}, head[3] = {0};
  multMatrixVector(root_rot,x,head);
  double norm = sqrt(pow(head[0],2) + pow(head[2],2));
  head[0] /= norm; head[2] /= norm;
  double heading = atan2(-head[2],head[0]);//Angle in world around y-axis

  mat[0] = cos(-heading);  mat[1] = 0; mat[2] = sin(-heading);
  mat[3] = 0;              mat[4] = 1; mat[5] = 0;
  mat[6] = -sin(-heading); mat[7] = 0; mat[8] = cos(-heading);
}
void removeRootPosition(double v[3], Supervisor *robot)
{
  Node *root = robot->getFromDef("root");
  const double *root_pose = root->getPosition(); //getCenterOfMass();
  
  for(int i=0;i<3;i++)
    v[i] = v[i] - root_pose[i];
}
void OriginTrans(double pose[3], Supervisor *robot)
{
  //This function converts a VECTOR from world to origin (translation + rotation)
  double Ry[9] = {0};
  BuildOriginTrans(Ry,robot);
  removeRootPosition(pose,robot);//Apply a translation of -(root's position)
  double inter[3] = {0};
  multMatrixVector(Ry,pose,inter);//Apply a rotation of -(root's angle around y)
  for(int i=0;i<3;i++)
    pose[i] =inter[i];
  return;
}
void RotateTrans(double mat[9], Supervisor *robot)
{
  //This function converts a rotation MATRIX from world to origin
  double tran[9] = {0}, inter[9] = {0};
  BuildOriginTrans(tran,robot);
  mult(tran,mat,inter);
  for(int i=0;i<9;i++)
    mat[i] = inter[i];
}
void RotVectTrans(double v[3], Supervisor *robot)
{
  //This function only rotates a VECTOR from world to origin
  double tran[9] = {0}, inter[3] = {0};
  BuildOriginTrans(tran,robot);
  multMatrixVector(tran,v,inter);
  for(int i=0;i<3;i++)
    v[i] = inter[i];
}

void getRootHeight(double state[INPUT_STATE_SIZE], Supervisor *robot)
{
  //Gives root height relativ to ground
  Node *root = robot->getFromDef("root");
  const double *pose_vector = root->getCenterOfMass();
  state[0] = pose_vector[1];
  return;
}

void getPose(std::string part, int offset, double state[INPUT_STATE_SIZE], Supervisor *robot)
{
  //Gives center of mass position T of specified part relativ to root (x,y,z)
  Node *elem = robot->getFromDef(part);
  const double *part_pose = elem->getCenterOfMass();

  double T[3] = {0};
  int i = 0;
  for(i=0;i<3;i++)
     T[i] = part_pose[i];
  OriginTrans(T,robot);

  for(i=0;i<3;i++)
    state[offset+i] = T[i]; //Update state vector with local coordinates 
    
  return;
}

//Personal algorithm, seems to produce the same results than RotMatToQuaternion
//(Not used)
void extractQuaternionFromMatrix(double rot[9], double q[4])
{
  //Finds the main vector of rotation and angle
  double cosTheta = 0.5*(rot[0]+rot[4]+rot[8]-1);
  double sinTheta = 0.5*sqrt(pow(rot[7]-rot[5],2)+pow(rot[2]-rot[6],2)+pow(rot[3]-rot[1],2));
  double theta = atan2(sinTheta,cosTheta);
  double n[3] = {(rot[7]-rot[5])/2/sinTheta,(rot[2]-rot[6])/2/sinTheta,(rot[3]-rot[1])/2/sinTheta};
  int i=0;
  for(i=0;i<3;i++)//In case theta == 0, force vector to 0
    if(isnan(n[i]) || theta == 0)
      n[i] = 0;
  //Create the quaternion corresponding to the vector and angle
  q[0] = cos(theta/2);
  q[1] = sin(theta/2)*n[0];
  q[2] = sin(theta/2)*n[1];
  q[3] = sin(theta/2)*n[2];
}
//Conversion algorithm taken from DeepLoco and implemented in the controller:
void RotMatToQuaternion(double mat[9], double q[4])
{
  double tr = mat[0] + mat[4] + mat[8];

  if(tr > 0)
  {
    double S = sqrt(tr + 1.0)*2;// S=4*qw 
    q[0] = 0.25 * S;
    q[1] = (mat[7] - mat[5])/S;
    q[2] = (mat[2] - mat[6])/S;
    q[3] = (mat[3] - mat[1])/S;
  }
  else if( (mat[0]>mat[4]) && (mat[0]>mat[8]) )
  {
    double S = sqrt(1.0 + mat[0] - mat[4] - mat[8])*2;// S=4*qx 
    q[0] = (mat[7] - mat[5])/S;
    q[1] = 0.25 * S;
    q[2] = (mat[1] + mat[3])/S;
    q[3] = (mat[2] + mat[6])/S;
  }
  else if( mat[4] > mat[8] )
  {
    double S = sqrt(1.0 + mat[4] - mat[0] - mat[8])*2;// S=4*qy
    q[0] = (mat[2] - mat[6])/S;
    q[1] = (mat[1] + mat[3])/S;
    q[2] = 0.25 * S;
    q[3] = (mat[5] + mat[7])/S;
  }
  else
  {
    double S = sqrt(1.0 + mat[8] - mat[0] - mat[4])*2;// S=4*qy
    q[0] = (mat[3] - mat[1])/S;
    q[1] = (mat[2] + mat[6])/S;
    q[2] = (mat[5] + mat[7])/S;
    q[3] = 0.25 * S;
  }
}
void dispQuater(double q[4])
{
  printf("<%lf,(%lf,%lf,%lf)>\n",q[0],q[1],q[2],q[3]);
}
void getRotation(std::string part, int offset, double state[INPUT_STATE_SIZE], Supervisor *robot)
{
  /*Gives rotation of specified body part relativ to root as a quaternion*/
  double rot[9] = {0};
  double q[4] = {0};//Relativ rotation as quaternion
  
  Node *node = robot->getFromDef(part);
  const double *rot_node = node->getOrientation();//Returns an array[9] instead of a 2D matrix
  for(int i=0;i<9;i++)
    rot[i] = rot_node[i];

  RotateTrans(rot,robot);
  
  RotMatToQuaternion(rot,q);//Convert relRot into corresponding quaternion

  if(q[0] < 0)
  {
    q[0] *= -1;
    q[1] *= -1;
    q[2] *= -1;
    q[3] *= -1;
  }
  
  for(int i=0;i<4;i++)
    state[offset+i] = q[i];
  return;
}
void getPoseAndRotation(double state[INPUT_STATE_SIZE], Supervisor *robot)
{
  /*This function places correctly the position and rotation of each part in vector "state"*/
  int i=0;
  for(i=0;i<NUM_PARTS;i++)
  {
    //printf("Part: %s\n",parts[i]);
    getPose(parts[i],1+7*i,state,robot);
    getRotation(parts[i],1+3+7*i,state,robot);
    //1=root_height, 3=part's pose_vector, 7=3 pose + 4 rot /part
  }
  return;
}

//New version
void getVelocities(double state[INPUT_STATE_SIZE], Supervisor *robot)
{
  //This function gets the linear and angular velocities in world's 
  //reference frame of each part and directly gives them to the state vector. 
  Node *node = NULL;
  int i=0,j=0;
  const double *vel = NULL;
  double linear_vel[3] = {0}, angular_vel[3] = {0};

  for(i=0;i<NUM_PARTS;i++)
  {
    node = robot->getFromDef(parts[i]);
    vel = node->getVelocity();
    for(j=0;j<3;j++)
    {
      linear_vel[j]  = vel[j];
      angular_vel[j] = vel[3 + j];
    }
    RotVectTrans(linear_vel,robot);
    RotVectTrans(angular_vel,robot);
    for(j=0;j<3;j++)
    {
      state[VEL_OFFSET + 6*i + j] = linear_vel[j];
      state[VEL_OFFSET + 6*i + 3 + j] = angular_vel[j];
    }
  }
}

void resetPhase()
{
  phase = 0;
}

void updatePhase(double state[INPUT_STATE_SIZE], double timer, Supervisor *robot)
{
  //Timer indicates the elapsed time since last update instead of giving the timestep
  int phiX = 0, i = 0;
  phase += timer/(double)1000; // timer is in [ms] -> convert to seconds
  //printf("[updatePhase] Phase = %lf\n",phase);
  if(phase > WCD)// (1s cycles is default setting, should not be changed)
    phase = 0;//Restart a walk cycle
    
  state[PHASE_OFFSET] = phase;//offset 105 in state

  //printf("Phase: %lf\n", phase);
  
  //Reset all phase indicators
  for(i=1;i<=4;i++)
    state[PHASE_OFFSET+i] = 0;
  
  if(0<=phase && phase<0.25*WCD)
  {
    phiX = 1;
    stance = RIGHT_STANCE;
  }
  else if(0.25*WCD<=phase && phase<0.5*WCD)
    phiX = 2;
  else if(0.5*WCD<=phase && phase<0.75*WCD)
  {
    phiX = 3;
    stance = LEFT_STANCE;
  }
  else if(0.75*WCD<=phase && phase<1*WCD)
    phiX = 4;
  
  state[PHASE_OFFSET + phiX] = 1;

  //Check if Phi has changed
  if(old_phi != phiX)
  {
    printf("Phi: %d\n",phiX);
    old_phi = phiX;
  }
  //If foot stance has changed, new planning required
  if(old_stance != stance)
  {
    stance_changed = true;
    old_stance = stance;
    //printf("Stance changed: %d\n",stance);
  }
  else
    //stance_changed = false;

  //Check if swinging foot touches ground
  if(stance == RIGHT_STANCE)//While right stance
  {
    if(state[STEP_BOOL_OFFSET+1])//Left foot touches ground
    {
      foot_landed = true;
      //printf("Foot landed\n");
    }
  }
  else if(stance == LEFT_STANCE)//While left stance
  {
    if(state[STEP_BOOL_OFFSET])//Right foot touches ground
    {
      foot_landed = true; // Well, doesn't work too good
      //printf("Foot landed\n");
    }
  }
  
  //Wait for timing AND robot to land its swinging foot  
  if(stance_changed)// && foot_landed)
  {
    //printf("Planning footstep\n");
    planFootstep(stance,robot);
    stance_changed = false;
    foot_landed = false;
  }
  
}

void getFeetContacts(double state[INPUT_STATE_SIZE], Supervisor *robot)
{
  Node* rightFoot = robot->getFromDef(parts[4]);
  Node* leftFoot = robot->getFromDef(parts[7]);
  
  if(rightFoot->getNumberOfContactPoints())
    state[STEP_BOOL_OFFSET] = 1;
  else
    state[STEP_BOOL_OFFSET] = 0;
  if(leftFoot->getNumberOfContactPoints())
    state[STEP_BOOL_OFFSET+1] = 1;
  else
    state[STEP_BOOL_OFFSET+1] = 0;
}

//New version
void getWalkDeltas(double state[INPUT_STATE_SIZE], Supervisor *robot)
{
  //Reset all deltas to zero
  int i = 0;
  for(i=0;i<NUM_DELTAS;i++)
    state[DELTAS_OFFSET+i] = 0;
  
  double* p01 = getFeetTargets();
  double delta0[3] = {0};
  double delta1[3] = {0};

  std::string string1 = "";
  std::string string2 = "";
  
  //Update adequate deltas accordingly to stance
  enum {rightX0,rightY0,rightZ0,leftX0,leftY0,leftZ0,
        rightX1,rightY1,rightZ1,leftX1,leftY1,leftZ1};
  if(stance == RIGHT_STANCE)
  {
    Node *stance = robot->getFromDef("rightAnkle");
    Node *swing = robot->getFromDef("leftAnkle");
    const double* st = stance->getCenterOfMass();
    const double* sw = swing->getCenterOfMass();

    for(i=0;i<3;i++)
    {
      delta0[i] = p01[i] - sw[i];
      delta1[i] = p01[3+i] - st[i];
    }

    OriginTrans(delta0,robot);
    OriginTrans(delta1,robot);
    
    state[DELTAS_OFFSET + leftX0] = delta0[0];
    state[DELTAS_OFFSET + leftY0] = delta0[1];
    state[DELTAS_OFFSET + leftZ0] = delta0[2];
    state[DELTAS_OFFSET + rightX1] = delta1[0];
    state[DELTAS_OFFSET + rightY1] = delta1[1];
    state[DELTAS_OFFSET + rightZ1] = delta1[2];
  }
  else if(stance == LEFT_STANCE)
  {
    Node *stance = robot->getFromDef("leftAnkle");
    Node *swing = robot->getFromDef("rightAnkle");
    const double* st = stance->getPosition();
    const double* sw = swing->getPosition();

    for(i=0;i<3;i++)
    {
      delta0[i] = p01[i] - sw[i];
      delta1[i] = p01[3+i] - st[i];
    }

    OriginTrans(delta0,robot);
    OriginTrans(delta1,robot);
  
    state[DELTAS_OFFSET + rightX0] = delta0[0];
    state[DELTAS_OFFSET + rightY0] = delta0[1];
    state[DELTAS_OFFSET + rightZ0] = delta0[2];
    state[DELTAS_OFFSET + leftX1] = delta1[0];
    state[DELTAS_OFFSET + leftY1] = delta1[1];
    state[DELTAS_OFFSET + leftZ1] = delta1[2];
  }
}

void getTargetHeading(double state[INPUT_STATE_SIZE], Supervisor *robot)
{
  double *heading = getHeading();//Heading vector in world
  double head[3] = {heading[0],0,heading[1]};//Convert to 3D vector
  RotVectTrans(head,robot);//Apply rotation
  double angle = atan2(-head[2],head[0]);//Calculate angle
  state[TARGET_HEADING_OFFSET] = angle;
}

void buildStateVector(double state[INPUT_STATE_SIZE], Supervisor *robot, double timer)
{
  //Define what heading the robot should follow
  defineHeading();

  //Create state vector
  getRootHeight(state, robot);
  getPoseAndRotation(state,robot);
  getVelocities(state,robot);
  getFeetContacts(state,robot);
  updatePhase(state,timer,robot);
  getWalkDeltas(state,robot);
  getTargetHeading(state,robot);
}