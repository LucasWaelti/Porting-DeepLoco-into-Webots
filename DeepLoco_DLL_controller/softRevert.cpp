#include "softRevert.hpp"

// This file implements a soft revert of the simulation
// A standard revert of the simulation also reverts the controller
// which is not the wanted behavior

#define NUM_PARTS 13
#define NUM_HINGE1 7
#define NUM_HINGE2 5

// Declare all parts whose physics need to be reset
const char parts[NUM_PARTS][20] = {"root","torsoBottom","torsoTop","rightHipArt","rightHip",
	                               "rightKnee","rightKneeArt","rightAnkle","leftHipArt",
	                               "leftHip","leftKnee","leftKneeArt","leftAnkle"};
// t = translation, r = rotation (x,y,z,angle)
double part_tX[NUM_PARTS] = {0.0};
double part_tY[NUM_PARTS] = {0.0};
double part_tZ[NUM_PARTS] = {0.0};
double part_rX[NUM_PARTS] = {0.0};
double part_rY[NUM_PARTS] = {0.0};
double part_rZ[NUM_PARTS] = {0.0};
double part_rA[NUM_PARTS] = {0.0};
// s = speed
double part_stX[NUM_PARTS] = {0.0};
double part_stY[NUM_PARTS] = {0.0};
double part_stZ[NUM_PARTS] = {0.0};
double part_srX[NUM_PARTS] = {0.0};
double part_srY[NUM_PARTS] = {0.0};
double part_srZ[NUM_PARTS] = {0.0};

const char hinge1[NUM_HINGE1][20] = {"jTorsoTopY","jRightHipY","jRightKneeY",
									 "jRightAnkleY","jLeftHipY","jLeftKneeY",
									 "jLeftAnkleY"};
const char hinge2[NUM_HINGE2*2][20] = { "jTorsoBottomX","jTorsoBottomZ",
										"jRightHipArtX","jRightHipArtZ",
										"jRightKneeArtX","jRightKneeArtZ",
										"jLeftHipArtX","jLeftHipArtZ",
										"jLeftKneeArtX","jLeftKneeArtZ"};
// p = position, s = speed (all rotational)
double joint_pX[NUM_HINGE2] = {0.0};
double joint_pZ[NUM_HINGE2] = {0.0};

double joint_pY[NUM_HINGE1] = {0.0};

const double *tran = NULL;
const double *rot = NULL;

void init_softRevert(Supervisor* sup)
{
	//printf("Store solid\n");
	// Store solids
	for(int i=0;i<NUM_PARTS;i++)
	{
		// Get translation
		tran = sup->getFromDef(parts[i])->getField("translation")->getSFVec3f();
		part_tX[i] = tran[0];
		part_tY[i] = tran[1];
		part_tZ[i] = tran[2];

		// Get rotation
		rot = sup->getFromDef(parts[i])->getField("rotation")->getSFRotation();
		part_rX[i] = rot[0];
		part_rY[i] = rot[1];
		part_rZ[i] = rot[2];
		part_rA[i] = rot[3];
	}

	//printf("Store hinge1\n");
	// Store Hinge1Joints
	for(int i=0;i<NUM_HINGE1;i++)
	{
		joint_pY[i] = sup->getFromDef(hinge1[i])->getField("position")->getSFFloat();
		
		//std::cout << hinge1[i] << " " << joint_pX[i] << std::endl;
	}

	//printf("Store hinge2\n");
	// Store Hinge2Joints
	for(int i=0, j=0; i<NUM_HINGE2; ++i,j=2*i)
	{
		joint_pX[i] = sup->getFromDef(hinge2[j])->getField("position")->getSFFloat();
		joint_pZ[i] = sup->getFromDef(hinge2[j+1])->getField("position")->getSFFloat();

		//std::cout << joint_pX[i] << ", " << joint_pZ[i] << std::endl;
	}
}

void softRevert(Supervisor* sup)
{
	double tran[3] = {0};
	double rot[4] = {0};
	// Set solids
	for(int i=0;i<NUM_PARTS;i++)
	{
		// Set translation
		tran[0] = part_tX[i];
		tran[1] = part_tY[i];
		tran[2] = part_tZ[i];
		sup->getFromDef(parts[i])->getField("translation")->setSFVec3f(tran);
		

		// Set rotation
		rot[0] = part_rX[i];
		rot[1] = part_rY[i];
		rot[2] = part_rZ[i];
		rot[3] = part_rA[i];
		sup->getFromDef(parts[i])->getField("rotation")->setSFRotation(rot);
	}

	// Set Hinge1Joints
	for(int i=0;i<NUM_HINGE1;i++)
	{
		sup->getFromDef(hinge1[i])->getField("position")->setSFFloat(joint_pY[i]);
		
		//std::cout << hinge1[i] << " " << joint_pX[i] << std::endl;
	}

	// Set Hinge2Joints
	for(int i=0, j=0; i<NUM_HINGE2; ++i,j=2*i)
	{
		sup->getFromDef(hinge2[j])->getField("position")->setSFFloat(joint_pX[i]);
		sup->getFromDef(hinge2[j+1])->getField("position")->setSFFloat(joint_pZ[i]);

		//std::cout << joint_pX[i] << ", " << joint_pZ[i] << std::endl;
	}
	sup->simulationResetPhysics();
}



/*
//Reference functions
Node* selected_node = robot->getFromDef("node's name");
//from file apply.cpp
void setStartSpeed(Supervisor* robot)
{
  Node *root = robot->getFromDef("root");

  Node *tb = robot->getFromDef("torsoBottom");
  Node *tt = robot->getFromDef("torsoTop");

  Node *rh = robot->getFromDef("rightHipArt");
  Node *rh = robot->getFromDef("rightHip");

  Node *rk = robot->getFromDef("rightKnee");
  Node *rk = robot->getFromDef("rightKneeArt");

  Node *ra = robot->getFromDef("rightAnkle");
  Node *lh = robot->getFromDef("leftHip");
  Node *lk = robot->getFromDef("leftKnee");
  Node *la = robot->getFromDef("leftAnkle");
  
  const double vel_root[6] = {1.58487,-0.0521823,0.139421,
                             -1.33136,0.691321,-0.193296}; 
  root->setVelocity(vel_root);

  const double vel_tb[6] = {1.60774,-0.0522843,-0.0513435,
                           -0.572099,0.85682,-0.0614508};
  tt->setVelocity(vel_tb);
  
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
*/