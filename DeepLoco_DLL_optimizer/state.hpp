#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>
#include <math.h>
#include "control.hpp"

#define GROUND_HEIGHT 			0
#define INPUT_STATE_SIZE 		125
#define OUTPUT_STATE_SIZE 		22
#define OUTPUT_CONV_STATE_SIZE	17
#define NUM_PARTS 				8
#define RIGHT_STANCE			0
#define LEFT_STANCE				1
#define WCD						1  //(Walk Cycle Duration 1 [s])
#define DELTA_0					0.05
#define DELTA_1					0.1
#define DELTA_IMPACT			1 //Impact of step error on walk (currently disabled)

#define VEL_OFFSET 				57
#define PHASE_OFFSET			105
#define STEP_BOOL_OFFSET 		110
#define DELTAS_OFFSET			112
#define NUM_DELTAS				12
#define TARGET_HEADING_OFFSET 	124

#define EPSILON 0.000001

//All the webots classes are defined in the "webots" namespace
using namespace webots;

void displayInput(double state[INPUT_STATE_SIZE]);

void transpose(const double* root, double tRoot[9]); 
void transpose(double root[9], double tRoot[9]);//1 overload rid of const array

void mult(const double* rotPart, double tRoot[9], double relRot[9]); 
void mult(const double* rotPart, const double tRoot[9], double relRot[9]);
void mult(double rotPart[9], double tRoot[9], double relRot[9]);//1 overload rid of const array

void multMatrixVector(const double m[9],double v[3], double T[3]);
void multMatrixVector(double m[9],double v[3], double T[3]); 

void removeRootPosition(double v[3], Supervisor *robot);

void OriginTrans(double pose[3], Supervisor *robot);
void BuildOriginTrans(double mat[9], Supervisor *robot);

void getRootHeight(double state[INPUT_STATE_SIZE], Supervisor *robot);

void getRotationMatrix(std::string part, Supervisor *robot, double relRot[9]);

void getPose(std::string part, int offset, double state[INPUT_STATE_SIZE], Supervisor *robot);

void dispRot(double *rot);

void extractQuaternionFromMatrix(double rot[9], double q[4]); 
void RotMatToQuaternion(double mat[9], double q[4]);

void dispQuater(double q[4]);

void getRotation(std::string part, int offset, double state[INPUT_STATE_SIZE], Supervisor *robot);

void getPoseAndRotation(double state[INPUT_STATE_SIZE], Supervisor *robot);

void getVelocities(double state[INPUT_STATE_SIZE], Supervisor *robot);

void resetPhase();

void updatePhase(double state[INPUT_STATE_SIZE], double timer, Supervisor *robot);

void getFeetContacts(double state[INPUT_STATE_SIZE], Supervisor *robot);

void getWalkDeltas(double state[INPUT_STATE_SIZE], Supervisor *robot);

void getTargetHeading(double state[INPUT_STATE_SIZE], Supervisor *robot);

void buildStateVector(double state[INPUT_STATE_SIZE], Supervisor *robot, double timer);//Overload

