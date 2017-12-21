#include <webots/Supervisor.hpp>

using namespace webots;

void defineHeading();

double* getHeading();

double* getFeetTargets();

void getRootPositionInPlane(double v[3], Supervisor *robot);

void setFirstSteps(Supervisor *robot);

void planFootstep(bool stance, Supervisor *robot);

void placeTargetIndicators(Supervisor *robot, double t0[3], double t1[3]);