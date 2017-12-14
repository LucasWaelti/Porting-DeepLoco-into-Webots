# Documentation about the project of importing DeepLoco into Webots

The DeepLoco project was implemented by searchers of the **University of British Columbia**:

- [Website](http://www.cs.ubc.ca/~van/papers/2017-TOG-deepLoco/)
- [Paper PDF](http://www.cs.ubc.ca/~van/papers/2017-TOG-deepLoco/2017-TOG-deepLoco.pdf)
- Contact of one of the searchers on this project with whom I could interract: [Jason Peng](https://xbpeng.github.io/), his email: JasonPeng142@hotmail.com
- All files were downloaded from this [GitHub Repository](https://github.com/xbpeng/DeepLoco)

Jump to a section:
- [Downloading the project from the GitHub repository](#Downloading)
- [Navigating through DeepLoco's Source Code](#DeepLocoSource)
- [Porting DeepLoco into Webots](#Porting)
	- [Webots World](#World)
	- [The Robot](#Robot)
	- [Robot's Controller](#RobotController)
		- [Working principle of the controller](#WorkingPrinciple)
		- [Robot's state](#RobotState)
		- [Neural Network's output action vector](#Action)
		- [Controller's source files](#Source)
		- [Possible improvements about the controller](#Improve)
	- [Robot's Optimizer](#Optimizer)
		- [Working principle of the optimizer](#WorkingPrincipleOpt)
		- [Optimizer's source files](#SourceOptimizer)
- [Implementation of **DeepLoco.dll**](#DeepDLL)
- [Implementation of **Optimizer_Webots.dll**](#OptiDLL)

<a name="Downloading"></a>
## Downloading the project from the GitHub repository

> You should not need to install anything as it is already done!

Download the repository on your machine **as well as** the `library` folder that can be found [here](https://github.com/xbpeng/DeepTerrainRL/releases). It can be found from the main repository under `Closed Issues: "Where is the external folder?"` too.

### Get the project to run

#### DeepLoco.exe

> This is the program that allows to visualize the robot walking. 

The project is a **Visual Studio Solution** whose `include` and `linker` dependencies are specified for the **Microsoft Visual Studio IDE**. I used the 2017 version of the IDE. But there are configurations that are neccessary to set properly in order to be able to compile the project: 

- First I had to recreate the architecture of the folders, which took quite some time. At that moment, I was not too familiar with the IDE and did not know what parameters were to be changed in order to solve that problem. That method was unfortunate as it led to a **8 Gb** folder containing the whole project with some redundancies... 

- Once the IDE could find all its include paths, following parameters had to be specified in order for the linker to be able to work properly: 
Go under `Project->DeepLoco Properties...`, and set following fields:
	- `Configuration Properties->General->Platform Toolset`: **Visual Studio 2013 (v120)** 
	(the compiler version is very important as different versions tend to be incompatible. This is mainly due to the fact that the project contains precompiled libraries.)
	- `Configuration Properties->Debugging->Command`: **$(TargetPath)** 
	- `Configuration Properties->Debugging->Command Arguments`: **-arg_file= args/test_args.txt**, the command line argument **DeepLoco.exe** needs to run. 
	- `Configuration Properties->Debugging->Working Directory`: **.**. 
	- `Configuration Properties->C/C++->General->Additional Include Directories`, after clicking `<Edit...>` (note that most of them were already set): 
		**C:\Users\Lucas\Documents\DeepLoco\DeepLoco-master**
		**..\library\Bullet3\src**
		**..\library**
		**..\library\caffe\include**
		**..\library\boost\boost_1_64_0**
		**..\library\caffe\3rdparty\include\hdf5**
		**..\library\caffe\3rdparty\include**
		**..\library\caffe\3rdparty\include\openblas**
		**..\library\caffe\3rdparty\include\lmdb**
		**%CUDA_PATH%\include**
		**..\library\OpenCV\include**
		**..\library\caffe\src**
		**%(AdditionalIncludeDirectories)**
	- `Configuration Properties->Linker->General->Additional Library Directories`, after clicking `<Edit...>`: 
		**..\library\lib**
		**..\library\Bullet3\build3\lib\Release**
		**..\library\boost\boost_1_64_0\stage\lib**
		**..\library\boost\boost_1_64_0\libs**
		**..\library\boost_lib**
		**..\library\caffe\3rdparty\lib**
		**..\library\OpenCV\x64\vc12\staticlib**
		**%CUDA_PATH%\lib\x64**
		**%(AdditionalLibraryDirectories)**

#### DeepLoco_Optimizer.exe

> This program is used to train the Policy of the robot for different scenarios. 

The project is a **Visual Studio Solution** using most of the same files that the **Solution** of **DeepLoco.exe**. The compilation did not generate much trouble and the presets in the IDE are analog to what was already described for **DeepLoco.exe**.

<a name="DeepLocoSource"></a>
## Navigating through DeepLoco's Source Code

DeepLoco is based on different libraries: 

- `caffe`: used for every task related to Neural Networks. The library was adapted to this project and the version published in the GitHub repository has to be used. 
- `Eigen`: a linear algebra library, used throughout the whole project. 
- `Bullet`: the physics engine used to simulate the robot. 

Everything was implemented from srcatch in this project, which yields a great number of files, that are often of no interest for porting DeepLoco into Webots (in particular all the rendering and the physics related files, which are all elements Webots takes care of). 

TODO: KEEP DOCUMENTING THIS

<a name="Porting"></a>
## Porting DeepLoco into Webots

### Introduction 

The source code of DeepLoco had to be readapted in order to interface it with Webots. The first approach was to recompile everything using `gcc`. This method revealed itself inconclusive as the `linker` had too much trouble accomplishing its task. It was decided that the source code of DeepLoco would have to be compiled into a DLL to implement Webots controllers, using the **Microsoft Visual Studio IDE**.

Two controllers were implemented: 

- a controller only used to make the robot walk, using the Neural Network of DeepLoco.
- a controller implementing the learning process in order to allow the robot to learn how to walk in the Webots environement. 

<a name="World"></a>
### Webots World

The World that was implemented is basic. It is only made of a plane, analog to what can be seen when launching **DeepLoco.exe** like this: `DeepLoco.exe -arg_file= args/test_args.txt`. 

Some modifications were made to the world's parameters: 
- `WorldInfo->CFM` = **1e-7**
- `WorldInfo->basicTimeStep` = **1**
- `WorldInfo->contactProperties->ContactProperties->CoulombFriction` = **0.9**
- `WorldInfo->contactProperties->ContactProperties->bounce` = **1e-5**
- `WorldInfo->contactProperties->ContactProperties->bounceVelocity` = **1e-5**
- `WorldInfo->contactProperties->ContactProperties->softCFM` = **1e-7**

These parameters have a huge impact on the simulation. Once set, they should not be changed. 

<a name="Robot"></a>
### The Robot

The robot was manually rebuilt as precisely as possible in Webots, based on the file containing its dimensions, joint positions and weights: `.\data\characters\biped3d_mocap.txt`. The robot has `Supervisor` capacities, which was necessary because of the type of information that is required to be fed to the Neural Network (more on that later). The field `Supervisor->controllerArgs` needs to be set accordingly to the controller used by the robot/Supervisor. 

The robot in DeepLoco has 8 parts: 
- root
- torso
- right_hip
- right_knee
- right_ankle
- left_hip
- left_knee
- left_ankle

Parts order: `0-root, 1-torso, 2-right_hip, 3-right_knee, 4-right_ankle, 5-left_hip, 6-left_knee, 7-left_ankle`

In DeepLoco, the robot is equipped with **hinge joints** at the knees and **universal joints** otherwise.

> Webots does not allow to have motorised universal joints, so they each had to be decomposed into a **hinge2joint** and a **hinge joint**. 

The robot in Webots therefore has 13 parts: 
- root
- torsoBottom
- torsoTop
- rightHipArt
- rightHip
- rightKnee
- rightKneeArt
- rightAnkle
- leftHipArt
- leftHip
- leftKnee
- leftKneeArt
- leftAnkle

For each universal joint, the **X-** and **Z-axis** are regrouped in the hinge2joint and the **Y-axis** is implemented as a simple hinge joint. The hinge2joints control intermediate small parts that had to be added to complete the articualtion. It was done in a way that the geometry and the weight of the robot remain unchanged. 

> The parts with "-Art" in their name, which stands for "-Articulation", are controlled by a hinge2joint and are intermediate parts. Except for the link between the knee and the ankle where the joints are inverted. 

The joints were named as follows: 
- jTorsoBottomX, jTorsoTopY, jTorsoBottomZ 
- jRightHipArtX, jRightHipY, jRightHipArtZ
- jRightKneeY
- jRightKneeArtX, jRightAnkleY, jRightKneeArtZ
- jLeftHipArtX, jLeftHipY, jLeftHipArtZ
- jLeftKneeY
- jLeftKneeArtX, jLeftAnkleY, jLeftKneeArtZ

> **Note**: Toggle the **Wireframe Frame Rendering** and **Show Joints Axis** to visualize the structure of the robot.

Robot control: 
Originally, a torque stable PD controller is implemented for each joint in DeepLoco, using following specifications: 
`Kp = [1000,300,300,100][Nm/rad]` (waist (root),hip,knee,ankle), 
`Kd = 0.1*Kp`, 
`Torque limits = [200,200,150,90][Nm]` (waist (root),hip,knee,ankle). 
The implementation is based on [this paper](https://www.cc.gatech.edu/people/home/turk/my_papers/stable_pd.pdf). 

<a name="RobotController"></a>
### Robot's Controller

General information:
- Path to the controller's directory: `.\Documents\DeepLoco_Webots\DeepLoco\controllers\DeepLoco_DLL_controller`
- Controller's main source file: `DeepLoco_controller.cpp`
- Controller's name: `DeepLoco_DLL_controller.exe`
- Required DLL: `DeepLoco.dll` (adapted source code from DeepLoco.exe) 

Webots arguments:
- `Supervisor->controllerArgs`: **DeepLoco.exe -arg_file= args/test_args.txt**

> **WARNING**: Do not forget to set this field accordingly when switching controllers

<a name="WorkingPrinciple"></a>
#### Working principle of the controller

The controller asks DeepLoco.dll to parse its arguments provided on the command line when starting the controller in Webots. The DLL initialises then its Neural Network using the library **caffe**. A few initialisations are made inside the controller and the main loop of the controller can then start running the simulation. The Neural Network needs to be queried at **30 Hz**. The `timer` variable takes care of respecting this frequency. 

Each time a control process is started: 
- the controller makes all the necessary observations about the state of the robot.
- it sends the observed state to the DLL in order to let the Network evaluate the state and generate an action. 
- the generated action is converted from an **Axis-Angle** representation to an **Euler** representation. 
- the action is finally applied to the motors. They receive a target position and the default PID controller of Webots will control the motor to move to the specified position.  

<a name="RobotState"></a>
#### Robot's state

The Network requires a **125D input state vector**. Different types of information are concatenated into this single vector, it is constructed as follows: 
- 0: Height of the root relativ to the ground
- 1-56: for each part (0-7), 56 values in total:
	- xyz-pose (relativ to root)
	- Rotation expressed as quaternion (<w,(x,y,z)>) (relativ to root)
- 57-104: for each part (0-7), 48 values in total:
	- xyz-translation speed in world
	- xyz-rotation speed along each axis
- 105: time in walk cycle [0,1]s
- 106: phi1 (bool)
- 107: phi2 (bool)
- 108: phi3 (bool)
- 109: phi4 (bool)
- 110: right foot contact with ground (bool)
- 111: left foot contact with ground (bool)
- 112-123: walk related data, 13 values in total:
	- right_delta0(x)
	- right_delta0(y)
	- right_delta0(z)
	- left_delta0(x)
	- left_delta0(y)
	- left_delta0(z)
	- right_delta1(x)
	- right_delta1(y)
	- right_delta1(z)
	- left_delta1(x)
	- left_delta1(y)
	- left_delta1(z)
- 124: error angle between desired orientation and root's heading. 

The foot deltas are the vectors going from the foot to ist target, indicating the Network if it is far from its objectiv or not. The swinging foot receives the target0 (light green in the simulation) while the standing foot receives the target1 (dark green in the simulation). When the stance changes, target1 becomes target0 and a new target1 is generated. 

<a name="Action"></a>
#### Neural Network's output action vector

After evaluating the input state vector depicted above, the Network produces a **22D action vector** indicating target positions expressed in the **Axis-Angle** convention for each joint. 
- 0 Torso angle
- 1 Torso x
- 2 Torso y
- 3 Torso z
- 4 right hip angle
- 5 right hip x
- 6 right hip y
- 7 right hip z
- 8 right knee angle
- 9  right ankle angle
- 10 right ankle x
- 11 right ankle y
- 12 right ankle z
- 13 left hip angle
- 14 left hip x
- 15 left hip y
- 16 left hip z
- 17 left knee angle
- 18  left ankle angle
- 19 left ankle x
- 20 left ankle y
- 21 left ankle z

These values are then converted by the controller into an **ZYX-Euler** representation before applying them to the motors. 

<a name="Source"></a>
#### Controller's source files

- The main file is `DeepLoco_controller.cpp`: its functionning was explained in the paragraph [Working principle of the controller](#WorkingPrinciple). 
- `wrapper.hpp`: This header contains the function of **DeepLoco.dll** the controller can call. 
```C++
void API cHelloWorld();
// you can call this function if you want to check that the DLL is correctly communicating with the program 

void API cParseArgs(int argc, char** argv);
// tells the DLL to parse its command line arguments

void API cInitCaffe();
// tells the DLL to initialise its network

void API cEvaluateNetwork(double in[INPUT_STATE_SIZE], double out[OUTPUT_STATE_SIZE]);
// get state action vector from network (provided in the out array)
```
- `state.cpp`: This file is responsible for building the state vector. Here are its most important functions: 
```C++
void getRootHeight(double state[INPUT_STATE_SIZE], Supervisor *robot);
// responsible for calculating the root's height and stores it in the state vector

void getPose(std::string part, int offset, double state[INPUT_STATE_SIZE], Supervisor *robot);
// gets the relativ position of the specified part relativ to the root and stores it in the state vector

void getRotation(std::string part, int offset, double state[INPUT_STATE_SIZE], Supervisor *robot);
// gets the relativ rotation of the specified part relativ to the root and stores it in the state vector

void getPoseAndRotation(double state[INPUT_STATE_SIZE], Supervisor *robot);
// this function orchestrates the 2 previous functions

void getVelocities(double state[INPUT_STATE_SIZE], Supervisor *robot);
// this function gets the linear and angular velocities in world's reference 
// frame of each part and directly gives them to the state vector. 

void updatePhase(double state[INPUT_STATE_SIZE], double timer, Supervisor *robot);
// this function defines where the robot is in the walk cycle (duration: 1s for making 2 steps)

void getFeetContacts(double state[INPUT_STATE_SIZE], Supervisor *robot);
// sets the booleans indicating if a foot is touching ground (1) or not (0)

void getWalkDeltas(double state[INPUT_STATE_SIZE], Supervisor *robot);
// this function indicates the distance between the robot's feet and its targets 

void getTargetHeading(double state[INPUT_STATE_SIZE], Supervisor *robot);
// gets the angle between the desired heading and the root's orientation (angle around Y axis)

void buildStateVector(double state[INPUT_STATE_SIZE], Supervisor *robot, double timer);
// this is the function implementing everything. Call it and your state vector gets built!
```
- `apply.cpp`: This files mainly receives the action from the Neural Network, converts it and applies it to the motor. 
```C++
void setStartSpeed(Supervisor* robot);
// gives an initial speed to some parts of the robot to help it start

void axisAngleToEuler(double command[OUTPUT_STATE_SIZE], double convCom[OUTPUT_CONV_STATE_SIZE]);
// conversion function between the 2 representations 

void applyMotorsTargets(double c[OUTPUT_CONV_STATE_SIZE]);
// applies the converted action to the motor and clamps out of range commands
```
- `control.cpp`: This file is used to define the desired heading of the robot and its foot targets. 
```C++
void defineHeading();
// defines a heading vector in the XZ plane to follow by the robot

void setFirstSteps(Supervisor *robot);
// defines first 2 foot targets before starting the simulation 

void planFootstep(bool stance, Supervisor *robot);
// each time a step needs to be planned, this function is called to set a new target

void placeTargetIndicators(Supervisor *robot, double t0[3], double t1[3]);
// places 2 markers on the ground (2 solid nodes named Target0 and Target1) 
// to indicate the placement of the foot targets
```

<a name="Improve"></a>
#### Possible improvements about the controller

- The PID controller of Webots does not match with the one in DeepLoco. Details about the controller implemented in DeepLoco can be found [here](https://www.cc.gatech.edu/people/home/turk/my_papers/stable_pd.pdf).

<a name="Optimizer"></a>
### Robot's Optimizer

<a name="WorkingPrincipleOpt"></a>
#### Working principle of the optimizer

In this case, the DLL has to do most of the job as it is orchestrating the whole learning process. Here, the Webots controller behaves like a slave while the DLL calls its functions to accomplish given tasks. 

The controller starts with the initialisation of different variables and declares a bunch of functions whose adress will be transmitted as callbacks to the DLL for later use during the learning process. While running, the DLL will store **tuples** containing information about the state of the robot. They are stored in the memory that will be then used to train the Network. There are actually 2 Networks: 
- a Network evaluating the **Policy**
- a Network evaluating the **Value Function**

The **Value Function** is actually used to train the **Policy** as it is first updated. More detailed explanations are available on [page 41:4 of the paper](http://www.cs.ubc.ca/%7Evan/papers/2017-TOG-deepLoco/2017-TOG-deepLoco.pdf). 

<a name="SourceOptimizer"></a>
#### Optimizer's source files

- `wrapper.hpp`: This wrapper exposes the functions of **Optimizer_Webots.dll**:
```C++
void API getFuncPointers(pfVoid f1, pfVoid f2, pfVoid f3, pfState f4,pfAction f5, pfBool f7, pfVoid f8);
// provides the callbacks from the main controller file to the DLL
// the arguments are function adresses declared in Optimizer.cpp

void API checkFunctionsPointers();
// makes a simple test by printing some text in the controller but called from the DLL. 

void API cSetupScenario();
// this is the function initialising the Networks as well as the scenario that specifies 
// what kind of task must be learned.

void API cRun();
// this function launches the learning process. Once launched, there is nothing left to do 
// but wait for it to be completed

void API cCleanUp();
// this is an unchanged cleanup function from the DeepLoco_Optimizer
```
- The main file is `Optimizer.cpp`: 
```C++
void interfaceTest();
// callback to test a call from the DLL

void stepSimulation();
// callback used by the DLL to step the simulation 

void newCycle();
// callback used by the DLL to reset the walk cycle timer

void getState(double state[INPUT_STATE_SIZE]);
// callback used by the DLL to create the state vector

void getActionAndApply(double action[OUTPUT_STATE_SIZE]);
// callback used by the DLL to let Webots retreive the action from the Network
// and apply it to the motors

bool detectFall();
// callback used by the DLL to check if the robot fell

void revertSimulation(); // soft-revert
// callback used by the DLL to revert the physics in Webots
// this is a SOFT-Revert!
```
- `softRevert.cpp`: this file allows to apply a soft revert to the robot in Webots.
```C++
void init_softRevert(Supervisor* sup);
// gets data necessary to reset properly the robot when soft-reverting

void softRevert(Supervisor* sup):
// actually implements the soft-revert of the robot
```

<a name ="DeepDLL"></a>
## Implementation of **DeepLoco.dll**

Only the file `main.cpp` of the DeepLoco Project had to be modified to implement the DLL and the files `main.hpp`, `wrapper.cpp` and `wrapper.hpp` had to be added. Following functions were implemented in `main.cpp` that are called by the wrapper when a request is made by the controller in Webots: 
```C++
// from file: main.cpp

void InitCaffe();
// Initialises the neural Network

void ParseArgs(int argc, char** argv);
// parses all arguments from the command line 

void prepNetwork();
// this function was written to explicitly select the Network that has to be loaded

void evaluateNetwork(double in[INPUT_STATE_SIZE], double out[OUTPUT_STATE_SIZE]);
// this functions evaluate the Network for a given input state provided by the controller
// and generates the action 
```

<a name ="OptiDLL"></a>
## Implementation of **Optimizer_Webots.dll**

The implementation for the learning is somewhat more complex because of the fact that the DLL controls the global process and that functions that are usually called from the controller in Webots need now to be called from the DLL itself. 

The file `main.cpp` had to be modified again. Following functions are used: 
```C++
void ParseArgs(int argc, char** argv);
// Nothing new here compared to DeepLoco.dll

void SetupScenario();
// this function was readapted from the one implemented in the Optimizer
// it only supports the scenario "imitate_step" which is the standard default walk 

void RunScene();
// once this functio is called, the learning process is engaged and will proceed until it's done

void CleanUp();
// clean up function for the scenario once the learning is over
```

`wrapper.cpp` has more features this time: 
```C++
// the wrapper declares the pointers to the functions from the controller
// that will be used for the learning. They are declared as global pointers
// to allow any file including "wrapper.cpp" to use these pointers and use 
// the functions from the controller

pfVoid pInterfaceTest = NULL;
pfVoid pStepSimulation = NULL;
pfVoid pNewCycle = NULL;
pfState pGetState = NULL;
pfAction pGetActionAndApply = NULL;
pfBool pDetectFall = NULL;
pfVoid pRevertSimulation = NULL;
```
The functions of `wrapper.cpp` were already discussed [here](#SourceOptimizer).

Another file was used for converting the state and action vectors from and to `Eigen`, `converter.cpp`:
```C++
void convert_action(Eigen::VectorXd& out_y, double action[OUTPUT_STATE_SIZE])
// Communication: DLL -> Webots
// converts action from an Eigen vector to a default C++ double array

void convert_state(Eigen::VectorXd& state, double state_webots[INPUT_STATE_SIZE])
// Communication: Webots -> DLL
// converts state to an Eigen vector from a default C++ double array
```

Insertions of Webots' commands within the DeepLoco_Optimizer were made at following locations: 

1. ***Network Evaluation\****: In function `void cNeuralNet::Eval(const Eigen::VectorXd& x, Eigen::VectorXd& out_y) const`, following lines were added: 
```C++
// INTERFACE WEBOTS
double action[OUTPUT_STATE_SIZE] = { 0 };
convert_action(out_y, action); // from converter.hpp
pGetActionAndApply(action); // from wrapper.hpp
pNewCycle(); // from wrapper.hpp
```
The function evaluates the state of the robot to produce an action. These few lines allow to retreive the action to transmit to Webots. The `const Eigen::VectorXd& x` argument should already contain the state of robot in Webots as it is built earlier by other functions. It is hard to verify though and **might be a source of error!** 

2. ***Tuples creation***: In function `void cScenarioExp::HandleNewActionUpdate()`, following lines were added:
```C++
//INTERFACE WEBOTS
double state[INPUT_STATE_SIZE] = { 0 };
pGetState(state);
Eigen::VectorXd state_prov = Eigen::VectorXd::Zero(INPUT_STATE_SIZE);
convert_state(state_prov,state);
mCurrTuple.mStateEnd.segment(0, INPUT_STATE_SIZE) = state_prov.segment(0, INPUT_STATE_SIZE);
//RecordState(mCurrTuple.mStateEnd); // Get state values from simulation instead
```
The function is queried each time a new tuple must be created. This occurs *each time the Neural Network is queried (30 Hz)*. The commented line was the original call to store the state in the tuple. 

3. ***Simulation Update***: In function `void cScenarioSimChar::Update(double time_elapsed)`: 
```C++
// Step in Webots
pStepSimulation(); // Replaces the update in DeepLoco
```
Most default update functions were disabled except for `PostSubstepUpdate(update_step);` that regulates the learning. Without this function, the process runs too fast in Webots. The function `PostSubstepUpdate(update_step);` calls `void cScenarioExp::HandleNewActionUpdate()` for saving the current tuple. 

4. ***Building State***: In function `void cTerrainRLCharController::BuildPoliState(Eigen::VectorXd& out_state) const`: 
```C++
//INTERFACE WEBOTS
double state[INPUT_STATE_SIZE] = { 0 };
pGetState(state);
Eigen::VectorXd state_prov = Eigen::VectorXd::Zero(INPUT_STATE_SIZE);
convert_state(state_prov, state);
out_state.segment(0, INPUT_STATE_SIZE) = state_prov.segment(0, INPUT_STATE_SIZE);
```
These few lines allow to store the current state of the robot in Webots so that other functions can then use it within the DLL. 

5. ***Detecting  Fall\****: In Function `bool cScenarioSimChar::HasFallen() const`:
```C++
// INTERFACE WEBOTS (this actually the whole function)
	bool fall = pDetectFall();
	//std::cout << "cScenarioSimChar::HasFallen fall = " << fall << std::endl;
	return fall;//mChar->HasFallen();
```
Furthermore, the function `cScenarioExpImitateStep::UpdateStepPlan()` checks for steps failure. This was deactivated, the instruction `mStepFail = true;` was desabled. It caused the simulation to revert immediatly. It is probable that this **has to be corrected** to ensure the good functionning of the learning process. 

