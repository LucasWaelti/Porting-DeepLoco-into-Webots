# Documentation about the project of importing DeepLoco into Webots

The DeepLoco project was implemented by searchers of the **University of British Columbia**:

- [Website](http://www.cs.ubc.ca/~van/papers/2017-TOG-deepLoco/)
- [Paper PDF](http://www.cs.ubc.ca/~van/papers/2017-TOG-deepLoco/2017-TOG-deepLoco.pdf)
- Contact of one of the searchers on this project with whom I could interract: [Jason Peng](https://xbpeng.github.io/), his email: JasonPeng142@hotmail.com
- All files were downloaded from this [GitHub Repository](https://github.com/xbpeng/DeepLoco)

Jump to a section:
- [Downloading the project from the GitHub repository](#Downloading)
- [Porting DeepLoco into Webots](#Porting)
	- [Webots World](#World)
	. [The Robot](#Robot)

<a name="Downloading">
## Downloading the project from the GitHub repository
</a>

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

That's it for the main executable **DeepLoco.exe**. It is not excluded that some more tweaking might be necessary. 

#### DeepLoco_Optimizer.exe

> This program is used to train the Policy of the robot for different scenarios. 

The project is a **Visual Studio Solution** using most of the same files that the **Solution** of **DeepLoco.exe**. The compilation did not generate much trouble and the presets in the IDE are analog to what was already described for **DeepLoco.exe**.

<a name="Porting"></a>
## Porting DeepLoco into Webots

### Introduction 

The source code of DeepLoco had to be readapted in order to interface it with Webots. The first approach was to recompile everything using `gcc`. This method revealed itself inconclusive as the `linker` had too much trouble accomplishing its task. It was decided that the source code of DeepLoco would have to be compiled into a DLL to implement Webots controllers, using the **Microsoft Visual Studio IDE**.

Two controllers were implemented: 

- a controller only used to make the robot walk, using the Neural Network of DeepLoco.
- a controller implementing the learning process in order to allow the robot to learn how to walk in the Webots environement. 

<a name="World">
### Webots World
</a>

The World that was implemented is basic. It is only made of a plane, analog to what can be seen when launching **DeepLoco.exe** like this: `DeepLoco.exe -arg_file= args/test_args.txt`. 

Some modifications were made to the world's parameters: 
- `WorldInfo->CFM` = **1e-7**
- `WorldInfo->basicTimeStep` = **1**
- `WorldInfo->contactProperties->ContactProperties->CoulombFriction` = **0.9**
- `WorldInfo->contactProperties->ContactProperties->bounce` = **1e-5**
- `WorldInfo->contactProperties->ContactProperties->bounceVelocity` = **1e-5**
- `WorldInfo->contactProperties->ContactProperties->softCFM` = **1e-7**

These parameters have a huge impact on the simulation. Once set, they sould not be changed. 

<a name="Robot">
### The Robot
</a>

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

For each universal joint, the **X-** and **Z-axis** are regrouped in the hinge2joint and the **Y-axis** is implemented as a simple hinge joint. The hinge2joints control intermediate small parts that had to be added to complete the articualtion. It was done in the way that the geometry and the weight of the robot remain unchanged. 

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

<a name="RobotController">
### Robot's Controller
</a>

General information:
- Path to the controller's directory: `.\Documents\DeepLoco_Webots\DeepLoco\controllers\DeepLoco_DLL_controller`
- Controller's main source file: `DeepLoco_controller.cpp`
- Controller's name: `DeepLoco_DLL_controller.exe`
- Required DLL: `DeepLoco.dll` (adapted source code from DeepLoco.exe) 

Webots arguments:
- `Supervisor->controllerArgs`: **DeepLoco.exe -arg_file= args/test_args.txt**

> **WARNING**: Do not forget to set this field accordingly when switching controllers

<a name="WorkingPrinciple">
#### Working principle of the controller
</a>

The controller asks DeepLoco.dll to parse its arguments provided on the command line when starting the controller in Webots. The DLL initialises then its Neural Network using the library **caffe**. A few initialisations are made inside the controller and the main loop of the controller can then start running the simulation. The Neural Network needs to be queried at **30 Hz**. The `timer` variable takes care of respecting this frequency. 

Each time a control process is started: 
- the controller makes all the necessary observations about the state of the robot.
- it sends the observed state to the DLL in order to let the Network evaluate the state and generate an action. 
- the generated action is converted from an **Axis-Angle** representation to an **Euler** representation. 
- the action is finally applied to the motors. They receive a target position and the default PID controller of Webots will control the motor to move to the specified position.  

<a name="RobotState">
#### Robot's state
</a>

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

<a name="Action">
#### Neural Network's output action vector
</a>

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

<a name="Source">
#### Controller's source files
</a>

- The main file is `DeepLoco_controller.cpp`: its functionning was explained in the paragraph "_Working principle of the controller_". 
- `wrapper.hpp`: This header contains the function of the DLL the controller can call. 
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
void getRootHeight(double state[INPUT_STATE_SIZE], Supervisor *robot)
// responsible for calculating the root's height and stores it in the state vector

void getPose(std::string part, int offset, double state[INPUT_STATE_SIZE], Supervisor *robot)
// gets the relativ position of the specified part relativ to the root and stores it in the state vector

void getRotation(std::string part, int offset, double state[INPUT_STATE_SIZE], Supervisor *robot)
// gets the relativ rotation of the specified part relativ to the root and stores it in the state vector

void getPoseAndRotation(double state[INPUT_STATE_SIZE], Supervisor *robot)
// this function orchestrates the 2 previous functions

void getVelocities(double state[INPUT_STATE_SIZE], Supervisor *robot)
// this function gets the linear and angular velocities in world's reference 
// frame of each part and directly gives them to the state vector. 

void updatePhase(double state[INPUT_STATE_SIZE], double timer, Supervisor *robot)
// this function defines where the robot is in the walk cycle (duration: 1s for making 2 steps)

void getFeetContacts(double state[INPUT_STATE_SIZE], Supervisor *robot)
// sets the booleans indicating if a foot is touching ground (1) or not (0)

void getWalkDeltas(double state[INPUT_STATE_SIZE], Supervisor *robot)
// this function indicates the distance between the robot's feet and its targets 

void getTargetHeading(double state[INPUT_STATE_SIZE], Supervisor *robot)
// gets the angle between the desired heading and the root's orientation (angle around Y axis)

void buildStateVector(double state[INPUT_STATE_SIZE], Supervisor *robot, double timer)
// this is the function implementing everything. Call it and your state vector gets built!
```

#### Possible improvements about the controller

- Implement the same PD controller as [the one used in DeepLoco](https://www.cc.gatech.edu/people/home/turk/my_papers/stable_pd.pdf). 

#### Implementation of DeepLoco.dll

TODO

### Robot's Optimizer

TODO

## Navigating through DeepLoco's Source Code

TODO: list classes and functions and describe their role
