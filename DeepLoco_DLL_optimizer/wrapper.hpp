#pragma once

#define INPUT_STATE_SIZE 	125
#define OUTPUT_STATE_SIZE 	22

#ifdef DLL
#define API __declspec(dllexport)  
#else 
#define API __declspec(dllimport) 
#endif

//C functions used as wrapper to C++ functions
extern "C"
{
	// Declare a type of function pointer
	typedef void(*pfVoid)();
	typedef bool(*pfBool)();
	typedef void(*pfState)(double state[INPUT_STATE_SIZE]);  //pointer function State
	typedef void(*pfAction)(double action[OUTPUT_STATE_SIZE]);//pointer function Action
	typedef void(*pfReward)(double reward);

	// Declare global pointers to functions in Optimizer.cpp
	extern pfVoid pInterfaceTest;
	extern pfVoid pStepSimulation;
	extern pfVoid pNewCycle;
	extern pfState pGetState;
	extern pfAction pGetActionAndApply;
	extern pfBool pDetectFall;
	extern pfVoid pRevertSimulation;
	extern pfReward pSaveReward;

	void API getFuncPointers(pfVoid f1, pfVoid f2, pfVoid f3, pfState f4,
							 pfAction f5, pfBool f7, pfVoid f8, pfReward f9);
	void API checkFunctionsPointers();

	void API cHelloWorld();
	void API cParseArgs(int argc, char** argv);
	void API cInitCaffe();
	void API cSetupScenario();
	void API cRun();
	void API cEvaluateNetwork(double in[INPUT_STATE_SIZE], 
							  double out[OUTPUT_STATE_SIZE]);
	void API cCleanUp();
}