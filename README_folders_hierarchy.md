# Hierarchy of the folders within `DeepLoco`: 

The following list displays the content of the folders: 

- `.\DeepLoco`: The original paper, a few files regarding certain elements of the project as well as its documentation are located here. 
	- `.\DeepLoco\DeepLoco-master`: Contains the Solution for the DeepLoco executable. All files, folders and libraries required by the project are located here. 
		- `.\DeepLoco\DeepLoco-master\x64\Release`: Folder where DeepLoco.exe is generated. It has then to be pasted in `.\DeepLoco\DeepLoco-master` to be used. 

	- `.\DeepLoco\DeepLoco_Webots\DeepLoco`: Contains everything that Webots needs. 
		- `.\DeepLoco\DeepLoco_Webots\DeepLoco\controllers`: 
			- `.\DeepLoco\DeepLoco_Webots\DeepLoco\controllers\DeepLoco_DLL_controller`: Contains all source files required to build the controller as well as the folders providing arguments to it. This is where `DeepLoco.dll` has to be placed. 
			- `.\DeepLoco\DeepLoco_Webots\DeepLoco\controllers\DeepLoco_DLL_optimizer`: Contains all source files required to build the controller as well as the folders providing arguments to it. This is where `Optimizer_Webots.dll` has to be placed. 
			- `.\DeepLoco\DeepLoco_Webots\DeepLoco\controllers\kinematic_debug_controller`: Contains the source file that implement a simple controller used to simply demonstrate the robot's degrees of freedom. 
		- `.\DeepLoco\DeepLoco_Webots\DeepLoco\world`: Contains the world with the robot. 

	- `.\DeepLoco\ControllerFromScratch`: contains the solution of the project used by the Visual Studio IDE that is used to create the dll used by the controller in Webots. 
		- `.\DeepLoco\ControllerFromScratch\ControllerFromScratch`: Contains a few folders related to DeepLoco. The two source files that had to be added/modified (`Main.cpp`, `wrapper.cpp`) with their respective header and a few libraries for the project. The Viusal Studio Project the Solution is using is located here as well. 
		- `.\DeepLoco\ControllerFromScratch\x64\Release`: Contains mainly `DeepLoco.dll` that the controller in Webots uses to control the robot. It has to be added with the four other DLLs present in thsi folder. 

	- `.\DeepLoco\OptimizerFromScratch`:  contains the solution of the project used by the Visual Studio IDE that is used to create the dll used by the optimizer in Webots.
		- `.\DeepLoco\OptimizerFromScratch\ControllerFromScratch`: Contains a few folders related to DeepLoco. The three source files that had to be added/modified (`Main.cpp`, `wrapper.cpp` and `converter.cpp`) with their respective header and a few libraries for the project. The Viusal Studio Project the Solution is using is located here as well. 
		- `.\DeepLoco\OptimizerFromScratch\x64\Release`: Contains mainly `Optimizer_Webots.dll` that the optimizer in Webots uses to control and train the robot. It has to be added with the four other DLLs present in thsi folder. 

	- `.\DeepLoco\library`: contains a few directories that need to be placed here as required by the Visual Studio Solution. 