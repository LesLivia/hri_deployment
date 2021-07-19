Deploying Formally-Verified Assistive Robotics Applications
====================================

This repository contains the implementation of a deployment framework for assistive 
robots applications, specifically targeting the healthcare setting.
The deployment framework builds upon the work presented in the following articles:
- [*A Model-driven Approach for the Formal Analysis of Human-Robot Interaction Scenarios*][paper3]
- [*Formal Verification of Human-Robot Interaction in Healthcare Scenarios*][paper2]
- [*Statistical Model Checking of Human-Robot Interaction Scenarios*][paper1]

We contribute with a model-to-code translation principle to convert a specific subset 
of **Stochastic Hybrid Automata** into **deployable code**. This contribution allows practitioners
to deploy their applications or simulate them in [V-Rep][vrep] to run further testing before 
actual deployment.

The framework consists of the following elements:
- a custom [ROS package](catkin_ws/src/hri_scenarios) under the [`catkin_ws/`](catkin_ws/src/hri_scenarios) folder
- a custom [V-Rep scene](VRep_Scenes/hri_healthcare_scene.ttt) in the [`VRep_Scenes/`](VRep_Scenes/hri_healthcare_scene.ttt) folder
- [Python scripts](py_scripts) under the [`py_scripts/`](py_scripts) folder

**Note**: The framework has been only tested on Ubuntu 18.04. Should you succeed in running it
on a different OS, please report any issue or interesting result to *livia.lestingi@polimi.it*.

Authors:

| Name              | E-mail address           |
|:----------------- |:-------------------------|
| Lestingi Livia    | livia.lestingi@polimi.it |

ROS Package
-----------

The standalone Python robot controller and the robot and humans in the simulated scene 
communicate over ROS.
Installing [ROS Melodic][ros] is necessary to proceed.

The custom package has to be built within the catkin workspace (paths should be properly modified):

	cd $REPO_PATH/catkin_ws
	catkin_make

Source the generated setup file:

	source $REPO_PATH/catkin_ws/devel/setup.bash

V-Rep Scene
-----------

Before running V-Rep, make sure `roscore` is running and the [ROS Interface][rosint]
is correctly loaded.

The custom scene can be opened either via the GUI or the following command:

	./$VREP_PATH/vrep.sh $REPO_PATH/hri_deployment/VRep_Scenes/hri_healthcare_scene.ttt

**Note**: The human is controlled through the following keyboard inputs:

| Key            | Action                |
|:---------------|:----------------------|
| Up Arrow       | Walk                  |
| Down Arrow     | Stop                  |
| Left Arrow     | Turn Left             |
| Right Arrow    | Turn Right            |
| Tab            | Switch to Other Human |
| Enter          | Set as Served         |


Python Script
-----------

Make sure you have the required dependencies installed:

	pip install -r $REPO_PATH/py_scripts/requirements.txt

Finally, run the controller Python script:

	python3 $REPO_PATH/py_scripts/main.py
	
---

*Copyright &copy; 2021 Livia Lestingi*

[paper1]: https://dx.doi.org/10.4204/EPTCS.319.2
[paper2]: https://doi.org/10.1007/978-3-030-58768-0_17
[paper3]: https://dx.doi.org/10.1109/SMC42975.2020.9283204
[vrep]: https://coppeliarobotics.com/downloads
[ros]: http://wiki.ros.org/melodic/Installation
[rosint]: https://www.coppeliarobotics.com/helpFiles/en/rosInterf.htm



