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
- [Python scripts](py_scripts) under the [`py_scripts/`](py_scripts) folder to control the robot in the scene and respond to human actions

Authors:

| Name              | E-mail address           |
|:----------------- |:-------------------------|
| Lestingi Livia    | livia.lestingi@polimi.it |


-----------



[paper1]: https://dx.doi.org/10.4204/EPTCS.319.2
[paper2]: https://doi.org/10.1007/978-3-030-58768-0_17
[paper3]: https://dx.doi.org/10.1109/SMC42975.2020.9283204
[vrep]: https://coppeliarobotics.com/downloads

