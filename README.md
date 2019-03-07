# GUI BASED TRAJECTORY PLANNER FOR LBR iiwa MODELS (BOTH iiwa 7 and iiwa 14)

* Robot toolpath generation over surface of the mold 
* Robot IK is calculated with optimization based approach
* Any tool can be attached to the robot and IK can be computed wrt TCP
* Some sample Molds and Tools are added. You can add your custom tools and molds (To add your files, just copy files into CAD_stl folder)

**Requirements to run the code**
* MinGW compiler for MEX if using windows [Link](https://www.mathworks.com/matlabcentral/fileexchange/52848-matlab-support-for-mingw-w64-c-c-compiler)
* Eigen c++ library [Link](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* If using MEX based optimization, then nlopt library is required [Link](https://nlopt.readthedocs.io/en/latest/)

**To run the Code**

run MAIN_TRAJECTORY_PLANNER_VISUALIZER.m

**NOTE**
* Current IK in this method does not take collision avoidance under consideration
* If any new tool is added, then TCP should be specified in the code
