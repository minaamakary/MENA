Welcome to my package readme file!

*********************************************************************************************************

Just to make this easy. 

Please download the below packages from ROBOTIS for turtlebot3 from the (github below link) and keep them handy in your workspace

https://github.com/ROBOTIS-GIT/turtlebot3

Also recommended

https://github.com/ROBOTIS-GIT/turtlebot3_simulations

https://github.com/ROBOTIS-GIT/turtlebot3_msgs

-->> but make sure to delete the following files below as we have them here and made changes to them in this package to avoid duplications and confusions

**turtlebot3_waffle.gazebo.xacro**
**turtlebot3_waflle.urdf.xacro**
**common_properties.xacro** (this file was not changed but was moved from the original robotis package - again please delete it to avoid duplications)


************************************************************************************************************


To run the project successfully all you need to do is 

Run the launch file turtlebot3_empty_world.launch! (below command)

roslaunch thesis turtlebot3_empty_world.launch

You will find it inside thesis --> launch 

The launch file has all the scripts so will run the whole system all together which contains the four main scripts
Distance Measurement - distance_measurement_node.py
Robot current position and angle - pose_orientation_node.py
Navigation - turtle_move_laserscan.py
Mapping - mappingRoom.py

If you need to display only navigation you can comment out the rest of the scripts and then rosrun separate scripts on individual terminals. 

**But for mapping you need all of them to be running.**


*****************************************************************************************************************


There are different worlds for testing purposes inside the worlds folder. If you need to use a different one than the one configured, modify it in the launch file turtlebot3_empty_world.launch



Thats all, thanks for reading!

