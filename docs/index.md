<a href="#hardware">Click here to learn about the Hardware</a>
------

<a href="#package layout">Click here to learn about the file layout in the package</a>
------

<a href="#startup">Click here to learn about the how to start up the Gummi Arm</a>
------

Introduction
======

Luffy is the ACRV version of the [Gummi Arm](https://github.com/GummiArmCE). This page serves to hopefully help you get set up and using the arm without too much hassle. 

The Gummi Arm is split into multiple packages, this is because the designers wanted to maintain the ability for people to design their own versions of the arm and make various modifications such as the gripper type for example.

<img src="images/Gummi2.png" alt="Luffy"/>

The packages are as follows:
- [gummi_base_luffy](https://nortonkellyboxall.github.io/gummi_base_luffy/)
- [gummi_ee_luffy](https://nortonkellyboxall.github.io/gummi_ee_luffy/)
- [gummi_head_twodof](https://nortonkellyboxall.github.io/gummi_head_twodof/)
- [gummi_interface](https://nortonkellyboxall.github.io/gummi_interface/)
- [gummi_moveit](https://github.com/nortonkellyboxall/gummi_moveit)
- [gummi_hardware_luffy](https://nortonkellyboxall.github.io/gummi_hardware_Luffy/)

Each of these packages are connected and required to be cloned or forked.

<a id="hardware"> Hardware </a>
======
The base package encapsulates the shoulder roll, shoulder yaw, shoulder pitch, upperarm roll, and elbow joints (as well as accompanying links). All of these joints except the upperarm roll are VSA joints, thus they contain two dynamixels and one dyamixel as an encoder. The encoder dynamixel does not have a gearbox and so only measures position, however since it is a dynamixel it allows it to be connected on the same bus as the driving motors. The links are all printed in nylon with the model able to be found in the gummi_hardware_luffy repo. 

<a id = "package layout"> Package layout </a>
======
The package is made up of multiple folders, this folder structure stays mostly consistent for each packages for hardware.

## Config
This folder contains all of the yaml files that define the joints. An example of this is the elbow joint
``` yaml
calibrated: 1       
equilibrium:
    sign: 1                 # Defines which way is positive for the joint 1 = anti-clockwise
    signFlexor: -1          # Defines the rotation direction of the flexing motor  
    signExtensor: -1        # Defines the rotation direction of the flexing motor
    nameFlexor: "biceps"
    nameExtensor: "triceps"
    servoRange: 6.2832      
    servoOffset: 0.0        # Offsetting can increase or decrease the tension in the joint
    loadLimit: 0.5  # Testing limitLoad method in equilibrium_model.py.
signEncoder: 1
signJoint: 1
name: "elbow"
nameEncoder: "elbow_encoder"
minAngle: -1.7
maxAngle: 0.47
maxAbsForwardError: 0.7
gains:
    P: 0.04
    I: 0.00002
    D: 0.2
restingPoseAngle: 0.0
```

This package also contains the joints used by moveit for each move group in controllers.yaml as well as the start up conditions yaml file named gummi_base.yaml. This file designates some of the starting conditions for the arm when the startup routine is run.

## Dynamixel
This folder contains the individual motor configurations. Note each motor used in VSA joints are configured to multi-turn mode. This mode can be further understood at http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-106.htm . This is also where the ID for each motor is referenced for the controller (The motor must be given the right ID prior to this). The joint name in this folder is what will be displayed when you echo the /gummi/joint_states topic. 

## Launch
This folder contains the necessary launch files to start up the arm in ROS. In the gummi_base_luffy package also contains the launch files that handle all the dynamixels from both the gummi_ee_luffy and gummi_head_twodof package. This is so that all of the motors can be launched at once. The details for how to use these launch files on start up are in the start up section.

The manager.launch file contains the details for each of the three serial ports used on the gummi arm. An example of how to configure one of these is as follows:
``` xml
gummi_d:
    <!-- Equivalent to /dev/ttyUSB1 can be find at /dev/serial/by-id -->
    port_name: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AL02L1KS-if00-port0 
    baud_rate: 1000000      #always set to 1000000 and make sure this is set when configuring the motor id number
    min_motor_id: 1         #sets the starting point for motor ids to be found on this bus
    max_motor_id: 25        #sets the finishing point for motor ids to be found on this bus
    update_rate: 100        #sets the feedback rate on the motors in Hz (100 is higher than the max currently and so it just sets to the max)
```

This is then linked to the controllers_base.launch file which loads the dynamixel controller and tells the system what bus the motor is on for example:
```xml
    <rosparam file="$(eval find('gummi_base_' + base) + '/dynamixel/biceps.yaml')" command="load"/>
    <node name="biceps_controller_spawner" pkg="dynamixel_controllers" type="controller_spawner.py"
          args="--manager=dxl_manager
                --port gummi_ae
                biceps_controller"
          output="screen"/>
```

## Meshes
This folder contains all of the STL files that will be used in the URDF file. It contains both the normal STL file and the Convex hull of the STL. The convex hull is for the collision model so that less vertices need to be used for the collision checking

## Scripts
This folder contains scripts that run during the catkin_make. These files create the URDF from the XACRO files as well as the concatenation of the yaml files from the gummi_ee_luffy and gummi_head_twodof for the startup routine.

## Xacro
This folder contains the precurser to the URDF files. gummi_description_base.urdf.xacro is where the physical parameters of the arm are set, watch tutorials on how to create urdf files to understand this. There is also a useful wizard that moveit has to streamline the process a bit. The gummi_base.srdf.xacro sets the high level groups of joints, collision exceptions, and any named poses for those joint combinations. gummi_srdf.xacro concatenates the srdf files from gummi_ee_luffy and gummi_head_twodof to create the move groups that match controllers.yaml. This is where named poses for the whole arm can be set. 

<a id = "startup"> Start Up </a>
======

To get the Gummi Arm started first open a terminal and navigate to the workspace that contains all of the Gummi Arm packages. To set up this workspace click [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and ensure that all the required [dependencies](https://github.com/GummiArmCE/docs/wiki/Installing) have been installed. This is also assuming that the Gummi Arm has all its joints configured properly and the construction completed (details can be found here).

Once in the workspace, source it using devel/setup.bash and then run roslaunch gummi_base_luffy all.launch. If everything has been built and installed properly this will run the arm through a startup routine and also open up moveit so that you can control the arm using Moveit's gui. Once this has been completed, you are free to run your code in another terminal (suggested that you use something like terminator, which splits terminals nicely).

## Start up code breakdown
This section will break down the all.launch file so that you know what the heck is going on.

``` xml
<launch>
  <arg name="base" value="$(env ROS_GUMMI_BASE)" />
  <arg name="head" value="$(env ROS_GUMMI_HEAD)" />
```
This is firstly declaring that when base or head is used that the environment variable assosciated to this gummi arm is used. If you dont change the environment variable then base will be luffy and head will be twodof. This is here since people might have different versions of the arm or different head arrangements.

``` xml
<include file="$(eval find('gummi_base_' + base) + '/launch/manager.launch')">
</include>
<include file="$(eval find('gummi_base_' + base) + '/launch/controllers.launch')">
</include>
 ```
 This runs the manager.launch and controllers.launch files which are inside gummi_base_luffy. The manager.launch file finds all of the dynamixels on each of the buses. If you are having an issue here then make sure all of the motors are connected properly, also check that the baud rate for each motor is correct and that you are telling the launcher the correct bus to find the motor on. 

 The controllers.launch file launches the controller files for gummi_base_luffy, gummi_ee_luffy, and gummi_head_twodof. 

 ``` xml
<include file="$(eval find('gummi_interface') + '/launch/gummi.launch')">
</include>
```
This launches the gummi arm and runs it through it's start up routine. Ensure that before the launch file is run that the motors are turned so that there is a bit of tension in each of the joints (power must be off), this will ensure it always starts up fine and no tendons get caught. The functions available in the gummi interface package will be discussed further [here](https://nortonkellyboxall.github.io/gummi_interface/).

``` xml
<include file="$(eval find('gummi_head_' + head) + '/launch/gummi_head.launch')">
</include>
```
This launches the camera on the gummi head as well as the second camera which is mounted remotely. It is a slightly modified version of the launch file in the realsense2_ros package. The main thing that has changed is that a depth topic that converts the depth into meters has been added.

```xml
<include file="$(eval find('gummi_moveit') + '/launch/gummi_moveit.launch')">
</include>
```

This launches moveit for the gummi arm. 

### Debugging
If this launch file isnt working, it is a good idea to launch each of them separately starting at the top and going down. This will help you determine where the source of the error is. Some common errors that appear are
- Power is not on so motors cant be found
- Motor has cable disconnected 
- Camera is not plugged in
- Motors are on the incorrect bus in controllers.launch
- Can't find the usb (make sure the right serial id is entered)

Also some helpful tools that I have found are useful for things like understanding namespaces etc
- rqt graph
- rqt plot
- rostopic echo /gummi/joint_states
- RVIZ
- rqt easy message publisher

### Writing your own stuff
Now you are ready to write your own stuff. To be able to command the gummi arm be sure to include this in your code

``` python
from gummi_interface.gummi import Gummi
from gummi_interface.msg import CoContraction

```
The first one imports the gummi arm class which contains all of the functions such as move_to and set co-contraction. The second one is a custom message type that co-contration uses.

It must be noted that at the moment, the dynamixels are quite slow on their read write. Each bus has 60Hz total to share across all the motors, so be conscious of this when designing your algorithm. 

To get a better understanding of how the Gummi class works then click [here](https://nortonkellyboxall.github.io/gummi_interface/).
