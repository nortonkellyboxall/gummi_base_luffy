#Introduction
Luffy is the ACRV version of the [Gummi Arm](https://github.com/GummiArmCE). This page serves to hopefully help you get set up and using the arm without too much hassle. 

The Gummi Arm is split into multiple packages, this is because the designers wanted to maintain the ability for people to design their own versions of the arm and make various modifications such as the gripper type for example.

The packages are as follows:
- gummi_base_luffy      [link to repo](https://github.com/nortonkellyboxall/gummi_base_luffy)
- gummi_ee_luffy        [link to repo](https://github.com/nortonkellyboxall/gummi_ee_luffy)
- gummi_head_twodof     [link to repo](https://github.com/nortonkellyboxall/gummi_head_twodof)
- gummi_interface       [link to repo](https://github.com/nortonkellyboxall/gummi_interface)
- gummi_moveit          [link to repo](https://github.com/nortonkellyboxall/gummi_moveit)
- gummi_hardware_Luffy  [link to repo](https://github.com/nortonkellyboxall/gummi_hardware_Luffy)

Each of these packages are connected and required to be cloned or forked.

#Hardware
The base package encapsulates the shoulder roll, shoulder yaw, shoulder pitch, upperarm roll, and elbow joints (as well as accompanying links). All of these joints except the upperarm roll are VSA joints, thus they contain two dynamixels and one dyamixel as an encoder. The encoder dynamixel does not have a gearbox and so only measures position, however since it is a dynamixel it allows it to be connected on the same bus as the driving motors. The links are all printed in nylon with the model able to be found in the gummi_hardware_luffy repo. 

#Package layout
The package is made up of multiple folders, this folder structure stays mostly consistent for each packages for hardware.

##Config
This folder contains all of the yaml files that define the joints. An example of this is the elbow joint

'''yaml
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

This package also contains the joints used by moveit for each move group in controllers.yaml as well as the start up conditions yaml file named gummi_base.yaml. This file designates some of the starting conditions for the arm when the startup routine is run.

##Dynamixel
This folder contains the individual motor configurations. Note each motor used in VSA joints are configured to multi-turn mode. This mode can be further understood at http://support.robotis.com/en/product/actuator/dynamixel/mx_series/mx-106.htm . This is also where the ID for each motor is referenced for the controller (The motor must be given the right ID prior to this). The joint name in this folder is what will be displayed when you echo the /gummi/joint_states topic. 

#Launch
This folder contains the necessary launch files to start up the arm in ROS. In the gummi_base_luffy package also contains the launch files that handle all the dynamixels from both the gummi_ee_luffy and gummi_head_twodof package. This is so that all of the motors can be launched at once. The details for how to use these launch files on start up are in the start up section.

The manager.launch file contains the details for each of the three serial ports used on the gummi arm. An example of how to configure one of these is as follows:

    gummi_d:
        <!-- Equivalent to /dev/ttyUSB1 can be find at /dev/serial/by-id -->
        port_name: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AL02L1KS-if00-port0 <!--/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5052NDS-if00-port0" --> 
        baud_rate: 1000000      #always set to 1000000 and make sure this is set when configuring the motor id number
        min_motor_id: 1         #sets the starting point for motor ids to be found on this bus
        max_motor_id: 25        #sets the finishing point for motor ids to be found on this bus
        update_rate: 100        #sets the feedback rate on the motors in Hz (100 is higher than the max currently and so it just sets to the max)

This is then linked to the controllers_base.launch file which loads the dynamixel controller and tells the system what bus the motor is on for example:
'''xml
    <rosparam file="$(eval find('gummi_base_' + base) + '/dynamixel/biceps.yaml')" command="load"/>
    <node name="biceps_controller_spawner" pkg="dynamixel_controllers" type="controller_spawner.py"
          args="--manager=dxl_manager
                --port gummi_ae
                biceps_controller"
          output="screen"/>

#Meshes
This folder contains all of the STL files that will be used in the URDF file. It contains both the normal STL file and the Convex hull of the STL. The convex hull is for the collision model so that less vertices need to be used for the collision checking

#Scripts
This folder contains scripts that run during the catkin_make. These files create the URDF from the XACRO files as well as the concatenation of the yaml files from the gummi_ee_luffy and gummi_head_twodof for the startup routine.

#Xacro
This folder contains the precurser to the URDF files. gummi_description_base.urdf.xacro is where the physical parameters of the arm are set, watch tutorials on how to create urdf files to understand this. There is also a useful wizard that moveit has to streamline the process a bit. The gummi_base.srdf.xacro sets the high level groups of joints, collision exceptions, and any named poses for those joint combinations. gummi_srdf.xacro concatenates the srdf files from gummi_ee_luffy and gummi_head_twodof to create the move groups that match controllers.yaml. This is where named poses for the whole arm can be set. 





