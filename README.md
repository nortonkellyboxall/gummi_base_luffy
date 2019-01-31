# gummi_base_template

The Gummi Arm is split into multiple packages, this is because the designers wanted to maintain the ability for people to design their own versions of the arm and make various modifications such as the gripper type for example. Luffy, which is the ACRV's version of the Gummi Arm does have some changes and so it has its own versions of these packages. 

The packages are as follows:
- gummi_base_luffy
- gummi_ee_luffy
- gummi_head_twodof
- gummi_interface
- gummi_moveit

The first three packages contain all of the URDF files and dynamixel configurations for each of their respective parts. 

## Setting-up

First set up a workspace

    $ mkdir -p <ROBOT-NAME>_ws/src
    $ cd <ROBOT-NAME>/src

Fork or clone this repository:

`$ git clone https://github.com/nortonkellyboxall/gummi_base_luffy.git` 

Do the same for the forearm:

`$ git clone https://github.com/GummiArmCE/gummi_ee_handshake.git`

To actually move the robot you will need either ros-\<YOUR-ROS-DISTRO\>-dynamixel-motor package installed or to you can clone the git from https://github.com/arebgun/dynamixel_motor.git . You will also need the gummi_interface package and if you want to use move-it, the gummi_moveit package: 

    $ git clone https://github.com/arebgun/dynamixel_motor.git
    $ git clone https://github.com/GummiArmCE/gummi_interface.git
    $ git clone https://github.com/GummiArmCE/gummi_moveit.git

Install all dependencies with rosdep:

`$ rosdep install -y --from-paths . --ignore-src --rosdistro <YOUR-ROS-DISTRO>`

Run catkin_make:

`$ catkin_make`

And source your workspace:

`$ source devel/setup.bash`

If you are working with multiple bases and you don't want to create new packages for each, you may also want to add the following lines to your ~/.bashrc file:

    parse_git_branch() {
    git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/ (\1)/'
    }
    export PS1="${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$(parse_git_branch)\[\033[00m\] $ "

This is a nice tweak of your shell to show the current branch in which you are working.

## Setting up a new base

Rename the directory "gummi_base_template" to "gummi_base_YOUR_OWN_BASE"

`$ mv gummi_base_template gummi_base_YOUR_OWN_BASE`

Edit the file "gummi_base_YOUR_OWN_BASE/set_env_gummi.sh"

`$ gedit gummi_base_YOUR_OWN_BASE/set_env_gummi.sh`

And change the line containing "export ROS_GUMMI_BASE=template" to "export ROS_GUMMI_BASE=YOUR_OWN_BASE" (required for launch files and by yaml_gen)

Edit the file package.xml

`$ gedit package.xml`

And change `<name>gummi_base_template</name>` to  `<name>gummi_base_YOUR_OWN_BASE</name>` (required finding the meshes - and textures if any- in the urdf file). 

You can now proceed to change "gummi_base_YOUR_OWN_BASE/launch/controllers_base.launch" to contain the motor drivers that your base has and where they are connected in the bus and the updating the yaml definitions of joints in "gummi_base_YOUR_OWN_BASE/dynamixel"

## Changing Urdf definitions:

Currently the urdf file is being generated during the catkin_make routine from the files in the xacro directory "gummi_description.urdf.xacro", "gummi_description_base.urdf.xacro" and an external file from the end effector "gummi_description_ee.urdf.xacro".

The simple setup we planned it to have all joint and link definitions for the base in the "gummi_description_base.urdf.xacro" (as well as "gummi_description_ee.urdf.xacro" containing the joints and links for the end effector only) and the file "gummi_description.urdf.xacro" would just call both and include them.

To change this to a new setup, simple either write by hand, or generate it from a solidworks model the URDF file as usual and place the joints and links from gripper up to the elbow joint (inclusive) in the end-effector xacro file and the remainder joints and links, that is, from base to elbow (exclusive) in the base urdf xacro file

TO DO:
 - finish separating yaml files
 - integrate alterations into gummi_interface

## New in this version (v1.0.0):

- First stable and tested version with the dovetail design. Tested with planning via moveit integration. 
- Decided to use branches instead of new repositories to keep all the different base definitions.

##

<a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png" /></a>

<br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International License</a>.
