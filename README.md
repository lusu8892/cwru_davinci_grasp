# cwru_davinci_grasp
This package is the implementation of needle grasping for the Patient-side manipulators (PSMs) of the da Vinci<sup>&reg;</sup> surgical robot. The needle grasp manipulation path is computed by class [`pick_place::PickPlan`](https://docs.ros.org/kinetic/api/moveit_ros_manipulation/html/classpick__place_1_1PickPlan.html) in MoveIt! library.

The current implementation allows user to either control the robot to grasp a needle with a set of defined grasping parameters or help user choose a valid grasp pose within a list of candidates grasp.

This package includes:

  - Simple needle grasper (library and main function) implemented by class [`pick_place::PickPlan`](https://docs.ros.org/kinetic/api/moveit_ros_manipulation/html/classpick__place_1_1PickPlan.html) in MoveIt! library.
  - Simple pose-based grasp generator implemented by optimal needle grasping algorithms.
  - A needle grasping data processing program.
  - A .yaml needle grasp config file, inside of which the grasping parameters are defined
  - Test code
 
Developed by [Su Lu](https://github.com/lusu8892/) at the MeRCIS Lab, Case Western Reserve University.
 
## Install

### Ubuntu
Kinetic:
```
git clone https://github.com/cwru-davinci/cwru_davinci_grasp
```

### Dependent packages
```
git clone https://github.com/JenniferBuehler/convenience-pkgs.git
git clone https://github.com/cwru-davinci/cwru_davinci_moveit.git
```

## How to use
Before use, make sure the pose information of the needle is being published to the topic "/updated_needle_pose" in the form of "geometry_msgs/PoseStamped" ROS message.

### To use main function
To use davinci_simple_needle_grasper_main function, user needs to specify which PSMs("psm_one" or "psm_two") is going to be used to grasp needle, the needle's name and to pick or place. The default setting is to use "psm_one" to pick the "needle_r". Then the main function will ask user to use which grasp mode to grasp the needle.

To pick up the needle:
```
roslaunch cwru_davinci_grasp davinci_simple_needle_grasp.launch which_arm:=psm_one needle_name:=needle_r movement:=pick
```

To place the same needle (Implementation In Progress):
```
roslaunch cwru_davinci_grasp davinci_simple_needle_grasp.launch which_arm:=psm_one needle_name:=needle_r movement:=place
```

### To use code in user's program
Within your ROS project package, add this package to your package.xml, CMakeLists.txt. Then in the C++ implementation file add this to your includes:

```
#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>
```

Add to your class's member variables and properly initialize it correspondingly:
```
// plain version
DavinciSimpleNeedleGrasper simpleNeedleGrasper_;
// or smart pointer version in boost::shared_ptr
DavinciSimpleNeedleGrasperPtr simpleNeedleGrasperPtr_;
```

In your member initialization list of your class constructor:
```
simpleNeedleGrasper_(node_handle, private_node_handle, "psm_one" or "psm_two", "needle_name");
```

To have a private_node_handle, create a ros::NodeHandle initialized with "~", such as:
```
ros::NodeHandle private_node_handle("~");
```

### Modes to grasp the needle
To pick up needle, you can either control the robot to grasp a needle with a set of defined grasping parameters or have the program find a valid grasp pose for you. There are four grasp modes to choose from, "DEFINED", "SUBOPTIMAL", "FINDGOOD" and "RANDOM".

To change the defined grasping parameters, change theta_normal value in file [dvrk_psm_grasp_needle_data.yaml](https://github.com/cwru-davinci/cwru_davinci_grasp/blob/master/config/dvrk_psm_grasp_needle_data.yaml)
```
// pick up needle by defined grasping parameters
simpleNeedleGrasper_.pickNeedle("needle_name", NeedlePickMode::DEFINED);

// or pick up needle by randomly searching through the list of candidates grasp
simpleNeedleGrasper_.pickNeedle("needle_name", NeedlePickMode::RANDOM);

// or pick up needle by sequentially finding the first valid grasp from the pre-sorted list of candidates grasp
simpleNeedleGrasper_.pickNeedle("needle_name", NeedlePickMode::SUBOPTIMAL);

// or pick up needle by sequentially finding the first valid grasp from the unsorted list of candidates grasp
simpleNeedleGrasper_.pickNeedle("needle_name", NeedlePickMode::FINDGOOD);
```

## TODO

Future features to be added to this project:

 - Provides grasp visualization and GUI for user to explicitly select preferred grasp.
 - Considers the interaction between the gripper tip and soft tissue to minimize intrusion.
 - Finishes the place functionality.
