# cwru_davinci_grasp
This package is the implementation of object grasp for daVinci surgical robot. With it, robot is able to grasp some objects, needle for instance, and place object to a new location within robot workspace. Both motions of object grasp and placement are solved by MoveIt! motion planner.

The current implementation allows user to either tell robot to grasp a needle with a defined grasping pose or help user to choose a pose within all possible grasping poses. For needle placement, it requires user to define a position vector(x, y, z) with respect to world frame(the global reference frame), the goal location where the needle is going to be placed.

This packge includes:

  - Simple needle grasper (library and main funtion) implementated by MoveIt! motion planner with the use of moveit_msgs/Grasp message.
  - Simple pose-based grasp generator implementated by optimal needle grasping algorithms.
  - A needle grasping data processing program.
  - A .yaml needle grasp config file, inside of which the grasping parameters are defined
  - Test code
 
Developed by [Su Lu](https://github.com/lusu8892/) at the Mercis Lab, Case Western Reserve University.
 
## Install

### Ubuntu Debian
Kinetic:
```
git clone https://github.com/lusu8892/cwru_davinci_grasp
```

### Some other packages needed before compiling
```
git clone https://github.com/JenniferBuehler/convenience-pkgs.git
git clone https://github.com/cwru-robotics/cwru_davinci_moveit.git
```

## How to use
Before use, make sure the updated needle pose is being published to topic "/updated_needle_pose", or remap whatever topic name to this one, the data type should be "geometry_msgs/PoseStamped".

### To use main function
To use davinci_simple_needle_grasper_main function, user needs to tell which PSMs(psm_one or psm_two) is going to be used to grasp needle, the needle's name and to pick or place. The default setting is to use psm_one to pick needle_r. The main function will try to pick needle with defined grasping pose, if it fails, then the motion planner will help user to choose a possible one.

To pick up the needle:
```
roslaunch cwru_davinci_grasp davinci_simple_needle_grasp.launch which_arm:=psm_one needle_name:=needle_r movement:=pick
```

To place the same needle:
```
roslaunch cwru_davinci_grasp davinci_simple_needle_grasp.launch which_arm:=psm_one needle_name:=needle_r movement:=place
```

### To use code in user's program
Within your robot's ROS package, add this package to your package.xml, CMakeLists.txt. Then in whatever C++ file add this to your includes:

```
#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>
```

Add to your class's member variables the following:
```
DavinciSimpleNeedleGrasper simpleNeedleGrasper_;
```

In your member initialization list of your class constructor:
```
simpleNeedleGrasper_(node_handle, private_node_handle, "psm_one" or "psm_two");
```

To have a private_node_handle, create a ros::NodeHandle initialized with "~", such as:
```
ros::NodeHandle private_node_handle("~");
```

To pick up needle, you can either tell robot to grasp needle by defined parameters or to have motion planner decided for you.

To change the defined parameters for needle grasp, change theta_normal value in file [dvrk_psm_grasp_needle_data.yaml](https://github.com/lusu8892/cwru_davinci_grasp/blob/master/config/dvrk_psm_grasp_needle_data.yaml)
```
// pick up needle by defined grasping parameters
simpleNeedleGrasper_.pickNeedle(NeedlePickMode::DEFINED, needle_name);

// or pick up needle by searching through all possible grasping transformations
simpleNeedleGrasper_.pickNeedle(NeedlePickMode::RANDOM, needle_name);
```

To place needle, you need to pass a needle pose goal to placeNeedle() function, The goal pose only requires translation part defined by user, the orientation of needle placement will be decided by motion planner. NOTICE: The goal pose needs to be defined with respect to world frame(the global reference frame). Here is an example:
```
geometry_msgs::Pose needle_pose_goal;

needle_pose_goal.position.x = -0.248;
needle_pose_goal.position.y = 0.0;
needle_pose_goal.position.z = 0.46;

needleGrasper.placeNeedle(needle_pose_goal, needle_name);
```

## TODO

Furture features to be added to this project:

 - Needle hand-off ability
   - Given an initial needle pose and a desired needle grasp pose.
   - One side gripper is able to pick it up with a grasping pose.
   - The other side gripper will then take the needle from the first gripper.
   - The second gripper move the needle to a pose such that the first gripper can grasp the needle with the desired grasp pose.
 - Other objects grasp feature, like surgical thread grasp, tissue grasp.
