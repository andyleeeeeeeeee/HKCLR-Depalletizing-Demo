# Robot Description

This this the description package of the Self-developed Depalletizing Robot arm.

## Intro

This package follows the boilerplate xarco files of the Franka Panda robot.
We have made xarco files for the default Depalletizing arm as well as a customized
vacuum gripper. 

the URDF for the combination of the arm and gripper could be generated with:

```
- robot_arm.urdf.xacro
  - robot_arm.xacro
```

by: `xacro robot_arm.urdf.xacro > robot_arm.xacro`

## Implementation Details

1. This is a manually made description package.

2. The meshes are converted from .SLDPRT files made by mechanical design team, wherein the STL files for collision
   could be further simplified.
   
3. The translations, namely kinematics, between links are carefully obtained from the .SLDASM model.

4. The joint position and velocity limits are exactly the same as real robot, whereas the
   mass and inertial properties of the links are not like the real one due to lack of data.
