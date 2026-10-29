# The original course version

These are the files of the first version, the final project of my Mechatronics course, kept as they were handed in so the old and the new version can be compared. Only the screen recording that went with them is left out.

| file | what it is |
|---|---|
| `assm1023.SLDASM` | the SolidWorks assembly (SolidWorks 2021) |
| `*.SLDPRT` | the nine parts. `part4-shaft2.SLDPRT` is not used in the assembly. The other eight are the same files as in `cad/` |
| `1126.xml` | the export of the assembly for Simscape, first generation (Simscape Multibody Link 5.0) |
| `assm1023 - *.STL` | the ten meshes of that export, one per part instance |
| `assm1023.slx` | the Simscape model built from that export (`mblibv1` library), with five sliders: waist, shoulder, elbow, wrist and gripper |

The course version had the CAD model and the Simscape model with sliders. It had no kinematics code. The DH table, the forward and inverse kinematics, the window and the checks are new in this repository.

## Opening them today

MATLAB R2024b still opens `assm1023.slx`, but it cannot simulate it, because the first generation library was removed from Simscape Multibody. `smimport` also refuses `1126.xml`, since it only reads the second generation format. That is why the assembly in `cad/` was exported again, and the new model in `simulink/` was built from that export.

## Known problems, and where they are

| problem | where |
|---|---|
| The assembly is under defined. The column could turn on the base plate, the elbow shaft could turn in the forearm, and the shoulder shaft could slide along its axis in the upper arm. Two concentric mates (`Concentric4`, `Concentric6`) join the same pair of faces twice | `assm1023.SLDASM` |
| Because of that, the export has extra joints: a revolute joint between the plate and the column, a revolute joint between the forearm and the elbow shaft, a prismatic joint between the shoulder shaft and the upper arm, and a planar joint between the body and the upper arm, which closes a loop. The model holds the first two with joint actuators set to 0 | `1126.xml`, `assm1023.slx` |
| The velocity and acceleration inputs of the joint actuators come from Constant blocks set to 1, while the position comes from the slider | `assm1023.slx`, the Mux blocks in `Subsystem` |
| The two finger joints get the same angle, but one joint goes from the hand to finger 1 and the other from finger 2 to the hand, with opposite axes. So both fingers turn the same way: one opens while the other closes, and they move side by side instead of opening | `assm1023.slx`, blocks `Revolute` and `Revolute1` |
| The elbow slider goes from 10 to 160 degrees, and the model multiplies it by -1. Zero, the pose the robot was drawn in, is outside this range | `assm1023.slx`, `Slider Gain4` and `Constant12` |

[../../docs/changes_from_course_project.md](../../docs/changes_from_course_project.md) explains how the new version handles each of these.
