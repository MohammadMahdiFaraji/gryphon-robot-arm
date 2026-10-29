# What changed since the course version

The first version of this project was the final project of my Mechatronics course: the Gryphon robot drawn in SolidWorks from my measurements, exported to Simscape, and a Simscape model with five sliders. Those original files are kept in `legacy/original_course_submission/`, so the two versions can be compared.

When the project was opened again for this repository, the original files were checked one by one. The parts themselves were kept exactly as they were. The list below says what was wrong in the rest and what the new version does instead.

## SolidWorks assembly

**The assembly was under defined.** The column could turn on the base plate, the elbow shaft could turn in the forearm, and the shoulder shaft could slide along its axis in the upper arm. Two concentric mates joined the same pair of faces twice. A copy of the assembly was fixed and saved as `cad/gryphon.SLDASM`:

- Lock mates hold the column on the base plate and the two shafts in the parts they belong to.
- The upper arm turns in the bore of the body (one concentric mate), instead of on the shoulder shaft with two concentric mates.

Now only the six real joints can move: waist, shoulder, elbow, wrist pitch and the two fingers. The eight part files in `cad/` are the original files, unchanged.

**The wrist roll is not modelled.** The real Gryphon EC has five axes and a gripper. The model, like the course version, has the waist, the shoulder, the elbow, the wrist pitch and the gripper. The roll of the gripper about its own axis is missing, so the hand is fixed to the wrist pitch joint.

## Simulink model

**It no longer runs in current MATLAB.** The old model was built with the first generation of Simscape Multibody (the `mblibv1` library). MATLAB R2024b can still open it, but it cannot simulate it, and `smimport` refuses the first generation XML. The fixed assembly was exported again with Simscape Multibody Link (second generation, XML plus STEP files), and `simulink/make_model.m` rebuilds the model from it with one command.

**Extra joints are gone.** Because the old assembly was under defined, the old export had a revolute joint between the plate and the column, a revolute joint between the forearm and the elbow shaft, a prismatic joint between the shoulder shaft and the upper arm, and a planar joint between the body and the upper arm that closed a loop. The old model held two of them with joint actuators set to 0. The new export has exactly six revolute joints. The three Lock mates are exported as unknown constraints, and `smimport` joins those parts rigidly, which is what a Lock mate means.

**Velocity and acceleration inputs were fixed.** Each old joint actuator received the slider position, and a velocity and an acceleration from Constant blocks set to 1. In the new model each joint gets only its angle, and the Simulink-PS converter computes smooth derivatives from it.

**The fingers moved the same way.** Both finger joints of the old model got the same angle, but the two joints had opposite axes, so one finger opened while the other closed. In the new model the gripper angle `g` goes to each finger with its own sign, and the fingers open symmetrically. A sensor on each fingertip checks this against `gripper.m`.

**The joint angles did not mean the same thing as in the math.** Simscape measures each joint from the frames of the SolidWorks mates. The new model has a sign and an offset block for each joint (`sim_sign` and `sim_offset` in `robot_params.m`), so it takes DH angles directly. A transform sensor measures the tool point, and it agrees with `forward_kinematics.m` within 0.001 mm.

## New in this version

The course version had no kinematics code. Everything in `matlab/` is new:

- `robot_params.m`: the DH table read from the CAD export, joint limits, home pose, the gripper, and the numbers for the Simulink model. `docs/dh_parameters.md` shows where every number comes from.
- `forward_kinematics.m`, `inverse_kinematics.m` (closed form, with the gripper angle as input and elbow up or down), `check_pose.m` (joint limits, floor and base), `plan_line.m` (straight lines) and `gripper.m` (the two fingers).
- `load_arm_parts.m` and `draw_arm.m`: the real CAD parts drawn in 3D for any pose.
- `arm_gui.m`: a window with sliders for the joints and the gripper, inverse kinematics on joint space or straight line paths, Home, Reset view, Start and Stop, and a switch that runs the Simscape model and makes it follow the window.
- `check_kinematics.m`: all checks in one script, with the results saved in `results/kinematics_check.txt`.
- `plot_workspace.m` and `pick_and_place.m`: the reachable area seen from the side, and a demo that picks up a block and puts it down with straight line moves.
- `click_to_fast_start` in the main folder, which opens the window with one click.

The joint limits are the slider ranges of the course model, converted to DH angles (`docs/dh_parameters.md`, section Joint limits). Only the gripper now starts at 0 instead of -1.

The STL meshes in `cad/stl/` were exported again, one per part. SolidWorks moves every STL so that it starts at zero ("translate to positive space"), so each mesh was moved back by the corner of the part's bounding box. Now every mesh sits in the frame of its part, like the positions in `gryphon.xml`.
