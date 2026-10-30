# Where the DH table comes from

The Denavit-Hartenberg (DH) table describes the robot with four numbers per joint. If one of them is wrong, every point computed with it lands in the wrong place. This page shows where each number of the table comes from, so anyone with the SolidWorks files can check it.

All numbers were read from `simulink/gryphon.xml`. Simscape Multibody Link writes this file when it exports the assembly. It lists where each part sits and every mate (a constraint between two parts) in the part's own coordinates. The parts themselves were drawn from measurements of the real robot (caliper and tape measure), so these numbers are only as good as those measurements. Lengths are in mm, angles in degrees.

## The table

| link | a (mm) | alpha | d (mm) | theta |
|---|---|---|---|---|
| 1 | 57.3 | 90 | 372 | theta1 (waist) |
| 2 | 224.159 | 0 | 0 | theta2 (shoulder) |
| 3 | 236.475 | 0 | 0 | theta3 (elbow) |
| 4 (gripper) | 134.104 | 0 | -17.723 | theta4 (wrist pitch) |

Standard DH convention: `A_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)`.

The base frame {0} sits on the axis of the column, at the bottom of the base plate, with z pointing up. SolidWorks uses y as "up", so the axes of {0} are `x0 = Z`, `y0 = X`, `z0 = Y` of the CAD file. With this choice x0 runs along the short side of the base plate.

When every angle is zero, the arm is stretched out level along x0, and the gripper points straight ahead.

The last frame {4} sits on the **tool point**: half way between the two fingertips of the closed gripper. Its x axis points along the gripper.

## Each number

**Joint 1, the waist.** The mates `Concentric2` and `Coincident8` put the body (`Part5`) on top of the column (`part8-shaft3`), on the same vertical axis. The top face of the column is 189 mm above the floor.

**d1 = 372 and alpha1 = 90.** In the body, the face that sits on the column is at local height 1 mm, and the bore of the shoulder (mate `Concentric shoulder (arm in body bore)`) at local height 184 mm. So the shoulder axis is 189 + 184 - 1 = 372 mm above the floor. It is horizontal, which is the 90 degree twist alpha1.

**a1 = 57.3.** In the body, the waist axis passes through local x = 32 mm and the shoulder axis through local x = 89.3 mm. The shoulder axis is therefore 89.3 - 32 = 57.3 mm in front of the waist axis. The two axes do not meet, so a1 is not zero.

**a2 = 224.159.** The upper arm (`part3`) has the shoulder bore at local x = 255 mm and the elbow hole (`Concentric7`) at local x = 479.159 mm. Both holes are parallel to the y axis of the part. So a2 = 479.159 - 255 = 224.159 mm. The part itself is 517 mm long, because it goes on 255 mm behind the shoulder.

**a3 = 236.475.** The forearm (`Part1`) has the elbow hole at local x = 150.001 mm and the wrist hole (`Concentric11`) at local x = 386.476 mm, again both parallel to its y axis. So a3 = 236.475 mm.

**a4 = 134.104.** The fingertips are the points of the two fingers (`handpro`) that are farthest from their pins. Both fingers are the same part, so this is one point of the part, local (-0.1, 2.4, -80). In the frame of the hand (`hand1`), the point half way between the two fingertips is at (134.104, 56.223, 0.100), and the wrist axis runs along the y axis of the hand. So the tool point is 134.104 mm from the wrist axis.

**d4 = -17.723.** Not every link sits in the same vertical plane. Seen from above, the body, the upper arm and the forearm are side by side along the joint axes, and the tool point ends 17.723 mm beside the plane that goes through the waist axis. The joint 2, 3 and 4 axes are all parallel, so this sideways offset can go into a single `d`. It is put into d4, so that the last frame sits exactly on the tool point. The inverse kinematics takes it into account: seen from above, the arm does not point exactly at the target (`theta1 = atan2(y, x) + atan2(d4, r)`).

## The gripper

The two fingers turn about their own pins (mates `Concentric22` and `Concentric24`). In the tool frame {4} both pins are parallel to y4, about 80 mm behind the tool point, and 12 mm apart. The gripper angle `g` is how far each finger has turned from the closed pose of the CAD file. The two fingers turn in opposite directions, so they open symmetrically. `gripper.m` gives the fingertips for any `g`:

| g (deg) | 0 | 9.3 | 15 | 30 |
|---|---|---|---|---|
| fingertips apart (mm) | 6.2 | 30 | 45.6 | 85.0 |

The fingers do not move the tool point, because it is fixed to the hand.

## Checking the table against the CAD file

In the saved SolidWorks assembly, the robot stands in one pose. Reading the part positions from the XML gives that pose in DH angles:

    theta = [-17.291  -4.252  63.768  0.513]

The upper arm is almost level, the forearm points up, and the gripper continues the forearm. The same XML gives the tool point in the base frame: `[451.925 -122.120 675.339]`. Forward kinematics with the table above gives `[451.925 -122.119 675.339]`, 0.001 mm away.

There is a second check, which is easy to see: `load_arm_parts.m` attaches every STL mesh to its DH frame in this CAD pose. When `draw_arm.m` then draws the robot in any other pose, the parts stay joined at the joints. With a wrong table, they would drift apart.

A third check looks at the frames themselves. In standard DH, the z axis of frame i-1 must lie on the axis of joint i, and the XML gives every joint axis as the line of its concentric mate. In the CAD pose, z0, z1, z2 and z3 lie on the waist, shoulder, elbow and wrist axes. The largest angle between a z axis and its joint axis is 0.0001 degrees, and every frame origin is at most 0.0005 mm from its axis, which is only the rounding of the table to three decimals.

The same mates also show how many joints there are. The 16 mates join the 10 part instances in 9 pairs. Six pairs have a concentric mate and can turn (the four arm joints and the two fingers), and three pairs are held by a Lock mate. So the assembly has exactly six joints, and the Simscape model built from it has six revolute joints and nothing else.

## Joint limits

My course model moved every joint with a slider, measured from the pose of the CAD file. The limits are the same ranges, written as DH angles: `limits = cad_pose + slider range`.

| joint | slider in the course model | limits (DH, deg) |
|---|---|---|
| 1 waist | -35 to 135 | -52.3 to 117.7 |
| 2 shoulder | -15 to 75 | -19.3 to 70.7 |
| 3 elbow | -160 to -10 (the slider went from 10 to 160 and was multiplied by -1) | -96.2 to 53.8 |
| 4 wrist | -90 to 60 | -89.5 to 60.5 |
| gripper | -1 to 30 | 0 to 30 |

The gripper range starts at 0 instead of -1: below 0 the fingertips would press into each other.

## Angles in the Simulink model

The Simscape joints do not measure their angle from the DH zero. They measure it from the frames of the SolidWorks mates. The conversion is one line, `q = sim_sign .* theta + sim_offset`, for the four arm joints and then the two fingers (which get `g`):

    sim_sign   = [1  1  1  1  1  -1]
    sim_offset = [-123.271  90  -180  -180.043  -179.333  178.222]

- **Offsets:** each offset is the angle of the model joint in the CAD pose minus the DH angle in that pose. For example, the shoulder joint of the model reads 85.748 in the CAD pose, where theta2 is -4.252, so its offset is 90.
- **Joint 4:** -180.043 instead of -180, because the tool point is 0.1 mm off the middle plane of the hand, which turns the gripper direction by atan(0.1 / 134.1) = 0.043 degrees.
- **Finger 2:** smimport made this joint go from the finger to the hand, so it turns the other way and its sign is -1.

The signs were found by turning each joint by +20 degrees in the model and comparing the sensor with forward kinematics. `simulink/make_model.m` builds these sign and offset blocks into the model, so the model takes DH angles and `g` directly.

## Result

`matlab/check_kinematics.m` runs all the checks and writes `results/kinematics_check.txt`:

- **Forward kinematics vs the CAD tool point:** 0.001 mm.
- **Forward kinematics vs the Simscape model:** 0.0010 mm in 8 poses. The model has its own transform sensor between the base and the tool point.
- **gripper.m vs the Simscape model:** 0.0018 mm for both fingertips, in the same 8 poses with different gripper angles.
- **Inverse kinematics followed by forward kinematics:** back to the same point within 1e-12 mm, over 20000 random poses.
- **DH frames vs the joint axes in the CAD file:** z0 to z3 lie on the four joint axes within 0.0001 degrees and 0.0005 mm.
- **Joints:** 6 pairs of parts turn and 3 are locked, and the Simscape model has exactly 6 revolute joints.
