function p = robot_params()
% ROBOT_PARAMS  Dimensions of the Gryphon model and the numbers that connect the math to the
% Simulink model. All lengths are in mm and all angles in degrees.
%
% The lengths are the distances between the joint axes of the SolidWorks assembly
% (cad/gryphon.SLDASM), read from its export simulink/gryphon.xml.
% docs/dh_parameters.md explains where each number comes from.
%
% MohammadMahdi Faraji, mmyasharfaraji@gmail.com

% DH table (standard convention), one column per link
%            link 1   link 2    link 3    link 4 (gripper)
p.a     = [  57.3    224.159   236.475   134.104 ];
p.alpha = [  90        0         0         0     ];
p.d     = [ 372        0         0       -17.723 ];

% The pose in which the robot was drawn in SolidWorks (DH angles)
p.cad_pose = [-17.291 -4.252 63.768 0.513];

% joint limits [min max]. In my course model every joint had a slider that moved it away
% from the pose of the CAD file. The limits here are the same ranges: cad_pose + slider range.
% (The elbow slider went from 10 to 160, but that model multiplied it by -1.)
slider = [-35 135;
          -15  75;
         -160 -10;
          -90  60];
p.limits = p.cad_pose' + slider;

% home pose: upper arm 45 deg up, forearm level, gripper 45 deg down
p.home = [0 45 -45 -45];

% the base, in the base frame {0}, used by check_pose.m to keep the gripper out of it
p.plate = [-77.5 77.5; -175 55; 0 64];   % base plate: x, y and z range
p.body  = [157.4 393];                   % column and the body that turns with joint 1:
                                         % a cylinder around the z0 axis [radius height]

% Gripper. Each finger turns about its own pin. g = 0 is the closed pose of the CAD file,
% a larger g opens it. Pins, axis and fingertips are written in the tool frame {4}.
p.grip_limits = [0 30];
p.grip_home = 15;
p.finger_axis  = [0.000746 1 0];         % both pins are parallel to y4
p.finger_pivot = [-80.100 -80.100;       % one column per finger (handpro-1, handpro-2)
                    4.960   4.960;
                   -6.777   5.223];
p.finger_tip   = [ -0.043   0.043;       % fingertips with the gripper closed. The tool
                   -2.500   2.500;       % point (origin of frame 4) is half way between them.
                   -1.809   1.809];
p.finger_sign  = [1 -1];                 % the two fingers turn in opposite directions

% In the Simulink model every joint measures its angle from the frames of the CAD mates,
% not from the DH zero. This converts a DH angle to the angle of the model joint:
%   q = sim_sign .* theta + sim_offset
% (four arm joints, then the two fingers, which get g; docs/dh_parameters.md explains them)
p.sim_sign   = [1 1 1 1 1 -1];
p.sim_offset = [-123.271 90 -180 -180.043 -179.333 178.222];
end
