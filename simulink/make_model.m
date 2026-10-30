% MAKE_MODEL  Builds gryphon_arm.slx from the SolidWorks export.
%
% Steps:
%   1. smimport reads gryphon.xml (made with Simscape Multibody Link) and the STEP files
%   2. every joint gets a motion input: the four arm joints take their angle in the DH
%      convention, and the two fingers take the gripper angle g
%   3. a base frame and a tool point frame are added, with a sensor between them, so the
%      model itself reports where the tool point is (to check forward_kinematics.m).
%      Two more sensors report the fingertips (to check gripper.m).
% Run it again if the robot is exported from SolidWorks again.
%
% MohammadMahdi Faraji, mmyasharfaraji@gmail.com

here = fileparts(mfilename('fullpath'));
cd(here);
addpath(fullfile(here, '..', 'matlab'));
p = robot_params();
model = 'gryphon_arm';

if bdIsLoaded(model), close_system(model, 0); end
if isfile([model '.slx']), delete([model '.slx']); end
if isfile('gryphon_arm_data.m'), delete('gryphon_arm_data.m'); end
% Simscape Multibody Link cannot export the three Lock mates of the assembly (column on the
% plate, shoulder shaft in the body, elbow shaft in the forearm). smimport warns about them
% and joins those parts rigidly, which is what a Lock mate means anyway.
warning('off', 'sm:import:xml:cststojoint:UnknownConstSetToWeld');
smimport('gryphon.xml', 'ModelName', model, 'DataFileName', 'gryphon_arm_data');

% ---- joints: angle comes from outside, torque is computed by Simscape
% names given by smimport, the new name, and the two parts each joint must connect
joints = {'Revolute4', 'Joint1',  'part8_shaft3_1_RIGID', 'Part5_1_RIGID';
          'Revolute5', 'Joint2',  'Part5_1_RIGID',        'part3_1_RIGID';
          'Revolute3', 'Joint3',  'part3_1_RIGID',        'Part1_1_RIGID';
          'Revolute2', 'Joint4',  'Part1_1_RIGID',        'hand1_1_RIGID';
          'Revolute',  'Finger1', 'hand1_1_RIGID',        'handpro_1_RIGID';
          'Revolute1', 'Finger2', 'handpro_2_RIGID',      'hand1_1_RIGID'};
inputs = {'theta1', 'theta2', 'theta3', 'theta4', 'grip'};
values = [p.home p.grip_home];
for i = 1:5
    x = 40;  y = 60 + 110 * (i - 1);
    add_block('simulink/Sources/Constant', [model '/' inputs{i}], 'Value', num2str(values(i)), ...
        'Position', [x y x+50 y+30]);
end
for i = 1:6
    check_joint(model, joints(i, :));
    j = [model '/' joints{i, 2}];
    set_param([model '/' joints{i, 1}], 'Name', joints{i, 2});
    set_param(j, 'TorqueActuationMode', 'ComputedTorque', 'MotionActuationMode', 'InputMotion', ...
        'PositionTargetSpecify', 'off');

    % theta (DH, deg) or g -> sign and offset -> physical signal for the joint
    x = 150;  y = 60 + 110 * (min(i, 5) - 1) + 50 * (i == 6);
    add_block('simulink/Math Operations/Gain', [model '/sign' num2str(i)], 'Gain', num2str(p.sim_sign(i)), ...
        'Position', [x y x+30 y+30]);
    add_block('simulink/Math Operations/Bias', [model '/offset' num2str(i)], 'Bias', num2str(p.sim_offset(i), 8), ...
        'Position', [x+60 y x+110 y+30]);
    add_block('nesl_utility/Simulink-PS Converter', [model '/to ' joints{i, 2}], 'Unit', 'deg', ...
        'FilteringAndDerivatives', 'filter', 'SimscapeFilterOrder', '2', 'InputFilterTimeConstant', '0.1', ...
        'Position', [x+150 y x+180 y+30]);
    add_line(model, [inputs{min(i, 5)} '/1'], ['sign' num2str(i) '/1']);
    add_line(model, ['sign' num2str(i) '/1'], ['offset' num2str(i) '/1']);
    add_line(model, ['offset' num2str(i) '/1'], ['to ' joints{i, 2} '/1']);
    conv = get_param([model '/to ' joints{i, 2}], 'PortHandles');
    joint = get_param(j, 'PortHandles');
    add_line(model, conv.RConn(1), joint.LConn(2));
end

% ---- base frame {0}: on the column axis at the bottom of the base plate, z up (the CAD
% uses y up). In the frame of the plate this point is at (175, 0, -77.5) mm, and the axes
% of {0} are the z, x and y axes of the CAD.
base = [model '/part7_1_RIGID'];
add_block('sm_lib/Frames and Transforms/Rigid Transform', [base '/Frame 0'], ...
    'RotationMethod', 'RotationMatrix', 'RotationMatrix', '[0 1 0; 0 0 1; 1 0 0]', ...
    'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', '[175 0 -77.5]', ...
    'TranslationCartesianOffsetUnits', 'mm');
add_block('nesl_utility/Connection Port', [base '/F0'], 'Side', 'Right');
connect_frames(base, 'ReferenceFrame', 'Frame 0', 'F0');

% ---- tool point: half way between the fingertips of the closed gripper,
% written in the frame of the hand part (hand1)
tool = [model '/hand1_1_RIGID'];
add_block('sm_lib/Frames and Transforms/Rigid Transform', [tool '/Tool point'], ...
    'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', '[134.104 56.223 0.100]', ...
    'TranslationCartesianOffsetUnits', 'mm');
add_block('nesl_utility/Connection Port', [tool '/TCP'], 'Side', 'Right');
connect_frames(tool, 'ReferenceFrame', 'Tool point', 'TCP');

% ---- fingertips, in the frame of the finger part (both fingers are the same part, handpro)
fingers = {[model '/handpro_1_RIGID'], [model '/handpro_2_RIGID']};
for k = 1:2
    add_block('sm_lib/Frames and Transforms/Rigid Transform', [fingers{k} '/Fingertip'], ...
        'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', '[-0.1 2.4 -80]', ...
        'TranslationCartesianOffsetUnits', 'mm');
    add_block('nesl_utility/Connection Port', [fingers{k} '/Tip'], 'Side', 'Right');
    connect_frames(fingers{k}, 'ReferenceFrame', 'Fingertip', 'Tip');
end

% ---- sensors: position of the tool point and of the fingertips in frame {0}, in mm
add_sensor(model, base, tool, 'Tool sensor', 'tool point (mm)', 'tool_mm');
add_sensor(model, base, fingers{1}, 'Finger 1 sensor', 'fingertip 1 (mm)', 'finger1_mm');
add_sensor(model, base, fingers{2}, 'Finger 2 sensor', 'fingertip 2 (mm)', 'finger2_mm');

Simulink.BlockDiagram.arrangeSystem(model);
set_param(model, 'StopTime', '10');

% smimport keeps the part data (masses, colours, frames) in gryphon_arm_data.m and the model
% points to that file with its full path on this computer. Keep the data inside the
% model instead, so the model also opens on another computer.
ws = get_param(model, 'ModelWorkspace');
ws.FileName = 'gryphon_arm_data.m';      % without the folder, the model still remembers this name
ws.DataSource = 'Model File';
delete('gryphon_arm_data.m');
save_system(model);
fprintf('saved %s.slx\n', model);


function check_joint(model, row)
% stop if smimport gave the joints other names than when this file was written
h = get_param([model '/' row{1}], 'LineHandles');
ends = {};
for l = [h.LConn(:); h.RConn(:)]'
    if l <= 0, continue; end
    b = [get_param(l, 'SrcBlockHandle'); get_param(l, 'DstBlockHandle')];
    ends = [ends, arrayfun(@(x) get_param(x, 'Name'), b(b > 0)', 'UniformOutput', false)];
end
if ~all(ismember(row(3:4), ends))
    error('%s does not join %s and %s. Was the CAD exported again? Update the joint list.', row{1}, row{3}, row{4});
end
end

function connect_frames(sys, ref_block, transform_block, port_block)
% Reference frame -> rigid transform -> connection port, inside a part subsystem.
% The two new blocks go under the blocks that smimport made, so they do not sit on top of them.
old = setdiff(find_system(sys, 'SearchDepth', 1, 'Type', 'Block'), {sys, [sys '/' transform_block], [sys '/' port_block]});
pos = cell2mat(get_param(old, 'Position'));
y = max(pos(:, 4)) + 40;
x = get_param([sys '/' ref_block], 'Position');
set_param([sys '/' transform_block], 'Position', [x(1) + 150, y, x(1) + 190, y + 40]);
set_param([sys '/' port_block], 'Position', [x(1) + 300, y + 12, x(1) + 330, y + 28], ...
    'Orientation', 'left');                      % like the ports smimport made

r = get_param([sys '/' ref_block], 'PortHandles');
t = get_param([sys '/' transform_block], 'PortHandles');
c = get_param([sys '/' port_block], 'PortHandles');
add_line(sys, r.RConn(1), t.LConn(1), 'autorouting', 'on');
add_line(sys, t.RConn(1), c.RConn(1), 'autorouting', 'on');
end

function add_sensor(model, base, follower, name, display_name, variable)
% Transform Sensor between the last port of two part subsystems, x, y and z in mm
add_block('sm_lib/Frames and Transforms/Transform Sensor', [model '/' name], ...
    'MeasurementFrame', 'Base', 'SenseX', 'on', 'SenseY', 'on', 'SenseZ', 'on');
b = get_param(base, 'PortHandles');
f = get_param(follower, 'PortHandles');
s = get_param([model '/' name], 'PortHandles');
add_line(model, b.RConn(end), s.LConn(1));
add_line(model, f.RConn(end), s.RConn(1));
mux = [name ' xyz'];
add_block('simulink/Signal Routing/Mux', [model '/' mux], 'Inputs', '3');
xyz = {'x', 'y', 'z'};
for k = 1:3
    conv = [name ' ' xyz{k}];
    add_block('nesl_utility/PS-Simulink Converter', [model '/' conv], 'Unit', 'mm');
    c = get_param([model '/' conv], 'PortHandles');
    add_line(model, s.RConn(1 + k), c.LConn(1));
    add_line(model, [conv '/1'], [mux '/' num2str(k)]);
end
add_block('simulink/Sinks/Display', [model '/' display_name]);
add_block('simulink/Sinks/To Workspace', [model '/' variable], 'VariableName', variable, 'SaveFormat', 'Array');
add_line(model, [mux '/1'], [display_name '/1']);
add_line(model, [mux '/1'], [variable '/1']);
end
