function parts = load_arm_parts()
% LOAD_ARM_PARTS  The ten STL meshes of the robot, each one stored relative to the DH frame
% it moves with.
%
% gryphon.xml says where every part sits in the SolidWorks pose (p.cad_pose). In that pose
% I compute the DH frames with forward_kinematics and keep each mesh relative to its frame.
% After that, drawing the robot for any angles only means moving every mesh with its frame
% (see draw_arm.m). If the DH table were wrong, the parts would come apart in the picture.
here = fileparts(mfilename('fullpath'));
p = robot_params();

% part instance, STL file, DH frame it moves with (1 = base ... 5 = gripper), finger number
list = {
    'part7-1',        'part7',        1, 0     % base plate
    'part8-shaft3-1', 'part8-shaft3', 1, 0     % column
    'Part5-1',        'Part5',        2, 0     % body, turns with joint 1
    'part6-shaft3-1', 'part6-shaft3', 2, 0     % shoulder shaft, fixed in the body
    'part3-1',        'part3',        3, 0     % upper arm
    'Part1-1',        'Part1',        4, 0     % forearm
    'part6-shaft3-2', 'part6-shaft3', 4, 0     % elbow shaft, fixed in the forearm
    'hand1-1',        'hand1',        5, 0     % hand
    'handpro-1',      'handpro',      5, 1     % finger 1
    'handpro-2',      'handpro',      5, 2     % finger 2
    };
colors = [0.25 0.27 0.30; 0.75 0.75 0.78; 0.42 0.47 0.55; 0.75 0.75 0.78; 0.42 0.47 0.55;
          0.42 0.47 0.55; 0.75 0.75 0.78; 0.75 0.75 0.78; 0.85 0.62 0.25; 0.85 0.62 0.25];

% where each part is in the CAD file (mm, y axis up)
pose = read_cad_poses(fullfile(here, '..', 'simulink', 'gryphon.xml'));

% base frame {0}: on the axis of the column (joint 1), at the bottom of the base plate,
% z up instead of y up, x along the short side of the plate
R0 = [0 0 1; 1 0 0; 0 1 0];
column = pose.(matlab.lang.makeValidName('part8-shaft3-1'));
plate = pose.(matlab.lang.makeValidName('part7-1'));
O0 = [column.t(1); plate.t(2); column.t(3)];

[~, ~, frames] = forward_kinematics(p.cad_pose);
n = size(list, 1);
parts = struct('name', list(:, 1), 'faces', [], 'vertices', [], 'color', [], 'frame', [], 'finger', []);
for k = 1:n
    mesh = stlread(fullfile(here, '..', 'cad', 'stl', [list{k, 2} '.STL']));      % mm, part frame
    P = pose.(matlab.lang.makeValidName(list{k, 1}));
    % part frame -> base frame
    T_part = [R0 * P.R, R0 * (P.t - O0); 0 0 0 1];
    % part frame relative to the DH frame it is fixed to
    T_rel = frames(:, :, list{k, 3}) \ T_part;
    v = T_rel(1:3, 1:3) * mesh.Points' + T_rel(1:3, 4);
    parts(k).faces = mesh.ConnectivityList;
    parts(k).vertices = v';
    parts(k).color = colors(k, :);
    parts(k).frame = list{k, 3};
    parts(k).finger = list{k, 4};
end
end


function pose = read_cad_poses(xml_file)
% rotation and translation (mm) of every part instance in gryphon.xml
doc = xmlread(xml_file);
list = doc.getElementsByTagName('Instance');
pose = struct();
for i = 0:list.getLength - 1
    node = list.item(i);
    uid = char(node.getAttribute('uid'));
    R = sscanf(char(node.getElementsByTagName('Rotation').item(0).getTextContent), '%f');
    t = sscanf(char(node.getElementsByTagName('Translation').item(0).getTextContent), '%f');
    pose.(matlab.lang.makeValidName(uid)).R = reshape(R, 3, 3)';   % the file lists the rows
    pose.(matlab.lang.makeValidName(uid)).t = 1000 * t(:);         % m -> mm
end
end
