% CHECK_KINEMATICS  Tests for forward and inverse kinematics and for the gripper.
%   1. forward_kinematics against the CAD file (SolidWorks pose)
%   2. forward_kinematics and gripper.m against the Simscape model (its own sensors)
%   3. inverse_kinematics -> forward_kinematics round trip for random poses
%   4. targets that must be refused
%   5. the mates of the CAD file (how many joints) and the DH frames against the joint axes
% The output is also saved to results/kinematics_check.txt
%
% MohammadMahdi Faraji, mmyasharfaraji@gmail.com

here = fileparts(mfilename('fullpath'));
log_file = fullfile(here, '..', 'results', 'kinematics_check.txt');
if ~isfolder(fileparts(log_file)), mkdir(fileparts(log_file)); end
if isfile(log_file), delete(log_file); end
diary(log_file);
p = robot_params();
fprintf('Kinematics check, %s\n\n', datetime('today', 'Format', 'yyyy-MM-dd'));

% 1. In the CAD file the tool point is at this point: half way between the two fingertips
%    (point (-0.1, 2.4, -80) of each finger part, from gryphon.xml, converted to the base frame)
cad_tool = [451.925 -122.120 675.339];
T = forward_kinematics(p.cad_pose);
fprintf('1) SolidWorks pose %s\n', mat2str(p.cad_pose));
fprintf('   tool point from CAD   [%.3f %.3f %.3f] mm\n', cad_tool);
fprintf('   forward_kinematics    [%.3f %.3f %.3f] mm   difference %.3f mm\n\n', T(1:3, 4), norm(T(1:3, 4)' - cad_tool));

% 2. Simscape model: put the joints and the gripper at some angles and read the sensors
if license('test', 'SimMechanics')          % the license name of Simscape Multibody
    model = 'gryphon_arm';
    addpath(fullfile(here, '..', 'simulink'));
    load_system(model);
    set_param(model, 'SimMechanicsOpenEditorOnUpdate', 'off');
    poses = [p.home p.grip_home; p.cad_pose 0; 0 0 0 0 30; 90 30 -60 20 10;
             -45 60 -90 -80 25; 110 -15 40 -60 5; -50 70 50 55 30; 30 10 -30 -20 0];
    fprintf('2) Simscape model vs forward_kinematics (tool point) and gripper.m (fingertips)\n');
    joints = find_system(model, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Regexp', 'on', ...
        'ReferenceBlock', '^sm_lib/Joints/');
    fprintf('   the model has %d joints: %s\n', numel(joints), strjoin(sort(strrep(joints, [model '/'], '')), ', '));
    worst = 0;  worst_finger = 0;
    for k = 1:size(poses, 1)
        for i = 1:4
            set_param([model '/theta' num2str(i)], 'Value', num2str(poses(k, i)));
        end
        set_param([model '/grip'], 'Value', num2str(poses(k, 5)));
        out = sim(model, 'StopTime', '1');
        tool = out.tool_mm(end, :);
        T = forward_kinematics(poses(k, 1:4));
        [~, width, tips] = gripper(poses(k, 5));
        tips = T(1:3, 1:3) * tips + T(1:3, 4);
        err = norm(tool' - T(1:3, 4));
        err_finger = max(norm(out.finger1_mm(end, :)' - tips(:, 1)), norm(out.finger2_mm(end, :)' - tips(:, 2)));
        worst = max(worst, err);
        worst_finger = max(worst_finger, err_finger);
        fprintf('   theta = %-26s g = %2d   Simscape [%8.2f %8.2f %8.2f]  FK [%8.2f %8.2f %8.2f]  diff %.4f mm   fingertips %.1f mm apart, diff %.4f mm\n', ...
            mat2str(poses(k, 1:4)), poses(k, 5), tool, T(1:3, 4), err, width, err_finger);
    end
    fprintf('   largest difference: tool point %.4f mm, fingertips %.4f mm\n\n', worst, worst_finger);
    close_system(model, 0);
else
    fprintf('2) Simscape Multibody is not installed, skipped\n\n');
end

% 3. random joint angles -> FK -> IK -> FK again, the tool point must come back to the same point
rng(1);
n = 20000;  solved = 0;  worst = 0;
for k = 1:n
    th = p.limits(:, 1)' + rand(1, 4) .* (p.limits(:, 2) - p.limits(:, 1))';
    T = forward_kinematics(th);
    phi = th(2) + th(3) + th(4);
    for elbow = {'up', 'down'}
        [th2, ok] = inverse_kinematics(T(1:3, 4)', phi, elbow{1});
        if ok
            T2 = forward_kinematics(th2);
            worst = max(worst, norm(T2(1:3, 4) - T(1:3, 4)));
            solved = solved + 1;
        end
    end
end
fprintf('3) IK round trip: %d random poses, %d IK solutions within the limits\n', n, solved);
fprintf('   largest position error: %.1e mm\n\n', worst);

% 4. targets that cannot be reached
fprintf('4) targets that must be refused\n');
% too far, on the base axis, under the floor, gripper folded back into the base, behind the robot
tests = {[900 0 400], 0; [5 3 600], 0; [420 0 -20], -45; [120 -18 100], -170; [-400 -100 300], -45};
for k = 1:size(tests, 1)
    [~, ok, msg] = inverse_kinematics(tests{k, 1}, tests{k, 2});
    fprintf('   %-16s phi %4d: ok = %d (%s)\n', mat2str(tests{k, 1}), tests{k, 2}, ok, msg);
end

% 5. gryphon.xml lists every mate of the assembly, with a point and a direction on each of
%    the two parts. Two parts that can turn have a concentric mate, and its line is the joint
%    axis. In the SolidWorks pose, the z axis of DH frame i-1 must lie on the axis of joint i.
fprintf('\n5) mates in gryphon.xml and the DH frames on the joint axes (SolidWorks pose)\n');
xml_file = fullfile(here, '..', 'simulink', 'gryphon.xml');
mates = read_mates(xml_file);
pairs = cellfun(@(u) strjoin(sort(u), ' + '), {mates.uid}, 'UniformOutput', false);
[pair_names, ~, which] = unique(pairs);
turning = 0;  locked = 0;
for k = 1:numel(pair_names)
    types = {mates(which == k).type};
    turning = turning + any(strcmp(types, 'Concentric'));
    locked = locked + any(strcmp(types, 'UnknownConstraint'));     % the Lock mates
end
fprintf('   %d mates join the %d parts in %d pairs: %d pairs turn about a concentric mate, %d are locked\n', ...
    numel(mates), numel(unique([mates.uid])), numel(pair_names), turning, locked);

% part frame -> base frame, the same base frame as in load_arm_parts.m
pose = read_cad_poses(xml_file);
R0 = [0 0 1; 1 0 0; 0 1 0];
column = pose.(matlab.lang.makeValidName('part8-shaft3-1'));
plate = pose.(matlab.lang.makeValidName('part7-1'));
O0 = [column.t(1); plate.t(2); column.t(3)];
[~, ~, frames] = forward_kinematics(p.cad_pose);
joint_mates = {'waist', 'Concentric2'; 'shoulder', 'Concentric shoulder (arm in body bore)';
               'elbow', 'Concentric7'; 'wrist', 'Concentric11'};
for i = 1:4
    m = mates(strcmp({mates.name}, joint_mates{i, 2}));
    P = pose.(matlab.lang.makeValidName(m.uid{1}));
    point = R0 * (P.R * m.point{1} + P.t - O0);      % a point on the joint axis
    u = R0 * P.R * m.dir{1};
    u = u / norm(u);                                 % direction of the joint axis
    z = frames(1:3, 3, i);                           % z axis of frame i-1
    o = frames(1:3, 4, i);                           % origin of frame i-1
    fprintf('   z%d on the %-8s axis: angle %.4f deg, origin of frame %d is %.4f mm from the axis\n', ...
        i - 1, joint_mates{i, 1}, acosd(min(1, abs(z' * u))), i - 1, norm(cross(o - point, u)));
end
diary off;


function mates = read_mates(xml_file)
% every mate in gryphon.xml: type, name, the two part instances, and on each of them a point
% (mm) and a direction, written in the frame of that part
doc = xmlread(xml_file);
list = doc.getElementsByTagName('Constraints').item(0).getChildNodes;
mates = struct('type', {}, 'name', {}, 'uid', {}, 'point', {}, 'dir', {});
for i = 0:list.getLength - 1
    node = list.item(i);
    if node.getNodeType ~= 1, continue; end          % 1 = an XML element (skip line breaks)
    m = struct('type', char(node.getNodeName), 'name', char(node.getAttribute('name')), ...
        'uid', {{}}, 'point', {{}}, 'dir', {{}});
    geom = node.getElementsByTagName('ConstraintGeometry');
    for k = 0:geom.getLength - 1
        g = geom.item(k);
        m.uid{k + 1} = char(g.getElementsByTagName('Uid').item(0).getTextContent);
        if g.getElementsByTagName('Position').getLength > 0        % a Lock mate has none
            m.point{k + 1} = 1000 * sscanf(char(g.getElementsByTagName('Position').item(0).getTextContent), '%f');
            m.dir{k + 1} = sscanf(char(g.getElementsByTagName('Axis').item(0).getTextContent), '%f');
        end
    end
    mates(end + 1) = m;
end
end
