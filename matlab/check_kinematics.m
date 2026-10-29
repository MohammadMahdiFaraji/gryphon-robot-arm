% CHECK_KINEMATICS  Tests for forward and inverse kinematics and for the gripper.
%   1. forward_kinematics against the CAD file (SolidWorks pose)
%   2. forward_kinematics and gripper.m against the Simscape model (its own sensors)
%   3. inverse_kinematics -> forward_kinematics round trip for random poses
%   4. targets that must be refused
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
diary off;
