% PLOT_WORKSPACE  Where the tool point can go, seen from the side in the plane of the arm.
%   grey : points the tool point can reach with some gripper angle
%          (60000 random joint angles inside the limits, forward kinematics)
%   blue : points where the gripper can also point straight down (phi = -90),
%          for example to pick something from a table (a grid of targets, inverse kinematics)
% Poses that check_pose.m does not allow (under the floor, gripper in the base) are left out.
% The figure is saved to results/figures/workspace.png
%
% MohammadMahdi Faraji, mmyasharfaraji@gmail.com

here = fileparts(mfilename('fullpath'));
p = robot_params();

% 1) random joint angles -> tool point
rng(2);
n = 60000;
th = p.limits(:, 1)' + rand(n, 4) .* (p.limits(:, 2) - p.limits(:, 1))';
tool = zeros(n, 3);
keep = true(n, 1);
for k = 1:n
    T = forward_kinematics(th(k, :));
    tool(k, :) = T(1:3, 4)';
    keep(k) = check_pose(th(k, :));
end
tool = tool(keep, :);
th = th(keep, :);
% distance in front of the base axis, along the direction the arm is turned to
% (negative = the gripper is folded back behind the base axis)
reach = tool(:, 1) .* cosd(th(:, 1)) + tool(:, 2) .* sind(th(:, 1));

% 2) grid of targets in front of the base, can the gripper point straight down there?
[gx, gz] = meshgrid(0:5:750, 0:5:900);
down = false(size(gx));
for k = 1:numel(gx)
    [~, ok_up] = inverse_kinematics([gx(k) 0 gz(k)], -90, 'up');
    [~, ok_down] = inverse_kinematics([gx(k) 0 gz(k)], -90, 'down');
    down(k) = ok_up || ok_down;
end
down_reach = sqrt(max(gx(down) .^ 2 - p.d(4) ^ 2, 0));    % same in-plane distance as above

% 3) figure
fig = figure('Color', 'w', 'Position', [100 100 820 640], 'Visible', 'off');
hold on;
scatter(reach, tool(:, 3), 2, [0.72 0.72 0.72], 'filled');
scatter(down_reach, gz(down), 4, [0.20 0.45 0.80], 'filled');
% the base: plate (its size along x0, the direction of the arm at home), and the cylinder
% of the column and the body
fill(p.plate(1, [1 2 2 1]), p.plate(3, [1 1 2 2]), [0.25 0.27 0.30], 'EdgeColor', 'none');
plot([-1 -1 1 1] * p.body(1), [64 p.body(2) p.body(2) 64], 'k--', 'LineWidth', 1);
[~, home_points] = forward_kinematics(p.home);
home_reach = home_points(1, :) * cosd(p.home(1)) + home_points(2, :) * sind(p.home(1));
plot(home_reach, home_points(3, :), 'k-o', 'LineWidth', 2, 'MarkerFaceColor', 'k', 'MarkerSize', 5);
plot([-500 800] , [0 0], 'k-', 'LineWidth', 1);
axis equal; grid on; box on;
xlim([-500 800]); ylim([-50 1000]);
xlabel('distance from the base axis, along the arm (mm)');
ylabel('height above the floor (mm)');
title('Workspace of the tool point, side view');
legend({'reachable with some gripper angle', 'reachable with the gripper pointing down', ...
    'base plate', 'column and body (check_pose)', 'arm at the home pose'}, ...
    'Location', 'southoutside', 'NumColumns', 2, 'Interpreter', 'none');

out_dir = fullfile(here, '..', 'results', 'figures');
if ~isfolder(out_dir), mkdir(out_dir); end
exportgraphics(fig, fullfile(out_dir, 'workspace.png'), 'Resolution', 150);
fprintf('largest reach in front of the base axis: %.0f mm, highest point: %.0f mm\n', max(reach), max(tool(:, 3)));
fprintf('with the gripper pointing down: from %.0f to %.0f mm in front of the base axis, up to %.0f mm high\n', ...
    min(down_reach), max(down_reach), max(gz(down)));
close(fig);
