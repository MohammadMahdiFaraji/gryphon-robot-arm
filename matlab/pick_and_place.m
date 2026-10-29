% PICK_AND_PLACE  The robot picks up a 30 mm block from the table and puts it down 320 mm
% further on, with the gripper pointing straight down the whole time.
% The moves down, up and across are straight lines, solved point by point with inverse
% kinematics (plan_line.m). The moves from and back to home are joint space moves.
% Saves results/figures/pick_and_place_joints.png and media/pick_and_place.gif
%
% MohammadMahdi Faraji, mmyasharfaraji@gmail.com

here = fileparts(mfilename('fullpath'));
p = robot_params();
parts = load_arm_parts();

phi = -90;                   % gripper pointing down
edge = 30;                   % edge of the block (mm)
pick  = [380 -160 edge / 2]; % centre of the block before and after
place = [380  160 edge / 2];
up = [0 0 150];              % the moves across are 150 mm higher
g_open = 20;                                              % gripper open, about 59 mm
g_hold = fzero(@(g) width_of(g) - edge, [0 g_open]);      % fingertips on the block

% ---- the path: one row per step [theta1..4 g], and whether the block is in the gripper
[above, ok, msg] = inverse_kinematics(pick + up, phi, 'up');
if ~ok, error('above the block: %s', msg); end
path = joint_move([p.home p.grip_home], [above g_open]);   % home -> above the block
holding = false(size(path, 1), 1);
lines = zeros(0, 2);                                        % first and last row of each line
moves = {pick,         g_open, false     % down to the block
         [],           g_hold, true      % close the gripper
         pick + up,    g_hold, true      % up
         place + up,   g_hold, true      % across
         place,        g_hold, true      % down
         [],           g_open, false     % open the gripper
         place + up,   g_open, false};   % up
for k = 1:size(moves, 1)
    if isempty(moves{k, 1})
        rows = gripper_move(path(end, :), moves{k, 2});
    else
        rows = line_move(path(end, :), moves{k, 1}, phi);
        lines(end + 1, :) = size(path, 1) + [0 size(rows, 1)]; %#ok<SAGROW>
    end
    path = [path; rows]; %#ok<AGROW>
    holding = [holding; repmat(moves{k, 3}, size(rows, 1), 1)]; %#ok<AGROW>
end
home = joint_move(path(end, :), [p.home p.grip_home]);     % back home
path = [path; home(2:end, :)];
holding = [holding; false(size(home, 1) - 1, 1)];

% tool point of every step, and how straight the lines are
tool = zeros(size(path, 1), 3);
for k = 1:size(path, 1)
    T = forward_kinematics(path(k, 1:4));
    tool(k, :) = T(1:3, 4)';
end
worst = 0;
for k = 1:size(lines, 1)
    pts = tool(lines(k, 1):lines(k, 2), :);
    u = (pts(end, :) - pts(1, :)) / norm(pts(end, :) - pts(1, :));
    d = pts - pts(1, :);
    worst = max(worst, max(vecnorm(d - (d * u') * u, 2, 2)));
end
fprintf('%d steps, %d straight lines, largest distance of the tool point from its line: %.1e mm\n', ...
    size(path, 1), size(lines, 1), worst);
fprintf('the gripper closes to g = %.2f deg (fingertips %.1f mm apart)\n', g_hold, width_of(g_hold));

% ---- joint angles during the demo
fig = figure('Color', 'w', 'Position', [100 100 900 420], 'Visible', 'off');
plot(path, 'LineWidth', 1.5);
grid on;
xlabel('step');  ylabel('angle (deg)');
legend({'\theta_1', '\theta_2', '\theta_3', '\theta_4', 'gripper g'}, 'Location', 'eastoutside');
title('Pick and place: joint angles and gripper');
exportgraphics(fig, fullfile(here, '..', 'results', 'figures', 'pick_and_place_joints.png'), 'Resolution', 150);
close(fig);

% ---- animation
fig = figure('Color', 'w', 'Position', [100 100 560 470], 'Visible', 'off');
ax = axes(fig);
patch(ax, [-250 650 650 -250], [-450 -450 450 450], [0 0 0 0], [0.94 0.94 0.94], ...
    'EdgeColor', 'none', 'FaceLighting', 'none');
h = draw_arm(ax, parts, path(1, 1:4), path(1, 5));
[cube_v, cube_f] = cube(edge);
c = cosd(above(1));  s = sind(above(1));
block_xyz = cube_v * [c -s 0; s c 0; 0 0 1]' + pick;     % turned like the gripper (no wrist roll)
block = patch(ax, 'Faces', cube_f, 'Vertices', block_xyz, 'FaceColor', [0.80 0.20 0.20], ...
    'EdgeColor', [0.4 0.1 0.1]);
trail = plot3(ax, nan, nan, nan, 'r-', 'LineWidth', 1.5);
xlim(ax, [-250 650]); ylim(ax, [-450 450]); zlim(ax, [0 850]);
view(ax, -40, 25);
title(ax, 'Pick and place with straight lines');

frames = {};
in_gripper = [];                 % corners of the block in the tool frame while it is held
for k = 1:size(path, 1)
    h = draw_arm(ax, parts, path(k, 1:4), path(k, 5), h);
    T = forward_kinematics(path(k, 1:4));
    if holding(k)
        if isempty(in_gripper)
            in_gripper = T \ [block_xyz'; ones(1, 8)];
        end
        block_xyz = (T(1:3, :) * in_gripper)';
    else
        in_gripper = [];
    end
    block.Vertices = block_xyz;
    set(trail, 'XData', tool(1:k, 1), 'YData', tool(1:k, 2), 'ZData', tool(1:k, 3));
    if mod(k, 2) == 1 || k == size(path, 1)
        f = getframe(fig);
        frames{end + 1} = f.cdata; %#ok<SAGROW>
    end
end
close(fig);
fprintf('the block ends at [%.1f %.1f %.1f], the target was [%.1f %.1f %.1f]\n', mean(block_xyz), place);

% one colour table for all frames keeps the GIF small and without flicker
[~, map] = rgb2ind(cat(1, frames{1:10:end}), 128, 'nodither');
gif_file = fullfile(here, '..', 'media', 'pick_and_place.gif');
if ~isfolder(fileparts(gif_file)), mkdir(fileparts(gif_file)); end
for k = 1:numel(frames)
    img = rgb2ind(frames{k}, map, 'nodither');
    if k == 1
        imwrite(img, map, gif_file, 'gif', 'LoopCount', inf, 'DelayTime', 0.06);
    else
        imwrite(img, map, gif_file, 'gif', 'WriteMode', 'append', 'DelayTime', 0.06);
    end
end
fprintf('saved %s (%d frames)\n', gif_file, numel(frames));


function rows = joint_move(from, to)
% every joint and the gripper turn together, starting and stopping smoothly
s = (1 - cos(pi * linspace(0, 1, 40)')) / 2;
rows = from + s .* (to - from);
end

function rows = line_move(last, target, phi)
% straight line of the tool point from where it is now, the gripper stays as it is
[seg, ok, msg] = plan_line(last(1:4), target, phi, 30);
if ~ok, error('line to %s: %s', mat2str(target), msg); end
rows = [seg(2:end, :), last(5) * ones(size(seg, 1) - 1, 1)];
end

function rows = gripper_move(last, g)
% the arm stays where it is, the fingers open or close
rows = [repmat(last(1:4), 13, 1), linspace(last(5), g, 13)'];
rows = rows(2:end, :);
end

function w = width_of(g)
[~, w] = gripper(g);
end

function [v, f] = cube(a)
% corners and faces of a cube with edge a, centred at the origin
v = a / 2 * [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1];
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end
