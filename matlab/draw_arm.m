function h = draw_arm(ax, parts, theta, g, h)
% DRAW_ARM  Draws the robot (the real CAD parts) for the joint angles theta and the
% gripper angle g, all in degrees.
%   h = draw_arm(ax, parts, theta, g)      first call, creates the patches
%   h = draw_arm(ax, parts, theta, g, h)   later calls, only moves them (faster)
% parts comes from load_arm_parts.
[~, ~, frames] = forward_kinematics(theta);
F = gripper(g);

if nargin < 5 || isempty(h)
    h = gobjects(1, numel(parts));
    hold(ax, 'on');
    for k = 1:numel(parts)
        h(k) = patch(ax, 'Faces', parts(k).faces, 'Vertices', move(parts(k), frames, F), ...
            'FaceColor', parts(k).color, 'EdgeColor', 'none', 'FaceLighting', 'gouraud', ...
            'AmbientStrength', 0.4);
    end
    axis(ax, 'equal');
    view(ax, 135, 20);
    camlight(ax, 'headlight');
    xlabel(ax, 'x (mm)'); ylabel(ax, 'y (mm)'); zlabel(ax, 'z (mm)');
    grid(ax, 'on');
else
    for k = 1:numel(parts)
        h(k).Vertices = move(parts(k), frames, F);
    end
end
end


function v = move(part, frames, F)
T = frames(:, :, part.frame);
if part.finger > 0
    T = T * F(:, :, part.finger);        % a finger also turns about its pin
end
v = (T(1:3, 1:3) * part.vertices' + T(1:3, 4))';
end
