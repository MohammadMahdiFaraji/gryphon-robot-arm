function [T, width, tips] = gripper(g)
% GRIPPER  Where the two fingers are for the gripper angle g (deg).
%   [T, width, tips] = gripper(g)
%   g     : how far each finger has turned from the closed pose of the CAD file (0 = closed)
%   T     : 4x4x2, the move of each finger in the tool frame {4}, a turn about its own pin
%   width : distance between the two fingertips in mm
%   tips  : 3x2 fingertips in the tool frame {4}
p = robot_params();
ax = p.finger_axis(:) / norm(p.finger_axis);
T = repmat(eye(4), 1, 1, 2);
tips = zeros(3, 2);
for k = 1:2
    R = axis_rotation(ax, p.finger_sign(k) * g);
    c = p.finger_pivot(:, k);
    T(1:3, 1:3, k) = R;
    T(1:3, 4, k) = c - R * c;                % the pin stays where it is
    tips(:, k) = R * (p.finger_tip(:, k) - c) + c;
end
width = norm(tips(:, 1) - tips(:, 2));
end


function R = axis_rotation(k, angle)
% rotation matrix for a turn about the unit vector k (Rodrigues formula)
K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
R = eye(3) + sind(angle) * K + (1 - cosd(angle)) * (K * K);
end
