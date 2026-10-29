function [T, points, frames] = forward_kinematics(theta)
% FORWARD_KINEMATICS  Where the tool point is for given joint angles.
%   [T, points, frames] = forward_kinematics([t1 t2 t3 t4])
%   theta  : joint angles in degrees
%   T      : 4x4 homogeneous matrix of the tool point in the base frame (mm).
%            The tool point is half way between the two fingertips of the closed gripper.
%   points : 3x5 positions of the base, joints 2, 3, 4 and the tool point (for plots)
%   frames : 4x4x5 all DH frames, frames(:,:,1) is the base and frames(:,:,5) the gripper
%
%   MohammadMahdi Faraji, mmyasharfaraji@gmail.com
p = robot_params();

T = eye(4);
points = zeros(3, 5);
frames = repmat(eye(4), 1, 1, 5);
for i = 1:4
    T = T * dh_matrix(p.a(i), p.alpha(i), p.d(i), theta(i));
    points(:, i + 1) = T(1:3, 4);
    frames(:, :, i + 1) = T;
end
end


function A = dh_matrix(a, alpha, d, theta)
% one row of the DH table as a 4x4 matrix
ct = cosd(theta);  st = sind(theta);
ca = cosd(alpha);  sa = sind(alpha);
A = [ct  -st*ca   st*sa  a*ct;
     st   ct*ca  -ct*sa  a*st;
     0    sa      ca     d;
     0    0       0      1];
end
