function [ok, msg] = check_pose(theta)
% CHECK_POSE  Is this pose allowed? Joint limits, the floor and the base of the robot.
%   [ok, msg] = check_pose([t1 t2 t3 t4])     angles in degrees
% Only the joint points and the tool point are checked, not the thickness of the links,
% so this is a first check and not a full collision test.
p = robot_params();
ok = false;

for i = 1:4
    if theta(i) < p.limits(i, 1) || theta(i) > p.limits(i, 2)
        msg = sprintf('joint %d would need %.1f deg (limit %.1f to %.1f)', i, theta(i), p.limits(i, 1), p.limits(i, 2));
        return
    end
end

[~, points] = forward_kinematics(theta);

% the elbow, the wrist and the tool point must stay above the floor
if any(points(3, 3:5) < 0)
    msg = 'the gripper would go below the floor';
    return
end

% the wrist and the tool point must stay out of the base plate (a box) and out of
% the column and the turning body (a cylinder around the base axis, see robot_params)
for k = 4:5
    q = points(:, k);
    on_plate = all(q >= p.plate(:, 1) & q <= p.plate(:, 2));
    in_body = hypot(q(1), q(2)) < p.body(1) && q(3) < p.body(2);
    if on_plate || in_body
        msg = 'the gripper would hit the base of the robot';
        return
    end
end

ok = true;
msg = 'ok';
end
