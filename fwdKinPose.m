function [position,orientation] = fwdKinPose(M,joint_angles)
    M_numeric = eval(subs(M, sym('th', [1, 6]), joint_angles));
    position = M_numeric(1:3, 4);
    orientation = M_numeric(1:3, 1:3);
end

