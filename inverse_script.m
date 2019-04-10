%% Forward Kinematics section
% execute forward kinematics script
forward_script

%% Inverse Kinematics section
% Available in workspace: a, d, L, M, M_joints, orient, pos, test_angles
M_num = eye(4);
M_num(1:3, 1:3) = orient;
M_num(1:3, 4) = pos;
equations = invKinSym(M, M_joints);
equations = subs(equations, sym('M', [4 4]), M_num);