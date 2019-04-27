%% Forward Kinematics section
% execute forward kinematics script
forward_script

%% Solve Inverse Kinematics 
% Available in workspace: a, d, L, M, M_joints, orient, pos, test_angles
M_num = eye(4);
M_num(1:3, 1:3) = orient;
M_num(1:3, 4) = pos;
solutions = invKinSym(M, M_joints);

for angle = keys(solutions)
    key = char(angle);
    solutions(key) = subs(solutions(key), sym('M', [3 4]), M_num(1:3, 1:4));
end

th = sym('th', [1, 6]);
th1 = solutions(char(th(1)));
th2 = solutions(char(th(2)));
% th3 = solutions(char(th(3)));
% th4 = solutions(char(th(4)));
th5 = solutions(char(th(5)));
th6 = solutions(char(th(6)));

solutions(char(th(5))) = eval(subs(th5, th(1), th1));
solutions(char(th(6))) = eval(subs(th6, th(1), th1));
solutions(char(th(2))) = eval(subs(th2, {th(1), th(5), th(6)}, {th1, th5, th6}));
% solutions(char(th(3))) = subs(th3, {th(5), th(6)}, {th5, th6});
% solutions(char(th(4))) = subs(th4, {th(1), th(2), th(5), th(6)}, {th1 ,th2, th5, th6});

for angle = keys(solutions)
    key = char(angle);
    disp(key);
    disp(solutions(key));
end

%% Test results
test_ik_angles = [];
[test_ik_pos,test_ik_orient] = fwdKinPose(M, test_ik_angles(1,:));