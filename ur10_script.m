%% Forward Kinematics section
% execute forward kinematics script
forward_script

%% Inverse Kinematics symbolic solutions
% Available in workspace: a, d, L, M, M_joints, orient, pos, test_angles
[th1,th2,th3,th4,th5,th6] = urIKSym(L, d, a, M_joints);

%% Substitute specific position and orientation (M matrix)
M_num = eye(4);
M_num(1:3, 1:3) = orient;
M_num(1:3, 4) = pos;
th1 = eval(subs(th1, sym('M', [3 4]), M_num(1:3, 1:4)));
th2 = subs(th2, sym('M', [3 4]), M_num(1:3, 1:4));
th3 = subs(th3, sym('M', [3 4]), M_num(1:3, 1:4));
th4 = subs(th4, sym('M', [3 4]), M_num(1:3, 1:4));
th5 = subs(th5, sym('M', [3 4]), M_num(1:3, 1:4));
th6 = subs(th6, sym('M', [3 4]), M_num(1:3, 1:4));

%% Build solutionset
% th1Sol = sym(zeros(1,8));
% th2Sol = sym(zeros(1,8));
% th3Sol = sym(zeros(1,8));
% th4Sol = sym(zeros(1,8));
% th5Sol = sym(zeros(1,8));
% th6Sol = sym(zeros(1,8));
th = sym('th', [1, 6]);
for i = 0:1:7
    s = de2bi(i, 3);
    % s(1): LSB , s(6): MSB
    th1Sol(i+1) = th1(s(3)+1);
    th5Sol(i+1) = th5(s(2)+1);
    th3Sol(i+1) = th3(s(1)+1);
    
    th2Sol(i+1) = th2(1);
    th4Sol(i+1) = th4;
    th6Sol(i+1) = th6;
    
    thSol = {th1Sol(i+1), th2Sol(i+1), th3Sol(i+1), th4Sol(i+1), th5Sol(i+1), th6Sol(i+1)};
    th5Sol(i+1) = subs(th5Sol(i+1), th, thSol);
    th5Sol(i+1) = eval(real(th5Sol(i+1)));
    th6Sol(i+1) = subs(th6Sol(i+1), th, thSol);
    th6Sol(i+1) = eval(real(th6Sol(i+1)));
    th3Sol(i+1) = subs(th3Sol(i+1), th, thSol);
    th3Sol(i+1) = eval(real(th3Sol(i+1)));
    th2Sol(i+1) = subs(th2Sol(i+1), th, thSol);
    th2Sol(i+1) = eval(real(th2Sol(i+1)));
    th4Sol(i+1) = subs(th4Sol(i+1), th, thSol);
    th4Sol(i+1) = eval(real(th4Sol(i+1)));
end

%% Testing
test_angles_1 = [th1Sol(8), th2Sol(8), th3Sol(8), th4Sol(8), th5Sol(8), th6Sol(8)];
test_angles_1 = eval(real(test_angles_1));
[test_pos_1, test_orient_1] = fwdKinPose(M, test_angles_1);