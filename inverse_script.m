%% Forward Kinematics section
% execute forward kinematics script
forward_script

%% Solve Inverse Kinematics Generic
% Available in workspace: a, d, L, M, M_joints, orient, pos, test_angles
M_num = eye(4);
M_num(1:3, 1:3) = orient;
M_num(1:3, 4) = pos;
generalSolutions = invKinSym(M_joints);
% generalSolutions = invKinNum(M, M_joints);

%% Substitute specific position and orientation (M matrix)
solutions = generalSolutions;
for angle = keys(solutions)
    key = char(angle);
    solutions(key) = subs(solutions(key), sym('M', [3 4]), M_num(1:3, 1:4));
end

% Helper variables
th = sym('th', [1, 6]);
th1 = eval(solutions(char(th(1))));
th2 = solutions(char(th(2)));
th3 = solutions(char(th(3)));
th4 = solutions(char(th(4)));
th5 = solutions(char(th(5)));
th6 = solutions(char(th(6)));

%% Build solutionset
for i = 0:1:63
    s = de2bi(i, 6);
    % s(1): LSB , s(6): MSB
    th1Sol(i+1) = th1(s(6)+1);
    th5Sol(i+1) = th5(s(5)+1);
    th6Sol(i+1) = th6(s(4)+1);
    th2Sol(i+1) = th2(s(3)+1);
    th3Sol(i+1) = th3(s(2)+1);
    th4Sol(i+1) = th4(s(1)+1);
    
    thSol = {th1Sol(i+1), th2Sol(i+1), th3Sol(i+1), th4Sol(i+1), th5Sol(i+1), th6Sol(i+1)};
    th5Sol(i+1) = subs(th5Sol(i+1), th, thSol);
    
    th6Sol(i+1) = subs(th6Sol(i+1), th, thSol);
    th6Sol(i+1) = subs(th6Sol(i+1), th, thSol);
    
    th2Sol(i+1) = subs(th2Sol(i+1), th, thSol);
    th2Sol(i+1) = subs(th2Sol(i+1), th, thSol);
    th2Sol(i+1) = subs(th2Sol(i+1), th, thSol);
    
    th3Sol(i+1) = subs(th3Sol(i+1), th, thSol);
    th3Sol(i+1) = subs(th3Sol(i+1), th, thSol);
    th3Sol(i+1) = subs(th3Sol(i+1), th, thSol);
    th3Sol(i+1) = subs(th3Sol(i+1), th, thSol);
    
    th4Sol(i+1) = subs(th4Sol(i+1), th, thSol);
    th4Sol(i+1) = subs(th4Sol(i+1), th, thSol);
    th4Sol(i+1) = subs(th4Sol(i+1), th, thSol);
    th4Sol(i+1) = subs(th4Sol(i+1), th, thSol);
    th4Sol(i+1) = subs(th4Sol(i+1), th, thSol);
end

%% Display solutions
for i = 1:1:64
    thiSol = [th1Sol(i), th2Sol(i), th3Sol(i), th4Sol(i), th5Sol(i), th6Sol(i)];
    thiSol = eval(real(thiSol));
    disp(thiSol);
end


%% Test results
test_ik_angles = [th1Sol(1), th2Sol(1), th3Sol(1), th4Sol(1), th5Sol(1), th6Sol(1)];
test_ik_angles = eval(real(test_ik_angles));
[test_ik_pos,test_ik_orient] = fwdKinPose(M, test_ik_angles);