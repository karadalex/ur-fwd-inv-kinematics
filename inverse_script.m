%% Forward Kinematics section
% execute forward kinematics script
forward_script

%% Generate Inverse Kinematics 
% Available in workspace: a, d, L, M, M_joints, orient, pos, test_angles
M_num = eye(4);
M_num(1:3, 1:3) = orient;
M_num(1:3, 4) = pos;
eqs = invKinSymEquations(M, M_joints);
eqs = subs(eqs, sym('M', [3 4]), M_num(1:3, 1:4));

%% Manually select equations to solve
theta = sym('th', [1, 6]);
theta2 = sym('th_2', [1, 6]); % substitution: theta2 = theta/2
t = sym('t', [1, 6]); % substitution: t = tan(theta2)
%eqsToSolve = [eqs(7),eqs(8)];
eqsToSolve = [eqs(5),eqs(6),eqs(7),eqs(9),eqs(10),eqs(11)];
%eqsToSolve = eqs;
eqsToSolve = rewrite(eqsToSolve, 'tan');
eqsToSolve = subs(eqsToSolve, theta, 2*theta2);
eqsToSolve = subs(eqsToSolve, tan(theta2), t);
disp(eqsToSolve);
solutions = solve(eqsToSolve, t);

%% Test results
th1 = atan(eval(solutions.t1))/2;
th2 = atan(eval(solutions.t2))/2;
th3 = atan(eval(solutions.t3))/2;
th4 = atan(eval(solutions.t4))/2;
th5 = atan(eval(solutions.t5))/2;
th6 = atan(eval(solutions.t6))/2;
test_ik_angles = [th1,th2,th3,th4,th5,th6];
[test_ik_pos,test_ik_orient] = fwdKinPose(M, test_ik_angles(1,:));