%% Forward Kinematics
d = [0.1273, 0, 0, 0.163941, 0.1157, 0.0922];
L = [0, -0.612, -0.5723, 0, 0, 0];
a = [pi/2, 0, 0, pi/2, -pi/2, 0];

[M, M_joints] = fwdKinSym(L, d, a);


%% Inverse Kinematics
M_U_0 = pose(-0.2664, -0.2750, -0.8, -pi/2, 0, 0);
%M_U_0 = pose(0, 0, 0, 0, 0, 0);
M_U_TCP = pose(0.2,0.1,0.5,pi,0,pi/2);
M_6_TCP = pose(0, 0, 0.1606, 0, 0, 0);
%M_6_TCP = pose(0, 0, 0, 0, 0, 0);
M_0_6 = inv(M_U_0) * M_U_TCP * inv(M_6_TCP);

th = invKin(M_0_6, M_joints, L, d, a);


%% Solutions validation
for i=1:8
    solution = th(:,i)';
    M_test = eval(subs(M, sym('th', [1, 6]), solution));
    M_test_U_TCP = M_U_0 * M_test * M_6_TCP;
    disp(M_test_U_TCP - M_U_TCP);
end

%% Calculate the Jacobian
disp("Jacobians:");
for i=1:8
    solution = th(:,i)';
    M_test = eval(subs(M, sym('th', [1, 6]), solution));
    Jv = geometricJacobianNum(solution, M, M_joints, 6);
    disp(Jv);
end