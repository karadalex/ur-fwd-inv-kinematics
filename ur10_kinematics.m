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

%th = invKin(M_0_6, M_joints, L, d, a);
thSym = invKinSym(M_joints, L, d, a);
th = eval(subs(thSym, sym('M', [4 4]), M_0_6));


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

%% Symbolic Jacobian
Jv_sym = geometricJacobian(M, M_joints, 6);
th_sym = sym('th', [1, 6]);
c_sym = sym('c', [1, 6]);
s_sym = sym('s', [1, 6]);
Jv_sym = subs(Jv_sym, conj(th_sym), th_sym);
Jv_sym = subs(Jv_sym, [cos(th_sym), sin(th_sym)], [c_sym, s_sym]);
th234 = sym('th234');
Jv_sym = subs(Jv_sym, th_sym(2)+th_sym(3)+th_sym(4), th234);
Jv_sym = subs(Jv_sym, [cos(th234), sin(th234)], [sym('c234'), sym('s234')]);
th2345 = sym('th2345');
Jv_sym = subs(Jv_sym, th234+th_sym(5), th2345);
Jv_sym = subs(Jv_sym, [cos(th2345), sin(th2345)], [sym('c2345'), sym('s2345')]);
th23 = sym('th23');
Jv_sym = subs(Jv_sym, th_sym(2)+th_sym(3), th23);
Jv_sym = subs(Jv_sym, [cos(th23), sin(th23)], [sym('c23'), sym('s23')]);

Jvv = sym('Jv', [6 6]);
% Convert to latex
for i=1:6
    for j=1:6
        disp([latex(Jvv(i,j)), ' = ', latex(Jv_sym(i,j))]);
    end
end