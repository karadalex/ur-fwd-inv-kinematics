L = [0, 0.612, 0.572, 0, 0, 0];
d = [0.128, 0, 0, 0.164, 0.116, 0.092];
a = [0, -pi/2, 0, 0, pi/2, -pi/2];

[M, M_joints] = fwdKinSym(L, d, a);
test_angles = [0.8, pi, -pi, pi/2, pi/2, pi/4];
[pos, orient] = fwdKinPose(M, test_angles);
