function th = invKin(M_0_6, M_joints, L, d, a)
% Calculate inverse kinmeatics angles
th = ones(6,8);

% theta 1
p_0_5 = M_0_6 * [0;0;-d(6);1];
th1_a = atan2(p_0_5(2), p_0_5(1));
r = sqrt(p_0_5(1)^2 + p_0_5(2)^2);
th1_b = acos(d(4)/r);
th(1,1:4) = th1_a + th1_b + pi/2;
th(1,5:8) = th1_a - th1_b + pi/2;
th(1,:) = real(th(1,:));


% theta 5
for i=1:4:8
    th1 = th(1,i);
    M_0_1 = eval(subs(M_joints(:,:,1), sym('th1'), th1));
    M_1_0 = eval(invTransf(M_0_1));
    M_1_6 = M_1_0 * M_0_6;
    
    ac5 = acos((M_1_6(3,4) - d(4)) / d(6));
    th(5,i:i+1) = real(ac5);
    th(5,i+2:i+3) = real(-ac5);
end

% theta 6
for i=1:2:8
    th1 = th(1,i);
    th5 = th(5,i);
    
    M_0_1 = eval(subs(M_joints(:,:,1), sym('th1'), th1));
    M_1_0 = eval(invTransf(M_0_1));
    M_1_6 = M_1_0 * M_0_6;
    M_6_1 = eval(invTransf(M_1_6));
    
    th(6, i:i+1) = atan2(-M_6_1(2,3)/sin(th5), M_6_1(1,3)/sin(th5));
end

% theta 3
for i=1:2:8
    th1 = th(1,i);
    th5 = th(5,i);
    th6 = th(6,i);
    
    M_0_1 = eval(subs(M_joints(:,:,1), sym('th1'), th1));
    M_4_5 = eval(subs(M_joints(:,:,5), sym('th5'), th5));
    M_5_6 = eval(subs(M_joints(:,:,6), sym('th6'), th6));
    
    M_1_0 = eval(invTransf(M_0_1));
    M_1_4 = M_1_0 * M_0_6 * inv(M_4_5 * M_5_6);
    p_1_3 = M_1_4 * [0;-d(4);0;1];
    p_1_3_norm = sqrt(p_1_3(1)^2 + p_1_3(2)^2 + p_1_3(3)^2);
    c3 = real((p_1_3_norm^2 - L(2)^2 - L(3)^2) / (2*L(2)*L(3)));
    s3 = real(sqrt(1-c3^2));
    
    th(3, i) = atan2(s3, c3);
    th(3, i+1) = atan2(-s3, c3);
end

% theta 2, theta 4
for i=1:8
    th1 = th(1,i);
    th5 = th(5,i);
    th6 = th(6,i);
    th3 = th(3,i);
    
    M_0_1 = eval(subs(M_joints(:,:,1), sym('th1'), th1));
    M_4_5 = eval(subs(M_joints(:,:,5), sym('th5'), th5));
    M_5_6 = eval(subs(M_joints(:,:,6), sym('th6'), th6));
    
    M_1_0 = eval(invTransf(M_0_1));
    M_1_4 = M_1_0 * M_0_6 * inv(M_4_5 * M_5_6);
    p_1_3 = M_1_4 * [0;-d(4);0;1];
    p_1_3_norm = sqrt(p_1_3(1)^2 + p_1_3(2)^2 + p_1_3(3)^2);
    
    % theta 2
    th(2, i) = -atan2(p_1_3(2), -p_1_3(1)) + asin(L(3)*sin(th3) / p_1_3_norm);
    
    % theta 4
    M_1_2 = eval(subs(M_joints(:,:,2), sym('th2'), th(2, i)));
    M_2_3 = eval(subs(M_joints(:,:,3), sym('th3'), th3));
    M_3_4 = invTransf(M_2_3) * invTransf(M_1_2) * M_1_4;
    th(4, i) = atan2(M_3_4(2,1), M_3_4(1,1));
    
    % Wrap angles to [-pi,pi] range
    th = wrapToPi(th);
end
end

