function [th1,th2,th3,th4,th5,th6] = urIKSym(L, d, a, M_joints)
    M_0_6 = sym(eye(4));
    M_symbols = sym('M', [3 4]);
    M_0_6(1:3, 1:4) = M_symbols;
    th = sym('th', [1, 6]);
    
    % Calculate th1
    M_0_5 = M_0_6 * invTransf(M_joints(:,:,6));
    p_0_5 = M_0_5(1:3, 4);
    p_0_5x = p_0_5(1);
    p_0_5y = p_0_5(2);
    term1 = atan2(p_0_5y, p_0_5x);
    term2 = acos(d(4)/sqrt(p_0_5x^2+p_0_5y^2))+pi/2;
    th1(1) = term1 + term2;
    th1(2) = term1 - term2;
    
    % Calculate th5
    p_0_6 = M_0_6(1:3, 4);
    p_0_6x = p_0_6(1);
    p_0_6y = p_0_6(2);
    th5(1) = acos((p_0_6x*sin(th(1)) - p_0_6y*cos(th(1)) - d(4))/d(6));
    th5(2) = -th5(1);
    
    % Calculate th6
    M_6_0 = invTransf(M_0_6);
    i_6_0 = M_0_6(1:3, 1);
    i_6_0x = i_6_0(1);
    i_6_0y = i_6_0(2);
    j_6_0 = M_0_6(1:3, 2);
    j_6_0x = j_6_0(1);
    j_6_0y = j_6_0(2);
    frac1 = (-i_6_0y*sin(th(1)) + j_6_0y*cos(th(1)))/sin(th(5));
    frac2 = (i_6_0x*sin(th(1)) - j_6_0x*cos(th(1)))/sin(th(5));
    th6 = atan2(frac1, frac2);
    
    % Calculate th3
    M_1_4 = invTransf(M_joints(:,:,1)) * M_0_5 * invTransf(M_joints(:,:,4));
    p_1_4 = M_1_4(1:3, 4);
    p_1_4x = p_1_4(1);
    p_1_4z = p_1_4(3);
    th3(1) = acos((p_1_4x^2 + p_1_4z^2 - L(2)^2-L(3)^2)/(2*L(2)*L(3)));
    th3(2) = -th3(1);
    
    % Calculate th2
    p_1_4xz = sqrt(p_1_4x^2 + p_1_4z^2);
    th2 = atan2(-p_1_4z, p_1_4x) - asin(-L(3)*sin(th3)/p_1_4xz);
    
    % Calculate th4
    M_3_4 = invTransf(M_joints(:,:,3)) * invTransf(M_joints(:,:,2)) * M_1_4;
    i_3_4 = M_3_4(1:3, 1);
    i_3_4x = i_3_4(1);
    i_3_4y = i_3_4(2);
    th4 = atan2(i_3_4y, i_3_4x);
end

