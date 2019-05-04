function angles = invKinSym(M_joints)
    M_target = sym(eye(4));
    M_symbols = sym('M', [3 4]);
    M_target(1:3, 1:4) = M_symbols;
    angles = containers.Map;
    
    theta = sym('th', [1, 6]);
    known = symvar(M_symbols);
    
    % Solve for th1, th5, th6
    disp("Solving for th1, th5, th6 ...");
    invM_1 = invTransf(M_joints(:,:,1));
    invM_6 = invTransf(M_joints(:,:,6));
    lhs = invM_1 * M_target * invM_6;
    rhs = sym(eye(4));
    for j = 2:1:5
        rhs = rhs * M_joints(:,:,j);
    end
    [angles, known] = solveIKequations(lhs, rhs, known, angles, 3);
    
    % Solve for th2
    disp("Solving for th2 ...");
    lhs = M_target;
    for j = 1:1:2
        lhs = invTransf(M_joints(:,:,j)) * lhs;
    end
    for j = 6:-1:4
        lhs = lhs * invTransf(M_joints(:,:,j));
    end
    rhs = M_joints(:,:,3);
    [angles, known] = solveIKequations(lhs, rhs, known, angles, 1);
    
    % Solve for th4,th3
    disp("Solving for th4, th3 ...");
    lhs = M_target;
    for j = 6:-1:4
        lhs = lhs * invTransf(M_joints(:,:,j));
    end
    rhs = sym(eye(4));
    for j = 1:1:3
        rhs = rhs * M_joints(:,:,j);
    end
    [angles, known] = solveIKequations(lhs, rhs, known, angles, 2);    
    
    disp(known);
    fprintf('\n');
end

