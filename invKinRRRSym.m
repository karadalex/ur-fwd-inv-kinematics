function angles = invKinRRRSym(M_joints)
    M_target = sym(eye(4));
    M_symbols = sym('T', [3 4]);
    M_target(1:3, 1:4) = M_symbols;
    angles = containers.Map;
    known = symvar(M_symbols);
    
    lhs = invTransf(M_joints(:,:,1)) * M_target;
    rhs = M_joints(:,:,2) * M_joints(:,:,3);
    disp(lhs-rhs);
    [angles, known] = solveIKequations(lhs, rhs, known, angles, 1);
    
    lhs = invTransf(M_joints(:,:,2)) * lhs;
    rhs = M_joints(:,:,3);
    disp(lhs-rhs);
    [angles, known] = solveIKequations(lhs, rhs, known, angles, 2);
end

