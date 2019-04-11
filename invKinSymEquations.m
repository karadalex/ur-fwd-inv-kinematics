function equations = invKinSymEquations(M_th, M_joints)
    lhs = sym(eye(4));
    lhs(1:3, 1:4) = sym('M', [3 4]);
    rhs = M_th;
    equations = [];
    eq_index = 1;
    
    theta = sym('th', [1, 6]);
    
    for i = 1:1:5
        invM_i = invTransf(M_joints(:,:,i));
        lhs = invM_i * lhs;
        rhs = invM_i * rhs;
        
        for j = 1:1:3
            for k = 1:1:4
                eq = expand(lhs(j,k) - rhs(j,k));
                equations = [equations, eq];
                fprintf('EQ %d:   %s \n', eq_index, eq);
                disp(symvar(eq));
                eq_index = eq_index + 1;
            end
        end
        
        fprintf('\n');
    end
    
end


