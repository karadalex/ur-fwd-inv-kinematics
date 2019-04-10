function equations = invKinSym(M_th, M_joints)
    lhs = sym('M', [4 4]);
    rhs = M_th;
    equations = [];
    
    theta = sym('th', [1, 6]);
    theta2 = sym('th_2', [1, 6]); % substitution: theta2 = theta/2
    t = sym('t', [1, 6]); % substitution: t = tan(theta2)
    
    for i = 1:1:5
        invM_i = invTransf(M_joints(:,:,i));
        lhs = lhs * invM_i;
        rhs = expand(rhs) * invM_i;
        
        for j =1:1:3
            for k = 1:1:4
                eq = lhs(j,k) - rhs(j,k);
                [m,n] = size(symvar(eq));
                eq = rewrite(eq, 'tan');
                eq = subs(eq, theta, 2*theta2);
                eq = subs(eq, tan(theta2), t);
                equations = [equations, eq];
            end
        end
    end
    
end


