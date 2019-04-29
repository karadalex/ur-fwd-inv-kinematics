function angles = invKinSym(M_th, M_joints)
    M_target = sym(eye(4));
    M_symbols = sym('M', [3 4]);
    M_target(1:3, 1:4) = M_symbols;
    angles = containers.Map;
    eq_index = 1;
    
    theta = sym('th', [1, 6]);
    known = symvar(M_symbols);
    
    % Get equations for th1, th5, th6
    disp("Solving for th1, th5, th6 ...");
    restart = false;
    for epochs = 1:1:3
        invM_1 = invTransf(M_joints(:,:,1));
        invM_6 = invTransf(M_joints(:,:,6));
        lhs = invM_1 * M_target * invM_6;
        rhs = sym(eye(4));
        for j = 2:1:5
            rhs = rhs * M_joints(:,:,j);
        end
        for j = 1:1:3
            for k = 1:1:4
                eq = expand(lhs(j,k) - rhs(j,k));
                eqVars = symvar(eq);
                unknown = setdiff(eqVars, known);
                fprintf('EQ %d:   %s \n', eq_index, eq);
                disp(unknown);
                if length(unknown) == 1
                    eq = rewrite(eq, 'tan');
                    eqSol = solve(eq, unknown);
                    eqSol = simplify(eqSol);
                    angles(char(unknown)) = eqSol;
                    disp(eqSol);
                    known = [known, unknown];
                    restart = true;
                    break;
                end
                eq_index = eq_index + 1;
            end
            if restart
                restart = false;
                break;
            end
        end
    end
    
    % Get equations for th4
    disp("Solving for th4 ...");
    lhs = M_target;
    for j = 1:1:2
        lhs = invTransf(M_joints(:,:,j)) * lhs;
    end
    for j = 4:1:6
        lhs = lhs * invTransf(M_joints(:,:,j));
    end
    rhs = M_joints(:,:,3);
    for j = 1:1:3
        for k = 1:1:4
            eq = simplify(lhs(j,k) - rhs(j,k));
            eqVars = symvar(eq);
            unknown = setdiff(eqVars, known);
            fprintf('EQ %d:   %s \n', eq_index, eq);
            disp(unknown);
            if length(unknown) == 1
                eq = rewrite(eq, 'tan');
                %eq = simplify(eq);
                eqSol = solve(eq, unknown);
                %eqSol = simplify(eqSol);
                angles(char(unknown)) = eqSol;
                disp(eqSol);
                known = [known, unknown];
            end
            eq_index = eq_index + 1;
        end
    end
    
    % Get equations for th2
    disp("Solving for th2 ...");
    lhs = M_target;
    for j = 6:-1:4
        lhs = lhs * invTransf(M_joints(:,:,j));
    end
    rhs = sym(eye(4));
    for j = 1:1:3
        rhs = rhs * M_joints(:,:,j);
    end
    restart = false;
    for j = 1:1:3
        for k = 1:1:4
            eq = simplify(lhs(j,k) - rhs(j,k));
            eqVars = symvar(eq);
            unknown = setdiff(eqVars, known);
            fprintf('EQ %d:   %s \n', eq_index, eq);
            disp(unknown);
            if length(unknown) == 1
                eq = rewrite(eq, 'tan');
                eqSol = solve(eq, unknown);
                eqSol = simplify(eqSol);
                angles(char(unknown)) = eqSol;
                disp(eqSol);
                known = [known, unknown];
                restart = true;
                break;
            end
            eq_index = eq_index + 1;
        end
        if restart
            restart = false;
            break;
        end
    end
    
    % Get equations for th3
    disp("Solving for th3 ...");
    lhs = M_target;
    for j = 1:1:2
        lhs = invTransf(M_joints(:,:,j)) * lhs;
    end
    rhs = sym(eye(4));
    for j = 3:1:6
        rhs = rhs * M_joints(:,:,j);
    end
    for j = 1:1:3
        for k = 1:1:4
            eq = simplify(lhs(j,k) - rhs(j,k));
            eqVars = symvar(eq);
            unknown = setdiff(eqVars, known);
            fprintf('EQ %d:   %s \n', eq_index, eq);
            disp(unknown);
            if length(unknown) == 1
                eq = rewrite(eq, 'tan');
                %eq = simplify(eq);
                eqSol = solve(eq, unknown);
                %eqSol = simplify(eqSol);
                angles(char(unknown)) = eqSol;
                disp(eqSol);
                known = [known, unknown];
            end
            eq_index = eq_index + 1;
        end
    end
    
    disp(known);
    fprintf('\n');
end

