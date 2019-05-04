function [angles, known] = solveIKequations(lhs, rhs, known, angles, epochs)
    restart = false;
    for epoch = 1:1:epochs
        for j = 1:1:3
            for k = 1:1:4
                eq = expand(lhs(j,k) - rhs(j,k));
                eqVars = symvar(eq);
                unknown = setdiff(eqVars, known);
                disp(eq);
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
            end
            if restart
                restart = false;
                break;
            end
        end
    end
end

