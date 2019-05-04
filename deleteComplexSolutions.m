function realSol = deleteComplexSolutions(complexSol)
    realSol = [];
    for i = 1:1:length(complexSol)
        sol = complexSol(i);
        if abs(imag(sol)) < 0.5*10^(-7)
            realSol = [realSol, real(sol)];
        end
    end
end

