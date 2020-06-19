function [M_total, M_joints] = fwdKinNum(L, d, a, th)
    [M_total, M_joints] = fwdKinSym(L, d, a);
    M_joints = subs(M_joints, sym('th', [1 6]), th);
    M_total = subs(M_total, sym('th', [1 6]), th);
end