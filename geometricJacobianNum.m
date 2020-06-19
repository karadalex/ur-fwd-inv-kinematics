function Jv = geometricJacobianNum(th, M, M_joints, dof)
    JvSym = geometricJacobian(M, M_joints, dof);
    Jv = eval(subs(JvSym, sym('th', [1, 6]), th));
end

