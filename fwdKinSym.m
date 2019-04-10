% warning('off', 'all')

function [M_total, M_joints] = fwdKinSym(L, d, a)
    % Create/convert to symbolic variables
    L = sym(L);
    d = sym(d);
    a = sym(a);
    theta = sym('th', [1, 6]);
    
    % Initialize transformation matrices
    M_joints = sym('M', [4 4 6]);
    M_total = eye(4);
    M_total = sym(M_total);
    
    for i = 1:1:6
      disp(i)
      M_joints(:,:,i) = [
        [cos(theta(i)), -sin(theta(i)), 0, L(i)],
        [sin(theta(i)) * cos(a(i)), cos(theta(i)) * cos(a(i)), -sin(a(i)), -sin(a(i)) * d(i)],
        [sin(theta(i)) * sin(a(i)), cos(theta(i)) * sin(a(i)), cos(a(i)), cos(a(i)) * d(i)],
        [0, 0, 0, 1]
      ];
      disp(M_joints(:,:,i))
      M_total = simplify(M_total) * M_joints(:,:,i);
    end
    M_total = simplify(M_total);
end

