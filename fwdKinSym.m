function [M_total, M_joints] = fwdKinSym(L, d, a)
    theta = sym('th', [1, 6]);

    % warning('off', 'all')
    M_total = eye(4);
    for i = 1:1:6
      disp(i)
      M_joints(:,:,i) = [
        [cos(theta(i)), -sin(theta(i)), 0, L(i)],
        [sin(theta(i)) * cos(a(i)), cos(theta(i)) * cos(a(i)), -sin(a(i)), -sin(a(i)) * d(i)],
        [sin(theta(i)) * sin(a(i)), cos(theta(i)) * sin(a(i)), cos(a(i)), cos(a(i)) * d(i)],
        [0, 0, 0, 1]
      ];
      disp(M_joints(:,:,i))
      M_total = M_total * M_joints(:,:,i);
    end
end

