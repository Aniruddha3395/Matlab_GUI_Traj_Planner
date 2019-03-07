function M = iiwa_manipulability_from_joints(thetas)
    M = [];
    for i = 1:size(thetas,1)
        J = iiwa_analytical_jacobian(thetas(i,:));
        s = svd(J);
        M = [M,prod(s)];
    end
end

