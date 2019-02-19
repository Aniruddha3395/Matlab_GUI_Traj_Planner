function [joints,status] = ascent_IK( init_guess,point,tolerance,options,theta_lb,theta_ub )

global robot1;

% Attempt IK with init_guess.
[joints] = fmincon(@(theta)compute_ascent_cost(theta,point),init_guess,...
    [],[],[],[],theta_lb,theta_ub,@(theta)ascent_constr(theta,point),...
    options);

% Verify the Solution
if strcmp(robot1.rob_type,'iiwa7')
    ee_base_all = get_iiwa7_FK_all_joints_mex( joints,eye(4) );
elseif strcmp(robot1.rob_type,'iiwa14')
    ee_base_all = get_iiwa14_FK_all_joints_mex( joints,eye(4) );
end
ee_base = ee_base_all(33:36,:) * robot1.robot_ree_T_tee;

% Error Position
err_xyz = norm(point(1:3)'-ee_base(1:3,4));

% Error Orientation
%         err_bx = real(acos( point(4)*ee_base(1,1) + point(5)*ee_base(2,1) + point(6)*ee_base(3,1) ));
err_by = real(acos( point(7)*ee_base(1,2) + point(8)*ee_base(2,2) + point(9)*ee_base(3,2) ));
err_bz = real(acos( point(10)*ee_base(1,3) + point(11)*ee_base(2,3) + point(12)*ee_base(3,3) ));
if err_xyz > tolerance(1) || err_bz > tolerance(4) || err_by > tolerance(3)
    status = false;
else
    status = true;
end
end