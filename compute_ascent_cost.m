function error = compute_ascent_cost( theta, point )

global robot1;

% represent the point on the part w.r.t. robot-base
pointi = point(1:3)';

% ROBOT end-effector
if strcmp(robot1.rob_type,'iiwa7')
    ee_base_all = get_iiwa7_FK_all_joints_mex( theta,eye(4) ); % eff transf
elseif strcmp(robot1.rob_type,'iiwa14')
    ee_base_all = get_iiwa14_FK_all_joints_mex( theta,eye(4) ); % eff transf
end
transf_mat = ee_base_all(33:36,:) * robot1.robot_ree_T_tee; % For attaching tool
tool_xyz = transf_mat(1:3,4);

% Error Function. Donot Change
err_orientation = 1 - ( point(7)*transf_mat(1,2) + point(8)*transf_mat(2,2) + point(9)*transf_mat(3,2));
err_pose = pointi-tool_xyz;
error = 0.5*(  0.25*err_orientation + err_pose(1)^2 + err_pose(2)^2 + err_pose(3)^2  );
%     error = 0.5*err_orientation + 0.5*sqrt(err_pose(1)^2 + err_pose(2)^2 + err_pose(3)^2) ;
%     error = max(abs(err_orientation), abs(sqrt(err_pose(1)^2 + err_pose(2)^2 + err_pose(3)^2)));
end
