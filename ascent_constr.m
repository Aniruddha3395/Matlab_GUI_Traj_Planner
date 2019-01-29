function [c,ceq] = ascent_constr( theta,point )
    global robot1;
    
    c = [];
    ceq = [];
    
% ROBOT end-effector
    FK_all = get_iiwa_FK_all_joints_mex( theta,eye(4) );
    ee_base = FK_all(33:36,:);
    transf_mat = ee_base * robot1.robot_ree_T_tee; % For attaching tool

% Error Function. Donot Change
    error = transf_mat(1,3)*point(10) + transf_mat(2,3)*point(11) + transf_mat(3,3)*point(12);
    c = [ c; -error + cos(20*pi/180) ];     %theta is the bound for the wiggling of tool
%     % cos(0) means tool matched the desired frame in all bx, by and bz.
        
end