% clc;
% clear;
% close all;
% warning 'off';

visualize_rob = true;
write_data = true;
show_joint_angle_change = true;

global robot1;
joint_vel = deg2rad([ 98; 98; 100; 130; 140; 180; 180 ]);
lim_on_failure = 5;

%%%%%%%%%%%%%%%%%%%DELETE THIS...ADDING EXTRA DATA %%%%%%%%%%%%%%%%%%%%
% abc = [179.09,7.11,-138.40]*(pi/180);
% [bx,by,bz] = euler_to_bxbybz(abc);
% target_pts = [544.61,137.02,312.33,bx,by,bz];
% dlmwrite('data_files/xyz_frames_test.csv',target_pts);
% dlmwrite('data_files/Group_IDX_test.csv',1);
% 
% fprintf('initial pose =\n');
% fprintf('xyz =');disp(target_pts(:,1:3));
% fprintf('bx =');disp(target_pts(:,4:6));
% fprintf('by =');disp(target_pts(:,7:9));
% fprintf('bz =');disp(target_pts(:,10:12));

%%%%%%%% robot base %%%%%%%%%
robot1_base = eye(4);

%%%%%%%% Tool to Robot transformation %%%%%%%%%%
robot1.robot_ree_T_tee = eye(4);
robot1.robot_ree_T_tee(1:3,4) = [-0.0494; 0; 0.1335]; % For Roller
% robot1.robot_ree_T_tee(1:3,4) = [0; 0; 0.0456]; % For Roller

%% %%%%%%%% CURVE DATA %%%%%%%%%%%%%
curve_file_name  = 'data_files/xyz_bxbybz_wrt_robotFrame.csv';            % n x 12 [x y z bx by bz]
xyz_bxbybz = dlmread(curve_file_name);          %%%%%% NOTE: INPUT FROM THE GUI in mm
xyz_bxbybz(:,1:3) = xyz_bxbybz(:,1:3)./1000;    %mm to m
grp_idx = csvread('data_files/Group_IDX.csv');             %%%%%% NOTE: check if new gui writes this file
no_pts = size(xyz_bxbybz,1);                % if not add

%%%%%%%% END CURVE DATA %%%%%%%%%%%%%
% fprintf('Number of Points considered for evaluation:  \n%d\n ',no_pts);

%%%%%%%%%%% tolerence setting %%%%%%%%%%%
tolerances(1) = 0.003;
tolerances(2) = 0.0524;
tolerances(3) = 0.0524;
tolerances(4) = 1;

%% run optimization
run optimization_IK.m

%% write data to file
if write_data
    dlmwrite('data_files/joint_angles.csv',joint_angles);
end

%% plot joint angles change
if show_joint_angle_change
    disp_joint_angle_change(joint_angles);
end

%% Visualization 
if visualize_rob
    visualize_kuka_mm(joint_angles);
end


% fprintf('Success Cases:  %d\n',success_cases);
% p = get_iiwa_FK_mex(joint_config,eye(4));
% p = p*robot1.robot_ree_T_tee;
% r = rotm2eul(p(1:3,1:3));
% [bx,by,bz] =euler_to_bxbybz(r); 
% fprintf('final pose =\n');
% fprintf('xyz =');disp([p(1,4),p(2,4),p(3,4)].*1000);
% fprintf('bx =');disp(bx);
% fprintf('by =');disp(by);
% fprintf('bz =');disp(bz);