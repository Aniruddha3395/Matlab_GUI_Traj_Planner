clc;
clear all;
close all;
warning off;

global robot1;
global lim_on_failure;

% xyz_bxbybz = dlmread('xyz_bxbybz_wrt_part.csv');
% group_idx = dlmread('group_idx');
xyz_bxbybz = dlmread('case_planner_pts.csv');
xyz_bxbybz(:,3) = xyz_bxbybz(:,3)+ 46.736;
group_idx = dlmread('case_planner_grps.csv');

%% Initialize robot and tool
% robot base
robot1_base = eye(4);

% Tool to Robot transformation
robot1.robot_ree_T_tee = eye(4);
robot1.robot_ree_T_tee(1:3,4) = [-0.0494; 0; 0.1335]; % For Roller
show_tool = true;

%% run optimization

% tolerence setting
tolerances(1) = 0.003;
tolerances(2) = 0.0524;
tolerances(3) = 0.0524;
tolerances(4) = 1;
lim_on_failure = 10;

%% visualize kuka
mold_base = strcat('CAD_stl/Composite_Mold.STL');

%tool info
ee_tool = 'roller';

%STLREAD is a function obtaiend from matlab exchange. Refer to the file for
%more details.
[mold_v, mold_f, mold_n, stltitle] = stlRead(mold_base);
delete(gca);
close all;

% transforming part w.r.t. robot base
[mold_v_transformed,mold_n_transformed] = robot_to_part(mold_v,mold_n,[],[]);

% Plotting the CAD part in Figure-1
fig1 = figure;
addToolbarExplorationButtons(fig1);
set(fig1,'units','normalized','outerpos',[0 0 1 1]);
show_origin();
p_mold = patch('Faces',mold_f,'Vertices',mold_v_transformed,'FaceVertexCData',...
    [0.8,0.8,0.8],'FaceColor',[0.3,0.3,0.3],'EdgeColor','none');
xlim([-400,1000]);
ylim([-800,800]);
zlim([-100,1500]);
xlabel('X-Axis');
ylabel('Y-Axis');
zlabel('Z-Axis');
daspect([1, 1, 1]);
view([35,15]);
rotate3d on;
hold on;

%% camera settings
hold on;
camlight(-360,0);
hold on;
camlight(0,0);
hold on;
camlight('right');
hold on;

%% Visualize robot with end effector

% NOTE:
%%%%%%%%%%         <<<<< KUKA visualizer >>>>>         %%%%%%%%%%

% NOTE: UNCOMMENT THIS ONLY WHEN YOU WANT TO VISUALIZE DATA FROM STL FILES
% [Link0_v, Link0_f, Link0_n, ~] = stlRead('Link0.STL');
% [Link1_v, Link1_f, Link1_n, ~] = stlRead('Link1.STL');
% [Link2_v, Link2_f, Link2_n, ~] = stlRead('Link2.STL');
% [Link3_v, Link3_f, Link3_n, ~] = stlRead('Link3.STL');
% [Link4_v, Link4_f, Link4_n, ~] = stlRead('Link4.STL');
% [Link5_v, Link5_f, Link5_n, ~] = stlRead('Link5.STL');
% [Link6_v, Link6_f, Link6_n, ~] = stlRead('Link6.STL');
% [Link7_v, Link7_f, Link7_n, ~] = stlRead('Link7.STL');
% [tool_v, tool_f, tool_n, ~] = stlRead('Concave_Dome.STL');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% getting data from .mat file is faster
load STL_DATA_mm.mat;
load(strcat('tool_',ee_tool,'_STL_data_mm.mat'));
home_pos = [0,-0.7283,0,-2.0944,0,1.4041,0];    %some home position
FK_T = get_iiwa_FK_all_joints_mex(home_pos,eye(4));
FK_T(1:3,4) = FK_T(1:3,4).*1000;
FK_T(5:7,4) = FK_T(5:7,4).*1000;
FK_T(9:11,4) = FK_T(9:11,4).*1000;
FK_T(13:15,4) = FK_T(13:15,4).*1000;
FK_T(17:19,4) = FK_T(17:19,4).*1000;
FK_T(21:23,4) = FK_T(21:23,4).*1000;
FK_T(25:27,4) = FK_T(25:27,4).*1000;
FK_T(29:31,4) = FK_T(29:31,4).*1000;
FK_T(33:35,4) = FK_T(33:35,4).*1000;

h0 = hgtransform;
p0 = patch('Faces',Link0_f,'Vertices',Link0_v,'FaceVertexCData',[0.8,0.8,0.8],...
    'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p0,'parent', h0);
h1 = hgtransform;
p1 = patch('Faces',Link1_f,'Vertices',Link1_v,'FaceVertexCData',[0.8,0.8,0.8],...
    'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p1,'parent', h1);
h2 = hgtransform;
p2 = patch('Faces',Link2_f,'Vertices',Link2_v,'FaceVertexCData',[0.8,0.8,0.8],...
    'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p2,'parent', h2);
h3 = hgtransform;
p3 = patch('Faces',Link3_f,'Vertices',Link3_v,'FaceVertexCData',[0.8,0.8,0.8],...
    'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p3,'parent', h3);
h4 = hgtransform;
p4 = patch('Faces',Link4_f,'Vertices',Link4_v,'FaceVertexCData',[0.8,0.8,0.8],...
    'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p4,'parent', h4);
h5 = hgtransform;
p5 = patch('Faces',Link5_f,'Vertices',Link5_v,'FaceVertexCData',[0.8,0.8,0.8],...
    'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p5,'parent', h5);
h6 = hgtransform;
p6 = patch('Faces',Link6_f,'Vertices',Link6_v,'FaceVertexCData',[0.8,0.8,0.8],...
    'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p6,'parent', h6);
h7 = hgtransform;
p7 = patch('Faces',Link7_f,'Vertices',Link7_v,'FaceVertexCData',[0.8,0.8,0.8],...
    'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p7,'parent', h7);

hold on;
set(h0,'matrix',FK_T(1:4,:));
set(h1,'matrix',FK_T(5:8,:));
set(h2,'matrix',FK_T(9:12,:));
set(h3,'matrix',FK_T(13:16,:));
set(h4,'matrix',FK_T(17:20,:));
set(h5,'matrix',FK_T(21:24,:));
set(h6,'matrix',FK_T(25:28,:));
set(h7,'matrix',FK_T(29:32,:));

if show_tool
    hold on;
    h_tool = hgtransform;
    p_tool = patch('Faces',tool_f,'Vertices',tool_v,'FaceVertexCData',...
        [0.8,0.8,0.8],'FaceColor',[0.1,0.1,0.1],'EdgeColor','none','FaceAlpha',1);
    set(p_tool,'parent', h_tool);
    set(h_tool, 'matrix', FK_T(33:36,:));
end


hold on;
[xyz,bx,by,bz] = robot_to_part(xyz_bxbybz(:,1:3),...
        xyz_bxbybz(:,4:6),...
        xyz_bxbybz(:,7:9),...
        xyz_bxbybz(:,10:12));

% scatter3(xyz(group_idx(1,1):group_idx(1,2),1),xyz(group_idx(1,1):group_idx(1,2),2),xyz(group_idx(1,1):group_idx(1,2),3),'.');
hold on;
% plot_tcp(bx,by,bz,xyz);
hold on;
view([90,73]);
%% OPTIMIZER

theta_th(1) = -168*pi/180; %true angle : 170
theta_th(2) = -118*pi/180; %true angle : 120
theta_th(3) = -168*pi/180; %true angle : 170
theta_th(4) = -118*pi/180; %true angle : 120
theta_th(5) = -168*pi/180; %true angle : 170
theta_th(6) = -118*pi/180; %true angle : 120
theta_th(7) = -173*pi/180; %true angle : 175
bounds_lim = 20;
compute_traj_attempt_lim = 5;

options = optimoptions('fmincon','Algorithm', 'interior-point');
options.MaxIterations = 2000;
options.MaxFunctionEvaluations = 1e10;
options.OptimalityTolerance = 1e-18;
options.StepTolerance = 1e-10;
options.ConstraintTolerance = 1e-200;
% options.Display = 'iter';
options.Display = 'none';
options.SpecifyObjectiveGradient = false;
options.ObjectiveLimit = 1e-20;
new_seed_strt = 5;

% joint_config = zeros(7,1);   %start joint config
joint_config = [0.31189,0.2209,-0.1785,-1.5357,0.0176,1.3463,0]';
traj_successful = false;
group_idx_success = [];
xyz_bxbybz_groups = [];
joint_angles_group = [];
idx = 1;
xyz = xyz./1000;
for i = 1:size(group_idx,1)
%     disp(i);
%     [xyz,bx,by,bz] = robot_to_part(xyz_bxbybz(group_idx(i,1):group_idx(i,2),1:3),...
%         xyz_bxbybz(group_idx(i,1):group_idx(i,2),4:6),...
%         xyz_bxbybz(group_idx(i,1):group_idx(i,2),7:9),...
%         xyz_bxbybz(group_idx(i,1):group_idx(i,2),10:12));
    xyz_bxbybz_T = [xyz(group_idx(i,1):group_idx(i,2),:),...
        bx(group_idx(i,1):group_idx(i,2),:),...
        by(group_idx(i,1):group_idx(i,2),:),...
        bz(group_idx(i,1):group_idx(i,2),:)];
%     size(xyz_bxbybz_T,1)
    for new_seed_strt = 1:compute_traj_attempt_lim
        theta_lb = theta_th';
        theta_ub = -theta_th';
        reach = 0;
        joint_angles = [];
        failed_idx = [];
        for  target_idx= 1:size(xyz_bxbybz_T,1)
            %compute IK for each point
            [joint_config,status] = ascent_IK( joint_config,xyz_bxbybz_T(target_idx,:),...
                tolerances,options, theta_lb, theta_ub );
            if status
%                 fprintf('%d : success..\n',target_idx);
                reach = reach + 1;
            else
%                 fprintf('%d : fail..\n',target_idx);
                failed_idx = [failed_idx;target_idx];
                if size(failed_idx,1)>lim_on_failure
                    break;
                end
            end
%             size(joint_config)
            joint_angles = [joint_angles; joint_config'];
            theta_lb = joint_config-bounds_lim*pi/180';
            theta_ub = joint_config+bounds_lim*pi/180';
            for theta_count = 1:size(theta_lb,1)
                % lower bound check
                if theta_lb(theta_count)<theta_th(theta_count)
                    theta_lb(theta_count) = theta_th(theta_count);
                end
                % upper bound check
                if theta_ub(theta_count)>-theta_th(theta_count)
                    theta_ub(theta_count) = -theta_th(theta_count);
                end
            end
        end
        
        if size(xyz_bxbybz_T,1)-reach<lim_on_failure
            %         fprintf('\n Solution verified \n');
            traj_successful = true;
            break;
        else
            %         fprintf('\n Solution does not exist...trying again \n');
            joint_config = -pi/2 + pi*rand(7,1);
        end
    end
    
    if traj_successful
        disp('success');
        %Append the Group Indices
        idx_start = idx;
        idx = idx + size(xyz_bxbybz_T,1)-1;
        idx_end = idx;
        range = [idx_start,idx_end];
        group_idx_success = [group_idx_success;range]; %Append the Group Indices
        xyz_bxbybz_groups = [xyz_bxbybz_groups; xyz_bxbybz_T]; %Append the grouped xyz_bxbybz
        joint_angles_group = [joint_angles_group;joint_angles];
        disp([range(2)-range(1)+1,size(joint_angles,1)]);
        idx = idx + 1;
        hold on;
            plot3(xyz_bxbybz_T(1:3,1),xyz_bxbybz_T(1:3,2),xyz_bxbybz_T(1:3,3),'g','LineWidth',5); %Plot the Points
        %     plot_traj_arr = [plot_traj_arr;plot_traj];
        %     prev_traj_status = 'LAST TRAJECTORY SUCCESSFUL...';
        %     last_success_joint_angles = joint_angles;
        pause(0.001);
        hold on;
        
    else
        disp('failure');
        hold on;
        %     joint_angles = last_success_joint_angles;
            plot3(xyz_bxbybz_T(1:3,1),xyz_bxbybz_T(1:3,2),xyz_bxbybz_T(1:3,3),'r','LineWidth',5); %Plot the failed traj's unsuccessful Points
        %     plot_failed_traj_arr = [plot_failed_traj_arr;plot_failed_traj];
        %     prev_traj_status = 'LAST TRAJECTORY FAILED...';
        %     offset_fail_counter = offset_fail_counter + 1;
        pause(0.001);
        hold on;
    end
    
end

dlmwrite('data_files/case_planner_joint_angles.csv',joint_angles_group);
cba = bxbybz_to_euler(xyz_bxbybz_groups(:,4:6),...
    xyz_bxbybz_groups(:,7:9),xyz_bxbybz_groups(:,10:12));
dlmwrite('data_files/case_planner_xyzcba.csv',[1000*xyz_bxbybz_groups(:,1:3),cba]);

