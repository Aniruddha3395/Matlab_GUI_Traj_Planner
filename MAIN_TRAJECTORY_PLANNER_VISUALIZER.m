%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                        %
%                                                                        %
%                                                                        %
% << Trajectory Planner GUI using Optimization based approach >>         %
% NOTE 1: When running the code from different path...just               %
% change the folder                                                      %
% NOTE 2: Whenever new tool is added, add the tool data at               %
% the end of this script                                                 %
% NOTE 3: To change the initial robot base frame to part frame           %
% transformation - Change the value of "rob_T_part" in 'robot_to_part.m' %
% (or) change the correspondance points in 'robot_to_part.m'             %
% NOTE 4: IF any changes are done in MEX files, then set "edited_MEX"    %
% to be true                                                             %
%                                                                        %
%                                                                        %
%                                                                        %
%                                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear all;
close all;
dbstop if error;
warning off;
format short;

%% Define global variables

set(0, 'DefaultFigureRenderer', 'opengl');
global roller_width;
global resolution;
global h0 h1 h2 h3 h4 h5 h6 h7 h_tool;
global p0 p1 p2 p3 p4 p5 p6 p7;
global take_video;
global show_tool;
global home_pos;
global kill_sig;
global store_file_str;
global store_file_tool_str;
global mold_v_transformed;
global mold_f;
global mold_v;
global mold_n;
global mold_n_transformed;
global p_mold;
global p_tool;
global lim_on_failure;
global FK_T;
global robot1;
global tool1;
global use_cpp_IK_solver;

%<<<<<<<<<<<<<<<<<<<<<<<.>>>>>>>>>>>>>>>>>>>>>>>>>%
use_cpp_IK_solver = true;
%<<<<<<<<<<<<<<<<<<<<<<<.>>>>>>>>>>>>>>>>>>>>>>>>>%

edited_MEX = false;

%% adding woking directory and all dependancies
[working_dir,~,~] = fileparts(mfilename('fullpath'));
addpath(genpath(working_dir),'-end');

%% Initializing MEX files for KUKA
if ~ispc
    cd MEX_files/;
    if exist('bxbybz_to_euler_mex.mexa64','file')==0 || edited_MEX
        run_MEX(use_cpp_IK_solver);
    end
    if exist('apply_transformation_mex.mexa64','file')==0 || edited_MEX
        run_MEX(use_cpp_IK_solver);
    end
    if exist('ascent_IK_mex.mexa64','file')==0 || edited_MEX
        run_MEX(use_cpp_IK_solver);
    end
    if exist('get_iiwa7_FK_all_joints_mex.mexa64','file')==0 || edited_MEX
        run_MEX(use_cpp_IK_solver);
    end
    if exist('get_iiwa7_FK_mex.mexa64','file')==0 || edited_MEX
        run_MEX(use_cpp_IK_solver);
    end
    if exist('get_iiwa14_FK_all_joints_mex.mexa64','file')==0 || edited_MEX
        run_MEX(use_cpp_IK_solver);
    end
    cd ..;
else
    cd MEX_files/;
    if exist('bxbybz_to_euler_mex.mexw64','file')==0 || edited_MEX
        run_MEX(use_cpp_IK_solver);
    end
    if exist('apply_transformation_mex.mexw64','file')==0 || edited_MEX
        run_MEX(use_cpp_IK_solver);
    end
    if exist('get_iiwa7_FK_all_joints_mex.mexw64','file')==0 || edited_MEX
        run_MEX(use_cpp_IK_solver);
    end
    if exist('get_iiwa7_FK_mex.mexw64','file')==0 || edited_MEX
        run_MEX(use_cpp_IK_solver);
    end
    if exist('get_iiwa14_FK_all_joints_mex.mexw64','file')==0 || edited_MEX
        run_MEX(use_cpp_IK_solver);
    end
    cd ..; 
end

roller_width = 24;
resolution = 3;
take_video = false;
show_tool = true;
kill_sig = false;
lim_on_failure = 1;


%% Load Mold, Tool and relative information
cd CAD_stl/Molds/;
dir_list_struct = dir;
dir_list_cell = struct2cell(dir_list_struct);
store_file_str ={};
pattern = {'.STL','.stl'};
for i = 1:size(dir_list_cell,2)
    if contains(dir_list_cell{1,i},pattern)
        store_file_str{end+1} = dir_list_cell{1,i};
    end
end
cd ../Tools/;
dir_list_struct = dir;
dir_list_cell = struct2cell(dir_list_struct);
store_file_tool_str ={'NO_TOOL'};
pattern = {'.STL','.stl'};
for i = 1:size(dir_list_cell,2)
    if contains(dir_list_cell{1,i},pattern)
        store_file_tool_str{end+1} = dir_list_cell{1,i};
    end
end
cd ../..;
mold_base = strcat('CAD_stl/Molds/',store_file_str{1});

if ~strcmp(store_file_tool_str{1},'NO_TOOL')
    ee_tool = strcat('CAD_stl/Tools/',store_file_tool_str{1});
else
    show_tool = false;
end

%STLREAD is a function obtaiend from matlab exchange. Refer to the file for
%more details.
[mold_v, mold_f, mold_n, ~] = stlRead(mold_base);

if ~strcmp(store_file_tool_str{1},'NO_TOOL')
    [tool_v, tool_f, tool_n, ~] = stlRead(ee_tool);
    delete(gca);
end

close all;

% transforming part w.r.t. robot base
[mold_v_transformed,mold_n_transformed] = robot_to_part(mold_v,mold_n,[],[]);

% Plotting the CAD part in Figure-1
fig1 = figure();
set(fig1,'units','normalized','outerpos',[0 0 1 1]);
% axis off;
axis equal;
addToolbarExplorationButtons(fig1);
pause(0.1);
% set(gcf,'color','w');
set(gca,'color',[0.9400 0.9400 0.9400]);

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
hold on;

%% camera settings
hold on;
camlight(-360,0);
hold on;
camlight(0,0);
hold on;
camlight('right');
hold on;
material metal;

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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% getting data from .mat file is faster
robot1.rob_type = 'iiwa7';
if strcmp(robot1.rob_type,'iiwa7')
    load STL_iiwa7_DATA_mm.mat;
else
    load STL_iiwa14_DATA_mm.mat;
end

home_pos = [-0.1321;0.1415;0.0895;-1.5916;-0.0033;1.4041;-0.0312];    %some home position
if strcmp(robot1.rob_type,'iiwa7')
    FK_T = get_iiwa7_FK_all_joints_mex(home_pos,eye(4));
elseif strcmp(robot1.rob_type,'iiwa14')
    FK_T = get_iiwa14_FK_all_joints_mex(home_pos,eye(4));
end
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
else
    hold on;
    h_tool = hgtransform;
    p_tool = patch();
    set(p_tool,'parent', h_tool);
    set(h_tool, 'matrix', FK_T(33:36,:));
end

%% Initialize tcp and robot base
% robot base
robot1_base = eye(4);

%% Initialize tool data

% Tool to Robot End Effector transformation
tool1 = containers.Map('KeyType','char','ValueType','any');
tool1('NO_TOOL') = eye(4);
%tool data for roller
t = eye(4);
t(1:3,4) = [-0.0494; 0; 0.1335];
tool1('Roller.STL') = t;
%tool data for roller2
t = eye(4);
t(1:3,4) = [-0.0494; 0; 0.152];
t(1:3,1:3) = eul2rotm([0.1,0,0]);
tool1('Roller2.STL') = t;
%tool data for sander
t = eye(4);
t(1:3,4) = [0; 0; 0.1383];
tool1('Sander.STL') = t;
%tool data for probe
t = eye(4);
t(1:3,4) = [0; 0; 0.0968];
tool1('Probe.STL') = t;
%tool data for calibration tool
t = eye(4);
t(1:3,4) = [0; 0; 0.0478];
tool1('Calib_Tool.STL') = t;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                       %
% add the new tool data as following -                                  %
% tool1('TOOL_NAME') = transformation matrix;                           %
%                                                                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ensuring new tools are added with respective TCP
for file_idx = 1:size(store_file_tool_str,2)
    try
        tool1(store_file_tool_str{file_idx});
    catch
        disp('TCP is not available for the following tool - ');
        disp(store_file_tool_str{file_idx});
        return;
    end
end

robot1.robot_ree_T_tee = tool1('NO_TOOL');

%% Run the GUI for trajectory selection
run traj_selection_GUI.m;

