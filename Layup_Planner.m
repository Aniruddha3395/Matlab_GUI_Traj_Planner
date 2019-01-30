clc;
clear all;
close all;
warning off;
format short;

set(0, 'DefaultFigureRenderer', 'opengl');

write_data = false;

%% Define part and Plot it
% Define the name of the part stored in the working directory
mold_base = 'CAD_stl/Composite_Mold.STL';
global roller_width resolution;
global h0 h1 h2 h3 h4 h5 h6 h7 h_tool;
global take_video show_tool;

roller_width = 24;
resolution = 3;
take_video = false;

% Note changing direction of Z axis in stl origin. make sure to change the
% dot product in filtering faces in curve_fit

%% Load Part and all the info
%STLREAD is a function obtaiend from matlab exchange. Refer to the file for
%more details.
[mold_v, mold_f, mold_n, stltitle] = stlRead(mold_base);
delete(gca);
close all;

[mold_v_transformed,mold_n_transformed] = robot_to_part(mold_v,mold_n,[],[]);

% Choose what color the CAD part needs to be displayed.
col_matrix = [0.941176, 0.972549, 1];

% Plotting the CAD part in Figure-1
fig1 = figure;
set(fig1,'units','normalized','outerpos',[0 0 1 1.2]);
show_origin();
patch('Faces',mold_f,'Vertices',mold_v_transformed,'FaceVertexCData',...
    [0.8,0.8,0.8],'FaceColor',[0.3,0.3,0.3],'EdgeColor','none');
% set(gca,'zdir','reverse')
xlim([-300,1000]);
ylim([-800,800]);
zlim([-100,1500]);
xlabel('X-Axis');
ylabel('Y-Axis');
zlabel('Z-Axis');
daspect([1, 1, 1]);
view([35,15]);
hold on;

%%%%%% camera settings %%%%%%
hold on;
camlight(-360,0);
hold on;
camlight(0,0);
hold on;
%     camlight('left');
hold on;
camlight('right');
hold on;
%     camlight(0,0);
%     hold on;
%     camlight(0,0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% show robot

show_tool = true;
% getting data from .mat file is faster
load STL_COARSE_DATA_mm.mat;
% load tool_roller_STL_data_mm.mat;
load tool_roller_STL_data_mm.mat;

FK_T = get_iiwa_FK_all_joints_mex(zeros(7,1),eye(4));
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
p0 = patch('Faces',Link0_f,'Vertices',Link0_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p0,'parent', h0);
h1 = hgtransform;
p1 = patch('Faces',Link1_f,'Vertices',Link1_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p1,'parent', h1);
h2 = hgtransform;
p2 = patch('Faces',Link2_f,'Vertices',Link2_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p2,'parent', h2);
h3 = hgtransform;
p3 = patch('Faces',Link3_f,'Vertices',Link3_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p3,'parent', h3);
h4 = hgtransform;
p4 = patch('Faces',Link4_f,'Vertices',Link4_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p4,'parent', h4);
h5 = hgtransform;
p5 = patch('Faces',Link5_f,'Vertices',Link5_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p5,'parent', h5);
h6 = hgtransform;
p6 = patch('Faces',Link6_f,'Vertices',Link6_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
set(p6,'parent', h6);
h7 = hgtransform;
p7 = patch('Faces',Link7_f,'Vertices',Link7_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.9,0.5,0.1],'EdgeColor','none','FaceAlpha',1);
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
    p_tool = patch('Faces',tool_f,'Vertices',tool_v,'FaceVertexCData',[0.8,0.8,0.8],'FaceColor',[0.1,0.1,0.1],'EdgeColor','none','FaceAlpha',1);
    set(p_tool,'parent', h_tool);
    set(h_tool, 'matrix', FK_T(33:36,:));
end

%%%%%%%%%%%%%%%running the GUI for trajectory selection
run traj_selection_GUI_test.m;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% write data to file
if write_data
    dlmwrite('data_files/group_Idx.csv',group_idx);
    dlmwrite('data_files/xyz_bxbybz.csv',xyz_bxbybz_groups);
    dlmwrite('data_files/joint_angles.csv',joint_angles_group);
end





