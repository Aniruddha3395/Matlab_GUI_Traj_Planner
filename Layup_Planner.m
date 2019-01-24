clear;
clc;
close all;
format short;

run_transform_data = true;

%% Define part and Plot it
% Define the name of the part stored in the working directory
part = 'CAD_stl/Composite_Mold.STL';
global roller_width resolution;

roller_width = 24;
resolution = 5;
% Note changing direction of Z axis in stl origin. make sure to change the
% dot product in filtering faces in curve_fit

%% Load Part and all the info
%STLREAD is a function obtaiend from matlab exchange. Refer to the file for
%more details.
[v, f, n, stltitle] = stlRead(part);
delete(gca);
close all;

% Choose what color the CAD part needs to be displayed.
col_matrix = [0.941176, 0.972549, 1];

% Plotting the CAD part in Figure-1
figure(1);
set(figure(1),'units','normalized','outerpos',[0 0 1 1.2]);
patch('Faces',f,'Vertices',v,'FaceVertexCData',col_matrix,'FaceColor',col_matrix);
% set(gca,'zdir','reverse')
xlabel('X-Axis');
ylabel('Y-Axis');
zlabel('Z-Axis');
daspect([1, 1, 1]);
hold on;

%%%%%%%%%%%%%%%running the GUI for trajectory selection
run traj_selection_GUI.m 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if ~isempty(xyz_cba)
    csvwrite('data_files/xyz_bxbybz_wrt_partFrame.csv',xyz_cba);
    csvwrite('data_files/Group_IDX.csv',grp);
    if run_transform_data
        run transform_data.m;
        run MAIN.m;
    end
else
    disp('NO POINTS SELECTED');
end
