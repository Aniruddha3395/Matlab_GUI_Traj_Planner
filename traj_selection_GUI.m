%% User-Interface for Selection of Points

global plot_traj_arr;
global xyz_bxbybz_groups;
global joint_angles_group;
global idx;
global group_idx;
global cl_last_data;
global cl_data;
global cl_failed_traj;
global move_rob;
global move_rob_all;
global msgbx;
global prev_traj_status;
global last_success_joint_angles;
global select_part;
global mold_v_transformed;
global mold_f;
global mold_n_transformed;
global rob_T_part;
global move_part_x;
global move_part_y;
global move_part_z;
global move_part_Rx;
global move_part_Ry;
global move_part_Rz;


% Define object for Data Cursor
dcm_obj = datacursormode(fig1);
set(dcm_obj,'SnapToDataVertex','off');
set(dcm_obj,'DisplayStyle','window');
strt_pts = [];
end_pts = [];
key=0;
flag = 0;
group_idx=[];
idx = 1;
xyz_bxbybz_groups = [];
joint_angles_group = [];
plot_traj_arr = [];
last_success_joint_angles = [];

% Title
titlebx = uicontrol(fig1,'Style','text');
titlebx.Position = [10 980 450 20];
titlebx.FontWeight = 'bold';
titlebx.FontSize = 12;
titlebx.String = '...GUI FOR TOOLPATH PLANNER FOR LAYUP ...';

% command messages
msgbx = uicontrol(fig1,'Style','text');
msgbx.Position = [550 960 800 30];
msgbx.FontWeight = 'bold';
msgbx.FontSize = 20;
msgbx.BackgroundColor = 'w';
msgbx.ForegroundColor = 'k';
prev_traj_status = [];
msgbx.String = strcat(prev_traj_status,'SELECT START POINT');

% Help Message
helpbx = uicontrol(fig1,'Style','text');
helpbx.Position = [10 10 790 20];
helpbx.FontWeight = 'bold';
helpbx.BackgroundColor = 'k';
helpbx.ForegroundColor = 'w';
helpbx.FontSize = 9;
helpbx.String = 'To select a point (start point or end point) on the Mold surface, use "data tips" to click point and press ENTER';

% DONE button
close_win = uicontrol(fig1,'Style','pushbutton');
close_win.Position = [1650 130 150 50];
close_win.String = 'CLOSE';
close_win.FontWeight = 'bold';
close_win.BackgroundColor = [0.8,0.9,1];
close_win.ForegroundColor = [0.1,0,0.3];
close_win.FontSize = 13;
close_win.Callback = @CloseFigWindow;

% View Complete Robot Simulation Button
move_rob_all = uicontrol(fig1,'Style','pushbutton');
move_rob_all.Position = [50 625 220 40];
move_rob_all.String = 'View Complete Motion';
move_rob_all.FontWeight = 'bold';
move_rob_all.BackgroundColor = [0.9,1,0.6];
move_rob_all.ForegroundColor = [0.1,0,0.3];
move_rob_all.FontSize = 12;
move_rob_all.Callback = @CompleteTrajViz;

% View Recent Trajectory Simulation Button
move_rob = uicontrol(fig1,'Style','pushbutton');
move_rob.Position = [50 680 220 40];
move_rob.String = 'View Last Motion';
move_rob.FontWeight = 'bold';
move_rob.BackgroundColor = [0.9,1,0.6];
move_rob.ForegroundColor = [0.1,0,0.3];
move_rob.FontSize = 12;
move_rob.Callback = @LastTrajViz;

% Stop Trajectory Simulation Button
stop_viz = uicontrol(fig1,'Style','pushbutton');
stop_viz.Position = [50 570 220 40];
stop_viz.String = 'Stop Motion';
stop_viz.FontWeight = 'bold';
stop_viz.BackgroundColor = [0.9,1,0.6];
stop_viz.ForegroundColor = [0.1,0,0.3];
stop_viz.FontSize = 12;
stop_viz.Callback = @KillViz;

% Clear All Data Button
cl_data = uicontrol(fig1,'Style','pushbutton');
cl_data.Position = [50 805 220 40];
cl_data.String = 'Clear All Data';
cl_data.FontWeight = 'bold';
cl_data.BackgroundColor = [1,0.5,0.5];
cl_data.ForegroundColor = [0.1,0,0.3];
cl_data.FontSize = 12;
cl_data.Callback = @ClearAllVars;

% Clear Last Trajectory Button
cl_last_data = uicontrol(fig1,'Style','pushbutton');
cl_last_data.Position = [50 860 220 40];
cl_last_data.String = 'Undo Trajectory';
cl_last_data.FontWeight = 'bold';
cl_last_data.BackgroundColor = [1,0.5,0.5];
cl_last_data.ForegroundColor = [0.1,0,0.3];
cl_last_data.FontSize = 12;
cl_last_data.Callback = @ClearLastTrajVars;

% Clear Failed Trajectory Button
cl_failed_traj = uicontrol(fig1,'Style','pushbutton');
cl_failed_traj.Position = [50 750 220 40];
cl_failed_traj.String = 'Clear Failed Trajectory';
cl_failed_traj.FontWeight = 'bold';
cl_failed_traj.BackgroundColor = [1,0.5,0.5];
cl_failed_traj.ForegroundColor = [0.1,0,0.3];
cl_failed_traj.FontSize = 12;
cl_failed_traj.Callback = @ClearFailedTraj;

% Write Data Button
write_file = uicontrol(fig1,'Style','pushbutton');
write_file.Position = [1650 190 150 50];
write_file.String = 'SAVE DATA';
write_file.FontWeight = 'bold';
write_file.BackgroundColor = [0.3,1,0.8];
write_file.ForegroundColor = [0.1,0,0.3];
write_file.FontSize = 13;
write_file.Callback = @WriteDataToFile;

% Select part Button
select_part = uicontrol(fig1,'Style','popupmenu');
select_part.Position = [1560 900 250 50];
select_part.String = store_file_str;
select_part.FontWeight = 'bold';
select_part.BackgroundColor = [1,1,0.9];
select_part.ForegroundColor = [0.1,0,0.3];
select_part.FontSize = 13;
select_part.Callback = @SelectPart;

% Tranform Part Button
% Info
infobx = uicontrol(fig1,'Style','text');
infobx.Position = [1620 680 200 20];
infobx.FontWeight = 'bold';
infobx.FontSize = 11;
infobx.String = 'Adjust Part Location';
x_leb = uicontrol(fig1,'Style','text');
x_leb.Position = [1640 650 20 20];
x_leb.FontWeight = 'bold';
x_leb.FontSize = 11;
x_leb.String = 'X';
y_leb = uicontrol(fig1,'Style','text');
y_leb.Position = [1640 620 20 20];
y_leb.FontWeight = 'bold';
y_leb.FontSize = 11;
y_leb.String = 'Y';
z_leb = uicontrol(fig1,'Style','text');
z_leb.Position = [1640 590 20 20];
z_leb.FontWeight = 'bold';
z_leb.FontSize = 11;
z_leb.String = 'Z';
rx_leb = uicontrol(fig1,'Style','text');
rx_leb.Position = [1640 560 25 20];
rx_leb.FontWeight = 'bold';
rx_leb.FontSize = 11;
rx_leb.String = 'Rz';
ry_leb = uicontrol(fig1,'Style','text');
ry_leb.Position = [1640 530 25 20];
ry_leb.FontWeight = 'bold';
ry_leb.FontSize = 11;
ry_leb.String = 'Ry';
rz_leb = uicontrol(fig1,'Style','text');
rz_leb.Position = [1640 500 25 20];
rz_leb.FontWeight = 'bold';
rz_leb.FontSize = 11;
rz_leb.String = 'Rx';
% x
move_part_x = uicontrol(fig1,'Style', 'slider');
move_part_x.Position = [1670 650 150 20];
move_part_x.String = 'Translate x: ';
move_part_x.FontWeight = 'bold';
move_part_x.BackgroundColor = [1,1,0.9];
move_part_x.ForegroundColor = [0.1,0,0.3];
move_part_x.FontSize = 13;
move_part_x.Min = -2;
move_part_x.Max = 2;
move_part_x.Value = rob_T_part(1,4)/1000;
move_part_x.SliderStep = [1/1000,1/1000];
move_part_x.Callback = @TransformMold_x;
% y
move_part_y = uicontrol(fig1,'Style', 'slider');
move_part_y.Position = [1670 620 150 20];
move_part_y.String = 'Translate y: ';
move_part_y.FontWeight = 'bold';
move_part_y.BackgroundColor = [1,1,0.9];
move_part_y.ForegroundColor = [0.1,0,0.3];
move_part_y.FontSize = 13;
move_part_y.Min = -2;
move_part_y.Max = 2;
move_part_y.Value = rob_T_part(2,4)/1000;
move_part_y.SliderStep = [1/1000,1/1000];
move_part_y.Callback = @TransformMold_y;
% z
move_part_z = uicontrol(fig1,'Style', 'slider');
move_part_z.Position = [1670 590 150 20];
move_part_z.String = 'Translate y: ';
move_part_z.FontWeight = 'bold';
move_part_z.BackgroundColor = [1,1,0.9];
move_part_z.ForegroundColor = [0.1,0,0.3];
move_part_z.FontSize = 13;
move_part_z.Min = -2;
move_part_z.Max = 2;
move_part_z.Value = rob_T_part(3,4)/1000;
move_part_z.SliderStep = [1/1000,1/1000];
move_part_z.Callback = @TransformMold_z;
% Rz
eul_an = rotm2eul(rob_T_part(1:3,1:3));
move_part_Rz = uicontrol(fig1,'Style', 'slider');
move_part_Rz.Position = [1670 560 150 20];
move_part_Rz.String = 'Rotate z: ';
move_part_Rz.FontWeight = 'bold';
move_part_Rz.BackgroundColor = [1,1,0.9];
move_part_Rz.ForegroundColor = [0.1,0,0.3];
move_part_Rz.FontSize = 13;
move_part_Rz.Min = -pi;
move_part_Rz.Max = pi;
move_part_Rz.Value = eul_an(1);
move_part_Rz.SliderStep = [pi/1000,pi/1000];
move_part_Rz.Callback = @TransformMold_Rz;
% Ry
move_part_Ry = uicontrol(fig1,'Style', 'slider');
move_part_Ry.Position = [1670 530 150 20];
move_part_Ry.String = 'Rotate y: ';
move_part_Ry.FontWeight = 'bold';
move_part_Ry.BackgroundColor = [1,1,0.9];
move_part_Ry.ForegroundColor = [0.1,0,0.3];
move_part_Ry.FontSize = 13;
move_part_Ry.Min = -pi;
move_part_Ry.Max = pi;
move_part_Ry.Value = eul_an(2);
move_part_Ry.SliderStep = [pi/1000,pi/1000];
move_part_Ry.Callback = @TransformMold_Ry;
% Rx
move_part_Rx = uicontrol(fig1,'Style', 'slider');
move_part_Rx.Position = [1670 500 150 20];
move_part_Rx.String = 'Rotate x: ';
move_part_Rx.FontWeight = 'bold';
move_part_Rx.BackgroundColor = [1,1,0.9];
move_part_Rx.ForegroundColor = [0.1,0,0.3];
move_part_Rx.FontSize = 13;
move_part_Rx.Min = -pi;
move_part_Rx.Max = pi;
move_part_Rx.Value = eul_an(3);
move_part_Rx.SliderStep = [pi/1000,pi/1000];
move_part_Rx.Callback = @TransformMold_Rx;

% Keep Selecting Start and End points alternatively.
hold on;
view([90,73]);
hold on;
while 1
    key=0;
    while key==0
        msgbx.String = strcat(prev_traj_status,'SELECT START POINT');
        try
            key = waitforbuttonpress;
        catch
            flag=1;
            break;
        end
    end
    if flag==1
        break;
    end
    c_info = getCursorInfo(dcm_obj);
    if ~isempty(c_info)
        strt_pt = c_info.Position;
    else
        continue;
    end
    strt_pts = [strt_pts;strt_pt];
    cl_last_data.Enable = 'off';
    cl_data.Enable = 'off';
    close_win.Enable = 'off';
    
    key=0;
    msgbx.String = 'SELECT END POINT';
    while key==0
        try
            key = waitforbuttonpress;
        catch
            flag=1;
            break;
        end
    end
    if flag==1
        break;
    end
    c_info = getCursorInfo(dcm_obj);
    if ~isempty(c_info)
        end_pt = c_info.Position;
    else
        continue;
    end
    end_pts = [end_pts;end_pt];
    
    cl_last_data.Enable = 'on';
    cl_data.Enable = 'on';
    close_win.Enable = 'on';
    
    % Call the curve_fit function
    [spline_pts,spline_normals,spline_faces] = curve_fit(strt_pt,end_pt,mold_v_transformed,mold_f,mold_n_transformed);
    
    % Appending Group Indices complete
    [bx,by,bz] = compute_TCP_new(spline_pts,spline_normals); %compute_TCP function
    xyz_bxbybz = horzcat( spline_pts,bx,by,bz );
    
    if ~isempty(xyz_bxbybz)
        xyz_bxbybz(:,1:3) = xyz_bxbybz(:,1:3)./1000;    %mm to m
        run compute_trajectory.m;
    end
end