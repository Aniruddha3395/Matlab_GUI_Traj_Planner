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

%%%% command messages %%%%
msgbx = uicontrol(fig1,'Style','text');
msgbx.Position = [550 980 800 20];
msgbx.FontWeight = 'bold';
msgbx.FontSize = 14;
prev_traj_status = [];
msgbx.String = strcat(prev_traj_status,'SELECT START POINT');

% Help Message
helpbx = uicontrol(fig1,'Style','text');
helpbx.Position = [10 10 750 20];
helpbx.FontWeight = 'bold';
helpbx.FontSize = 8;
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
move_rob_all.Position = [1580 805 220 40];
move_rob_all.String = 'View Complete Motion';
move_rob_all.FontWeight = 'bold';
move_rob_all.BackgroundColor = [0.9,1,0.6];
move_rob_all.ForegroundColor = [0.1,0,0.3];
move_rob_all.FontSize = 12;
move_rob_all.Callback = @CompleteTrajViz;

% View Recent Trajectory Simulation Button
move_rob = uicontrol(fig1,'Style','pushbutton');
move_rob.Position = [1580 860 220 40];
move_rob.String = 'View Last Motion';
move_rob.FontWeight = 'bold';
move_rob.BackgroundColor = [0.9,1,0.6];
move_rob.ForegroundColor = [0.1,0,0.3];
move_rob.FontSize = 12;
move_rob.Callback = @LastTrajViz;

% Stop Trajectory Simulation Button
stop_viz = uicontrol(fig1,'Style','pushbutton');
stop_viz.Position = [1580 750 220 40];
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

% Keep Selecting Start and End points alternatively.
hold on;
view([90,73]);
hold on;
while 1
    key=0;
    fprintf('Select Start Point\n')
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
    fprintf('Select End Point')
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