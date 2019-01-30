%% User-Interface for Selection of Points

global plot_traj_arr;

% Define object for Data Cursor
dcm_obj = datacursormode(fig1);
set(dcm_obj,'SnapToDataVertex','off');
set(dcm_obj,'DisplayStyle','window');
strt_pts = [];
end_pts = [];
key=0;
flag = 0;
group_idx=[];
normals = [];
idx = 1;
faces=[];
tool_num = [];
xyz_bxbybz_groups = [];
joint_angles_group = [];
xyz=[];
cba=[];
plot_traj_arr = [];

%%%%% DONE button %%%%%
close_win = uicontrol(fig1,'Style','pushbutton');
close_win.Position = [1500 40 150 50];
close_win.String = 'CLOSE';
close_win.FontWeight = 'bold';
close_win.BackgroundColor = [0.8,0.9,1];
close_win.ForegroundColor = [0.1,0,0.3];
close_win.FontSize = 12;
close_win.Callback = @CloseFigWindow;

%%%%% View Complete Robot Simulation Button %%%%%
move_rob_all = uicontrol(fig1,'Style','pushbutton');
move_rob_all.Position = [1500 100 150 50];
move_rob_all.String = 'View Complete Motion';
move_rob_all.FontWeight = 'bold';
move_rob_all.BackgroundColor = [0.8,0.9,1];
move_rob_all.ForegroundColor = [0.1,0,0.3];
move_rob_all.FontSize = 12;
move_rob_all.Callback = @visualize_kuka_mm_test;

%%%%% View Recent Trajectory Simulation Button %%%%%
move_rob = uicontrol(fig1,'Style','pushbutton');
move_rob.Position = [1500 160 150 50];
move_rob.String = 'View Last Motion';
move_rob.FontWeight = 'bold';
move_rob.BackgroundColor = [0.8,0.9,1];
move_rob.ForegroundColor = [0.1,0,0.3];
move_rob.FontSize = 12;
move_rob.Callback = @visualize_kuka_mm_test2;

%%%%% Clear All Data Button %%%%%
cl_data = uicontrol(fig1,'Style','pushbutton');
cl_data.Position = [1500 220 150 50];
cl_data.String = 'Clear All Data';
cl_data.FontWeight = 'bold';
cl_data.BackgroundColor = [0.8,0.9,1];
cl_data.ForegroundColor = [0.1,0,0.3];
cl_data.FontSize = 12;
cl_data.Callback = @ClearAllVars;

%%%%% Clear Last Trajectory Button %%%%%
cl_last_data = uicontrol(fig1,'Style','pushbutton');
cl_last_data.Position = [1500 280 150 50];
cl_last_data.String = 'Undo Trajectory';
cl_last_data.FontWeight = 'bold';
cl_last_data.BackgroundColor = [0.8,0.9,1];
cl_last_data.ForegroundColor = [0.1,0,0.3];
cl_last_data.FontSize = 12;
cl_last_data.Callback = @ClearLastTrajVars;

% Keep Selecting Start and End points alternatively.
hold on;
view([90,73]);
hold on;
while 1
    key=0;
    fprintf('Select Start Point\n')
    while key==0
        try
            key = waitforbuttonpress;
        catch
            flag=1;
            break;
        end
    end
    if flag==1
        fprintf('\nSelection Complete\n');
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
    while key==0
        try
            key = waitforbuttonpress;
        catch
            flag=1;
            break;
        end
    end
    if flag==1
        fprintf('\nSelection Complete\n');
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
    
    %Call the curve_fit function
    [spline_pts,spline_normals,spline_faces] = curve_fit(strt_pt,end_pt,mold_v_transformed,mold_f,mold_n_transformed);
    
    % Appending Group Indices complete
    [bx,by,bz] = compute_TCP_new(spline_pts,spline_normals); %compute_TCP function
    xyz_bxbybz = horzcat( spline_pts,bx,by,bz );
    
    if ~isempty(xyz_bxbybz)
        run MAIN.m;
    else
        disp('TRAJECTORY SELCTION IS INCOMPLETE!');
    end
    
end