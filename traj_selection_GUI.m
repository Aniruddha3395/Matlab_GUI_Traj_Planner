%% User-Interface for Selection of Points

global plot_traj_arr;
global xyz_bxbybz_groups;
global joint_angles_group;
global idx;
global group_idx;
global cl_last_data;
global cl_data;
global msgbx;
global prev_traj_status;
global last_success_joint_angles;
global mold_v_transformed;
global mold_f;
global mold_n_transformed;
global strt_pt;
global end_pt;
global offset_val;
global num_of_replica;
global close_win;
global replica_count_red;
global replica_count;
global replica_off_red;
global replica_off;

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
offset_val = 50;
num_of_replica = 0;
replica_count_red = false;

%% Initialize GUI buttons
InitGUIButtons(fig1);

%% Keep Selecting Start and End points alternatively.
hold on;
view([90,73]);
hold on;
while 1
    key=0;
    while key==0
        if replica_count_red
            set(replica_count,'BackgroundColor',[1,0.5,0.5]);
        else
            set(replica_count,'BackgroundColor',[1,1,0.9]);
        end
        if replica_off_red
            set(replica_off,'BackgroundColor',[1,0.5,0.5]);
        else
            set(replica_off,'BackgroundColor',[1,1,0.9]);
        end
        msgbx.String = strcat(prev_traj_status,'SELECT START POINT');
        prev_traj_status = [];
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
        if replica_count_red
            set(replica_count,'BackgroundColor',[1,0.5,0.5]);
        else
            set(replica_count,'BackgroundColor',[1,1,0.9]);
        end
        if replica_off_red
            set(replica_off,'BackgroundColor',[1,0.5,0.5]);
        else
            set(replica_off,'BackgroundColor',[1,1,0.9]);
        end
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