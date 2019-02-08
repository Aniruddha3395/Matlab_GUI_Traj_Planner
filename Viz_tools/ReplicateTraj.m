function ReplicateTraj(~,~,dir)

global strt_pt;
global end_pt;
global mold_v_transformed;
global mold_f;
global mold_n_transformed;
global num_of_replica;
global offset_val;
global cl_last_data;
global cl_data;
global close_win;
global msgbx;
global prev_traj_status;

traj_offset = 0;

if ~isempty(strt_pt)
    if num_of_replica>0
        msgbx.String = 'REPLICATING TRAJECTORY...';
        cl_last_data.Enable = 'off';
        cl_data.Enable = 'off';
        close_win.Enable = 'off';
        pause(0.5);
        for i = 1:num_of_replica
            traj_offset = traj_offset + offset_val;
            
            if dir==1
            [strt_pt_offset,end_pt_offset] = offset_pts(strt_pt,end_pt,traj_offset);
            else    
            [strt_pt_offset,end_pt_offset] = offset_pts(strt_pt,end_pt,-traj_offset);
            end
            
            % Call the curve_fit function
            [spline_pts,spline_normals,~] = curve_fit(strt_pt_offset,end_pt_offset,mold_v_transformed,mold_f,mold_n_transformed);
            
            % Appending Group Indices complete
            [bx,by,bz] = compute_TCP_new(spline_pts,spline_normals); %compute_TCP function
            xyz_bxbybz = horzcat( spline_pts,bx,by,bz );
            
            if ~isempty(xyz_bxbybz)
                xyz_bxbybz(:,1:3) = xyz_bxbybz(:,1:3)./1000;    %mm to m
                run compute_trajectory.m;
            end
        end
    end
end
cl_last_data.Enable = 'on';
cl_data.Enable = 'on';
close_win.Enable = 'on';
msgbx.String = strcat(prev_traj_status,'SELECT START POINT');

end