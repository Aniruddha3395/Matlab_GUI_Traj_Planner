%% User-Interface for Selection of Points

% Define object for Data Cursor
dcm_obj = datacursormode(figure(1));
set(dcm_obj,'SnapToDataVertex','off');
strt_pts = [];
end_pts = [];
key=0;
flag = 0;
points=[];
grp=[];
normals = [];
idx = 1;
faces=[];
tool_num = [];
xyz_cba = [];
xyz=[];
cba=[];

%%%%% DONE button %%%%%
close_win = uicontrol(figure(1),'Style','pushbutton');
close_win.Position = [1500 40 150 50];
close_win.String = 'DONE';
close_win.FontWeight = 'bold';
close_win.BackgroundColor = [0.8,0.9,1];
close_win.ForegroundColor = [0.1,0,0.3];
close_win.FontSize = 12;
close_win.Callback = @CloseFigWindow;


% Keep Selecting Start and End points alternatively.
while 1
    key=0;
    fprintf('Select Start Point\n')
    while key==0
        try key = waitforbuttonpress; catch flag=1; break; end
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
    
    key=0;
    fprintf('Select End Point')
    while key==0
        try key = waitforbuttonpress; catch flag=1; break; end
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
    [temp_pts,temp_normals,temp_faces] = curve_fit(strt_pt,end_pt,mold_v,mold_f,mold_n);  %Call the curve_fit function
    faces = [faces;temp_faces]; % Appendng the faces to the matrix
    points = [points;temp_pts]; % Appending the Points in curve
    xyz = [xyz; temp_pts];
    normals = [normals; temp_normals]; %Appending the normals in curve
    
    %Append the Group Indices
    size_points = size(temp_pts);
    idx_start = idx;
    idx = idx + size_points(1,1)-1;
    idx_end = idx;
    range = [idx_start,idx_end];
    grp = [grp;range]; %Append the Group Indices
    
    % Appending Group Indices complete
    [bx,by,bz,strt_idx,end_idx] = compute_TCP(xyz,range,normals); %compute_TCP function
    xyz_cba = [xyz_cba; horzcat( temp_pts,bx,by,bz )];
    plot_tcp_with_idx(bx,by,bz,strt_idx,end_idx,points);
    
    %Plot the Points
    scatter3(points(:,1),points(:,2),points(:,3),200,'*','b'); %Plot the Points
    idx = idx + 1;
end