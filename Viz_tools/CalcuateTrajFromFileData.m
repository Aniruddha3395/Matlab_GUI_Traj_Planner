function CalcuateTrajFromFileData(~,~)

global file_name_xyz_bxbybz;
global file_name_group_idx;
global dir_xyz_bxbybz;
global dir_group_idx;
global transform_traj_from_file_data;

file_xyz_bxbybz = dlmread(strcat(dir_xyz_bxbybz,file_name_xyz_bxbybz));
file_group_idx = dlmread(strcat(dir_group_idx,file_name_group_idx));

if isempty(file_xyz_bxbybz)
    disp('xyz_bxbybz is empty');
else
    if isempty(file_group_idx)
        disp('group_idx is empty');
    else
        if size(file_xyz_bxbybz,2)~=12
            disp('toolpath points file does not have all x,y,z,bx,by and bz information');
        else
            if size(file_xyz_bxbybz,1)~=file_group_idx(end,end)
                disp('grouping file does not correspond to the given toolpath points file');
            else
                ClearAllVars();
                if get(transform_traj_from_file_data, 'Value')==1
                    [xyz,bx,by,bz] = robot_to_part(file_xyz_bxbybz(:,1:3),...
                        file_xyz_bxbybz(:,4:6),...
                        file_xyz_bxbybz(:,7:9),...
                        file_xyz_bxbybz(:,10:12));
                    file_xyz_bxbybz_T = [xyz,bx,by,bz];
                else
                    file_xyz_bxbybz_T = file_xyz_bxbybz;
                end
                for group_id_counter = 1:size(file_group_idx,1)
                    spline_pts = file_xyz_bxbybz_T(file_group_idx(group_id_counter,1):file_group_idx(group_id_counter,2),1:3);
                    xyz_bxbybz = file_xyz_bxbybz_T(file_group_idx(group_id_counter,1):file_group_idx(group_id_counter,2),:);
                    xyz_bxbybz(:,1:3) = xyz_bxbybz(:,1:3)./1000;
                    run compute_trajectory.m;
                end
            end
        end
    end
end





end