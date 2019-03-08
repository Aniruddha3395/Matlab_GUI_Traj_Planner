function ImportPtsData(~,~)

global file_name_xyz_bxbybz;
global dir_xyz_bxbybz;
global file_name_group_idx;
global gen_traj_from_file_data;

[file_name_xyz_bxbybz,dir_xyz_bxbybz] = uigetfile('data_files/*.*');

if file_name_xyz_bxbybz~=0 
    if file_name_group_idx~=0
        gen_traj_from_file_data.Enable = 'on';
    end
end

end