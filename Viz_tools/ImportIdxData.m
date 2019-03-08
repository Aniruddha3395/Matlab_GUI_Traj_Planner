function ImportIdxData(~,~)

global file_name_xyz_bxbybz;
global dir_group_idx;
global file_name_group_idx;
global gen_traj_from_file_data;

[file_name_group_idx,dir_group_idx] = uigetfile('data_files/*.*');

if file_name_xyz_bxbybz~=0 
    if file_name_group_idx~=0 
        gen_traj_from_file_data.Enable = 'on';
    end
end

end