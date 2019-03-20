function WriteDataToFile(~,~)

global group_idx;
global xyz_bxbybz_groups;
global xyz_cba_groups;
global joint_angles_group;
global prev_traj_status;

dlmwrite('data_files/group_Idx.csv',group_idx);
dlmwrite('data_files/xyz_bxbybz.csv',xyz_bxbybz_groups);
dlmwrite('data_files/xyz_cba.csv',xyz_cba_groups);
dlmwrite('data_files/joint_angles.csv',joint_angles_group);


prev_traj_status = 'TOOLPATH SAVED TO FILE...';

end