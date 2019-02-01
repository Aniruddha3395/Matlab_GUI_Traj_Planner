function WriteDataToFile(~,~)

global group_idx;
global xyz_bxbybz_groups;
global joint_angles_group;
global msgbx;

dlmwrite('data_files/group_Idx.csv',group_idx);
dlmwrite('data_files/xyz_bxbybz.csv',xyz_bxbybz_groups);
dlmwrite('data_files/joint_angles.csv',joint_angles_group);

temp_msgstring = msgbx.String;
msgbx.String = 'TOOLPATH SAVED TO FILE';
pause(2);
msgbx.String = temp_msgstring;

end