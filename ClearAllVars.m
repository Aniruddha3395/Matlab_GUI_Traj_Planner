function ClearAllVars(~,~)

global joint_angles joint_angles_group xyz_bxbybz_groups group_idx;
global plot_traj_arr idx;

joint_angles = [];
joint_angles_group = [];
xyz_bxbybz_groups = [];
group_idx = [];
idx = 1;
delete(plot_traj_arr);

end
