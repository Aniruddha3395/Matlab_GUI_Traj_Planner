function ClearAllVars(~,~)

global joint_angles;
global joint_angles_group;
global xyz_bxbybz_groups;
global group_idx;
global plot_traj_arr;
global idx;
global plot_failed_traj_arr;
global strt_pt;
global end_pt;

joint_angles = [];
joint_angles_group = [];
xyz_bxbybz_groups = [];
group_idx = [];
idx = 1;
delete(plot_traj_arr);
delete(plot_failed_traj_arr);
strt_pt = [];
end_pt = [];

end
